/*
 * Foot-bot controller for the warehouse swarm — decentralized AMR.
 *
 * Scenario: 3 conveyor belts (east wall) continuously drop parcels into
 * their bins; each parcel is labeled with a destination address a..e.
 * Robots pick parcels up at the belts and deliver them to the matching
 * address zone A..E (west wall). Idle robots wait at the depot (center).
 *
 * Swarm mechanisms reused from the ball-collector core:
 * - Artificial potential fields: goal + Lennard-Jones separation +
 *   obstacle repulsion, with the two-stage hard-avoidance override.
 * - Local RAB communication.
 *
 * New mechanism — MARKET-BASED TASK ALLOCATION (decentralized):
 * Belt queue lengths are broadcast facility-wide (like a warehouse
 * WMS/IoT feed — public information). Each robot also hears, via RAB,
 * which belt its neighbors are heading to. It then locally computes a
 * utility for each belt:
 *      U(b) = w_q * queue(b) - w_in * inbound(b) - w_d * distance(b)
 * and commits to the best one (with hysteresis to avoid dithering).
 * No dispatcher assigns jobs: load balancing emerges from everyone
 * bidding with the same local rule.
 */

#ifndef FOOTBOT_WAREHOUSE_H
#define FOOTBOT_WAREHOUSE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>

using namespace argos;

class CFootBotWarehouse : public CCI_Controller {

public:

   enum EState {
      STATE_IDLE = 0,   /* no work: park (and top up) at the dock */
      STATE_TO_BELT,    /* committed to a belt, driving to its bin */
      STATE_DELIVER,    /* carrying a parcel to its address zone */
      STATE_CHARGE,     /* battery low: claim a charging bay until recharged */
      STATE_DEAD        /* battery empty: bricked in place */
   };

   static const UInt32 NUM_BELTS = 3;
   static const UInt32 NUM_ADDRS = 5;

   /* Wheel turning (from the official flocking example) */
   struct SWheelTurningParams {
      enum ETurningMechanism {
         NO_TURN = 0,
         SOFT_TURN,
         HARD_TURN
      } TurningMechanism;
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      Real MaxSpeed;
      void Init(TConfigurationNode& t_node);
   };

   /* Lennard-Jones separation (repulsion-only) */
   struct SSeparationParams {
      Real TargetDistance; /* cm */
      Real Gain;
      Real Exponent;
      void Init(TConfigurationNode& t_node);
      Real RepulsionOnlyLJ(Real f_distance);
   };

public:

   CFootBotWarehouse();
   virtual ~CFootBotWarehouse() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   /* Called by the loop functions */
   bool  IsCarrying() const { return m_nCarryAddr >= 0; }
   SInt8 GetCarriedAddress() const { return m_nCarryAddr; }
   /* A parcel with destination un_addr is handed over at the belt */
   void  AssignItem(UInt8 un_addr);
   /* The parcel was accepted at its address zone */
   void  Deliver();
   /* Facility-wide broadcast of the three belt queue lengths (WMS feed) */
   void  SetBeltQueues(const UInt8* pun_queues);
   /* Energy status (also read by the QT user functions for the HUD) */
   bool  IsCharging() const { return m_eState == STATE_CHARGE; }
   bool  IsDead() const { return m_eState == STATE_DEAD; }
   /* Eligible for a parcel handover at the bin? Uses the lower
    * keep-working floor, not the new-job bar: a robot that committed
    * legitimately and dipped a little en route may still take its
    * parcel instead of camping at the bin. */
   bool  WantsWork() const;

private:

   void UpdatePose();
   CVector2 VectorToPoint(const CVector2& c_world_target);
   CVector2 SeparationVector();
   CVector2 ObstacleVector();
   void ProcessMessages();
   void Broadcast();
   /* Market-based belt selection (see header comment) */
   void ChooseBelt();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_LEDsActuator*                 m_pcLEDs;
   CCI_RangeAndBearingActuator*      m_pcRABAct;
   CCI_RangeAndBearingSensor*        m_pcRABSens;
   CCI_FootBotProximitySensor*       m_pcProximity;
   CCI_PositioningSensor*            m_pcPosition;
   CCI_BatterySensor*                m_pcBattery;
   CRandom::CRNG*                    m_pcRNG;

   SWheelTurningParams m_sWheelTurningParams;
   SSeparationParams   m_sSeparationParams;

   /* Facility layout (from XML; a real AMR has the facility map) */
   CVector2 m_cBelt[NUM_BELTS];   /* bin pickup points */
   CVector2 m_cAddr[NUM_ADDRS];   /* address zone centers */
   /* Docking/charging bays: two rows (left & right side of the floor) */
   std::vector<CVector2> m_cDockSlots;
   /* Distance to the nearest bay — used by the energy planner */
   Real NearestDockDist() const;
   /* Structural pillars (known from the facility map). The proximity
    * sensor only sees ~10 cm — far too late to route around a column at
    * speed — so navigation-scale avoidance uses the map instead. */
   std::vector<CVector2> m_cPillars;
   Real m_fPillarRadius;
   Real m_fPillarRange;
   CVector2 PillarField();

   /* Motion parameters */
   Real m_fObstacleGain;
   Real m_fHardAvoidThreshold;

   /* Predictive energy management: the robot knows its own consumption
    * model (from the fleet spec in XML) and goes charging exactly when
    * the remaining charge approaches what it needs to reach a charging
    * bay — with a safety factor — instead of a blind threshold. */
   Real m_fDrainPerMeter;   /* charge fraction spent per meter driven */
   Real m_fDrainPerTick;    /* charge fraction spent per tick (idle) */
   Real m_fSafetyFactor;    /* multiplier on the predicted need */
   Real m_fReserveCharge;   /* untouchable emergency reserve */
   Real m_fResumeCharge;    /* leave the charger at this level */
   Real m_fMinWorkCharge;   /* minimum charge to accept a NEW job */
   Real m_fHardChargeThreshold; /* below this, go charge no matter what */
   Real m_fCharge;          /* latest battery reading, 0..1 */
   Real EnergyFor(Real f_distance) const;

   /* Pose */
   CVector2 m_cPos;
   CRadians m_cYaw;

   /* State */
   EState m_eState;
   SInt8  m_nCarryAddr;    /* -1 = empty-handed */
   SInt8  m_nBeltChoice;   /* -1 = none */
   UInt8  m_unQueues[NUM_BELTS];   /* WMS feed, refreshed every tick */
   UInt8  m_unInbound[NUM_BELTS];  /* neighbors heading to each belt */

   /* Neighbors heard this tick, kept for priority-aware separation */
   struct SNeighbor {
      Real     Range;    /* cm */
      CRadians Bearing;
      UInt8    State;
      UInt16   Id;
   };
   std::vector<SNeighbor> m_tNeighbors;
   /* Right-of-way rank: loaded > fetching > idle */
   static UInt8 PriorityOf(UInt8 un_state);

   /* Sticky avoidance turn direction: chosen when an obstacle first
    * triggers the hard-avoid reflex and kept until it clears, so a
    * dead-ahead pillar cannot make the robot dither left-right forever */
   SInt8 m_nAvoidSign;   /* 0 = free, +1 turn left, -1 turn right */

   /* Docking */
   SInt8 m_nDockSlot;                    /* my claimed slot, -1 = none */
   std::vector<bool> m_bSlotTaken;       /* claims heard from neighbors */
   void ChooseDockSlot();

   UInt16 m_unRobotId;
   UInt32 m_unTickCount;
};

#endif
