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
 * Market-based task allocation (decentralized, see footbot_task_allocation.cpp):
 * each robot locally computes a utility per belt from its OWN belief about
 * queue lengths (see footbot_comms.cpp — no facility-wide oracle) and who
 * else is already heading there, and commits to the best one. No
 * dispatcher assigns jobs: load balancing emerges from everyone bidding
 * with the same local rule.
 *
 * One class, split across several .cpp files by behavior so each stays
 * focused and easy to find — this header is the single source of truth
 * for all of CFootBotWarehouse's state and method signatures:
 *   footbot_warehouse.cpp        - lifecycle (Init/Reset) + the ControlStep
 *                                   state-machine orchestrator
 *   footbot_navigation.cpp       - potential-field steering primitives
 *   footbot_rescue.cpp           - stuck-detection / 2-level escape
 *   footbot_comms.cpp            - RAB packets, belt-status gossip
 *   footbot_task_allocation.cpp  - market-based belt bidding, job lifecycle
 *   footbot_docking.cpp          - dock-slot claiming + stigmergy bias
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
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
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

   /* ---- Job lifecycle (called by the loop functions; implemented in
    * footbot_task_allocation.cpp) ---- */
   bool  IsCarrying() const { return m_nCarryAddr >= 0; }
   SInt8 GetCarriedAddress() const { return m_nCarryAddr; }
   /* A parcel with destination un_addr is handed over at the belt */
   void  AssignItem(UInt8 un_addr);
   /* The parcel was accepted at its address zone */
   void  Deliver();
   /* Eligible for a parcel handover at the bin? Uses the lower
    * keep-working floor, not the new-job bar: a robot that committed
    * legitimately and dipped a little en route may still take its
    * parcel instead of camping at the bin. */
   bool  WantsWork() const;

   /* ---- Belt status (footbot_comms.cpp) ---- */
   /* Ground truth for the belts, handed to the controller so it can
    * SENSE it (only trusted when physically close — see SBeltBelief);
    * this is what the environment "would show" a real proximity/camera
    * sensor, not a global broadcast the robot is allowed to always use. */
   void  SetBeltGroundTruth(const UInt8* pun_queues, const bool* pb_blocked);

   /* ---- Energy status (also read by the QT user functions for the HUD) ---- */
   bool  IsCharging() const { return m_eState == STATE_CHARGE; }
   bool  IsDead() const { return m_eState == STATE_DEAD; }

private:

   /* Tuning constants shared across more than one behavior file — kept
    * here, once, so the copies used by ControlStep/ProcessMessages/
    * ComputeDockTarget (DOCKED_DIST) and by NeedsBeltPatrol/ChooseBelt
    * (BELT_INFO_MAX_AGE) can never drift out of sync. Constants used by
    * only a single .cpp stay local to that file instead (e.g. the
    * market-utility weights in footbot_task_allocation.cpp). */
   static constexpr UInt32 BELT_INFO_MAX_AGE = 100; /* ticks (~10s @ 10 ticks/s); belief older than this is "unknown" */
   static constexpr Real   DOCKED_DIST = 0.05;      /* m; "truly parked" cutoff */

   /* ==== footbot_navigation.cpp — potential-field steering ==== */
   void UpdatePose();
   CVector2 VectorToPoint(const CVector2& c_world_target);
   CVector2 SeparationVector();
   CVector2 ObstacleVector();
   CVector2 PillarField();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /* ==== footbot_comms.cpp — RAB packets + belt-status gossip ==== */
   void ProcessMessages();
   void Broadcast();
   static UInt8 PriorityOf(UInt8 un_state);
   /* Age in ticks since OriginTick; a huge value if never known */
   UInt32 BeltAge(UInt32 un_belt) const;
   void  SenseBelts();
   /* Bootstrap/patrol: with NO global oracle, a robot that has never
    * been near a belt (or whose info aged out) has no way to ever learn
    * about it purely by sitting at the dock — it must occasionally go
    * LOOK, the same way a real idle AMR fleet patrols stations rather
    * than waiting for a call that will never come. Returns true (and
    * fills c_target) when some belt's info is stale; false when
    * everything is fresh, so normal dock-idling resumes. */
   bool  NeedsBeltPatrol(CVector2& c_target) const;

   /* ==== footbot_task_allocation.cpp — market-based belt bidding ==== */
   void ChooseBelt();

   /* ==== footbot_docking.cpp — dock-slot claiming + stigmergy ==== */
   void ChooseDockSlot();
   /* Shared by step 2.5 (stuck-check target) and step 4 (movement) so
    * they can never disagree on what "heading to dock" currently means */
   CVector2 ComputeDockTarget(bool& b_exempt);
   /* Which physical side a slot belongs to (dock rows are at y>0 and
    * y<0 — see the facility map) */
   bool SlotIsLeftSide(size_t un_slot) const {
      return m_cDockSlots[un_slot].GetY() > 0.0;
   }
   /* Stigmergy (dock-side congestion avoidance): the environment itself
    * carries a fading trace of how much each dock side has been used
    * recently (painted as a gray tint near each row, see the loop
    * functions), sensed through the real ground sensor — not a number
    * beamed into the controller. It only ever influences a robot that is
    * ALREADY near a side (stigmergy is inherently local: you can only
    * follow/react to a trail where you currently are), specifically the
    * "waiting for a free slot" moment — reads as "this pile looks busy,
    * let me try the other one", the same rejection behavior seen in ant
    * nest-site selection. */
   Real GroundDarkness() const;

   /* ==== footbot_rescue.cpp — stuck detection / 2-level escape ==== */
   /* TWO ESCALATING LEVELS, applied to every active navigation state
    * (dock approach, belt approach, delivery) — not just docking. A wall
    * corner can cancel the proximity-avoidance vector to near-zero (two
    * walls' pushes nearly opposite each other), and 3+ robots converging
    * on a resource can knot into a cluster that keeps shuffling without
    * ever converging — neither looks like a hard stop, so raw
    * displacement can't detect them. This instead tracks distance-TO-
    * GOAL: if it fails to shrink over a rolling 3 s window, twice in a
    * row (~6 s stuck) escalates:
    *   Level 1 (light):  random-heading drive, 1.5 s.
    *   Level 2 (deep):   after 2 light escapes still fail to help —
    *                     straight reverse (pulls off a wedged contact
    *                     that turning alone can't clear) then a longer
    *                     random-heading drive, 3 s, broadcasting a
    *                     "rescuing" flag so neighbors give it priority. */
   /* Call with the CURRENT navigation target; returns true if it took
    * over the wheels this tick (caller should just Broadcast+return) */
   bool     UpdateStuckEscape(const CVector2& c_target);

   /* ==== Sensors & actuators ==== */
   CCI_DifferentialSteeringActuator*  m_pcWheels;
   CCI_LEDsActuator*                  m_pcLEDs;
   CCI_RangeAndBearingActuator*       m_pcRABAct;
   CCI_RangeAndBearingSensor*         m_pcRABSens;
   CCI_FootBotProximitySensor*        m_pcProximity;
   CCI_PositioningSensor*             m_pcPosition;
   CCI_BatterySensor*                 m_pcBattery;
   CCI_FootBotMotorGroundSensor*      m_pcGround;
   CRandom::CRNG*                     m_pcRNG;

   SWheelTurningParams m_sWheelTurningParams;
   SSeparationParams   m_sSeparationParams;

   /* Facility layout (from XML; a real AMR has the facility map) */
   CVector2 m_cBelt[NUM_BELTS];   /* bin pickup points */
   CVector2 m_cAddr[NUM_ADDRS];   /* address zone centers */
   /* Docking/charging bays: two rows (left & right side of the floor) */
   std::vector<CVector2> m_cDockSlots;
   /* Structural pillars (known from the facility map). The proximity
    * sensor only sees ~10 cm — far too late to route around a column at
    * speed — so navigation-scale avoidance uses the map instead. */
   std::vector<CVector2> m_cPillars;
   Real m_fPillarRadius;
   Real m_fPillarRange;

   /* Motion parameters */
   Real m_fObstacleGain;
   Real m_fHardAvoidThreshold;

   /* Energy policy: flat thresholds (no distance prediction — see
    * ControlStep for why that's safe at this fleet's battery size). */
   Real m_fResumeCharge;        /* leave the charger only at/above this */
   Real m_fMinWorkCharge;       /* minimum charge to accept a NEW job */
   Real m_fHardChargeThreshold; /* below this, go charge, no exceptions */
   Real m_fCharge;              /* latest battery reading, 0..1 */

   /* Pose */
   CVector2 m_cPos;
   CRadians m_cYaw;

   /* State */
   EState m_eState;
   SInt8  m_nCarryAddr;    /* -1 = empty-handed */
   SInt8  m_nBeltChoice;   /* -1 = none */
   UInt8  m_unInbound[NUM_BELTS];  /* neighbors heading to each belt */

   /* Belt status: LOCAL sensing + gossip relay (see footbot_comms.cpp
    * header comment for why this replaces a facility-wide oracle).
    * Queue/Blocked are only trustworthy while the computed age
    * (BeltAge()) is below BELT_INFO_MAX_AGE; ChooseBelt() treats a stale
    * belief as "unknown" and will not bid on it.
    *
    * Stored as an ABSOLUTE tick (OriginTick: when this was last
    * confirmed true), NOT a mutable "ticks-old" counter — a counter
    * that gets directly overwritten by gossip is vulnerable to mutual
    * reinforcement: if A and B keep hearing each other, each one's
    * fresh local increment gets undone every tick by the other's
    * still-low relayed value, so the belief never actually ages,
    * indefinitely, even with zero real sensing (found by tracing a
    * parked robot whose age stayed 0 for 80+ ticks while never once
    * sensing anything). Deriving age fresh from (now - OriginTick)
    * every time is immune to this: OriginTick only ever moves forward
    * (adopt a gossiped one only if it's MORE recent than mine), so the
    * computed age always reflects true elapsed simulation time. */
   struct SBeltBelief {
      UInt8  Queue;
      bool   Blocked;
      UInt32 OriginTick;   /* 0 = never known */
   };
   SBeltBelief m_tBeltBelief[NUM_BELTS];
   /* Ground truth handed in by the loop functions (see
    * SetBeltGroundTruth); only copied into the belief above when the
    * robot is physically close enough (SenseBelts) */
   UInt8 m_unTrueQueue[NUM_BELTS];
   bool  m_bTrueBlocked[NUM_BELTS];
   Real  m_fBeltSenseRange;

   /* Neighbors heard this tick, kept for priority-aware separation */
   struct SNeighbor {
      Real     Range;    /* cm */
      CRadians Bearing;
      UInt8    State;
      UInt16   Id;
      bool     Rescuing;
   };
   std::vector<SNeighbor> m_tNeighbors;

   /* Sticky avoidance turn direction: chosen when an obstacle first
    * triggers the hard-avoid reflex and kept until it clears, so a
    * dead-ahead pillar cannot make the robot dither left-right forever */
   SInt8 m_nAvoidSign;   /* 0 = free, +1 turn left, -1 turn right */

   /* Docking */
   SInt8 m_nDockSlot;                    /* my claimed slot, -1 = none */
   std::vector<bool> m_bSlotTaken;       /* claims heard from neighbors */

   /* Stuck escape state (see footbot_rescue.cpp) */
   Real     m_fStuckRefGoalDist;
   UInt32   m_unStuckRefTick;
   UInt8    m_unStuckStrikes;
   UInt8    m_unEscapeAttempts;
   SInt32   m_nEscapeTicksLeft;
   CRadians m_cEscapeHeading;
   SInt32   m_nRescueReverseTicksLeft;
   SInt32   m_nRescueDriveTicksLeft;
   CRadians m_cRescueHeading;
   bool     m_bRescuing;

   UInt16 m_unRobotId;
   UInt32 m_unTickCount;
};

#endif
