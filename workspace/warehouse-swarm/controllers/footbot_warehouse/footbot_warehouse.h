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

   /* Operator override — the fleet console's authority over one robot.
    * This is a HUMAN command layer on top of the autonomy (like the
    * e-stop / recall functions of a real AMR fleet manager), NOT a
    * central planner: under OP_AUTO (the default, and the only mode a
    * headless run ever sees) the robot is 100% autonomous. */
   enum EOverride {
      OP_AUTO = 0,   /* fully autonomous (default) */
      OP_STOPPED,    /* e-stop: freeze in place, keep broadcasting so
                      * neighbors still avoid it; resumes where it left off */
      OP_RECALL      /* finish the delivery in hand (never abandon a
                      * parcel), then park at a dock bay and stay there */
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
   /* Belt ground truth the controller may only SENSE when close (never a
    * global feed) — see SBeltBelief / SenseBelts. */
   void  SetBeltGroundTruth(const UInt8* pun_queues, const bool* pb_blocked);

   /* ---- Energy status (also read by the QT user functions for the HUD) ---- */
   bool  IsCharging() const { return m_eState == STATE_CHARGE; }
   bool  IsDead() const { return m_eState == STATE_DEAD; }

   /* ---- Operator console (warehouse_qt_user_functions.cpp) ---- */
   void      SetOverride(EOverride e_op);
   EOverride GetOverride() const { return m_eOverride; }
   EState    GetState() const { return m_eState; }
   Real      GetCharge() const { return m_fCharge; }

private:

   /* Constants shared by more than one behavior file live here once, so the
    * copies can't drift; single-file constants stay local to that .cpp. */
   static constexpr UInt32 BELT_INFO_MAX_AGE = 100; /* ticks; older belief = "unknown" */
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
   /* Bootstrap/patrol: with no global oracle, an idle robot must
    * occasionally go LOOK at a belt whose info has aged out, or the fleet
    * stays blind to belts nobody has driven past. Returns true (and fills
    * c_target) when some belt is stale; false when everything is fresh. */
   bool  NeedsBeltPatrol(CVector2& c_target) const;

   /* ==== footbot_task_allocation.cpp — market-based belt bidding ==== */
   void ChooseBelt();

   /* ==== footbot_docking.cpp — dock-slot claiming + stigmergy ==== */
   void ChooseDockSlot();
   /* Shared by the stuck-check and the movement step so they agree on what
    * "heading to dock" means. */
   CVector2 ComputeDockTarget(bool& b_exempt);
   bool SlotIsLeftSide(size_t un_slot) const {
      return m_cDockSlots[un_slot].GetY() > 0.0;
   }
   /* Stigmergy (dock-side congestion avoidance): a fading gray trace on the
    * floor of how busy each dock side has been, sensed via the real ground
    * sensor — inherently local, so it only sways a robot already near a
    * side (like ant nest-site rejection). */
   Real GroundDarkness() const;

   /* ==== footbot_rescue.cpp — stuck detection / 2-level escape ==== */
   /* Tracks distance-TO-GOAL (not raw displacement, which can't tell a
    * wedged/orbiting robot from a moving one): if it fails to shrink over
    * two rolling 3 s windows, escalate — Level 1 a 1.5 s random-heading
    * nudge, Level 2 (after repeated failure) a reverse-then-drive that
    * broadcasts a "rescuing" flag so neighbors yield. Returns true if it
    * took the wheels this tick (caller should Broadcast + return). */
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
   EOverride m_eOverride;  /* operator command layer, OP_AUTO by default */
   SInt8  m_nCarryAddr;    /* -1 = empty-handed */
   SInt8  m_nBeltChoice;   /* -1 = none */
   UInt8  m_unInbound[NUM_BELTS];  /* neighbors heading to each belt */
   /* A belt commitment cannot change again until this tick (anti-dither) */
   UInt32 m_unDecisionLockUntil;

   /* Belt status: local sensing + gossip relay (footbot_comms.cpp).
    * Trustworthy only while BeltAge() < BELT_INFO_MAX_AGE. Stored as an
    * absolute OriginTick (not a mutable "ticks-old" counter): age is always
    * derived as now - OriginTick and OriginTick only ever moves forward,
    * which keeps gossip immune to mutual-reinforcement staleness loops. */
   struct SBeltBelief {
      UInt8  Queue;
      bool   Blocked;
      UInt32 OriginTick;   /* 0 = never known */
   };
   SBeltBelief m_tBeltBelief[NUM_BELTS];
   /* Ground truth from the loop functions; copied into the belief only
    * when the robot is close enough to sense it (SenseBelts). */
   UInt8 m_unTrueQueue[NUM_BELTS];
   bool  m_bTrueBlocked[NUM_BELTS];
   Real  m_fBeltSenseRange;
   /* Patrol cooldown: after one scouting trip a surplus robot rests parked
    * (does not un-park to patrol) until this tick, so idle robots settle at
    * the dock instead of shuffling belt-to-belt forever. */
   UInt32 m_unPatrolHoldUntil;
   bool PatrolReady() const { return m_unTickCount >= m_unPatrolHoldUntil; }

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
