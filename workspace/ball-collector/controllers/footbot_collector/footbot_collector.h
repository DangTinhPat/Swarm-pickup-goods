/*
 * Foot-bot controller for the ball-collection swarm — decentralized version.
 *
 * Swarm algorithms used (all local, no central coordinator):
 *
 * 1. VIRTUAL PHYSICS / ARTIFICIAL POTENTIAL FIELDS
 *    Motion is the sum of force vectors: goal attraction + inter-robot
 *    separation + obstacle repulsion. Separation uses the repulsion-only
 *    part of the generalized Lennard-Jones potential (Spears et al.;
 *    same formulation as the official ARGoS flocking example), computed
 *    from range-and-bearing readings of nearby robots.
 *
 * 2. LOCAL COMMUNICATION (range-and-bearing, gossip)
 *    Every tick each robot broadcasts a 10-byte packet to neighbors in
 *    RAB range: its state, the position of a free ball it currently sees
 *    (recruitment), and the ball it has claimed plus its distance to it
 *    (task allocation). No global blackboard: information spreads only
 *    robot-to-robot.
 *
 * 3. DECENTRALIZED TASK ALLOCATION (greedy claim + yield)
 *    A robot heading to a ball "claims" it in its broadcasts. If two
 *    robots claim the same ball, the farther one yields and goes back to
 *    exploring — resolving conflicts pairwise, without any arbiter.
 *
 * 4. RECRUITMENT
 *    A robot that is carrying (thus cannot pick up) still reports balls
 *    it passes by; free neighbors adopt them as targets.
 *
 * 5. LÉVY WALK EXPLORATION
 *    When idle, robots do a Lévy walk: straight legs whose lengths follow
 *    a heavy-tailed Pareto distribution with random re-orientation —
 *    the optimal search strategy for sparse targets, observed in
 *    biological foragers (bees, albatrosses, ants).
 *
 * LED colors: green = exploring, cyan = going to a claimed ball,
 * red = carrying a ball back to the nest.
 */

#ifndef FOOTBOT_COLLECTOR_H
#define FOOTBOT_COLLECTOR_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CFootBotCollector : public CCI_Controller {

public:

   enum EState {
      STATE_EXPLORE = 0,
      STATE_GO_TO_BALL,
      STATE_RETURN
   };

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
      /* Returns a repulsive (negative) magnitude when closer than
       * TargetDistance, 0 otherwise */
      Real RepulsionOnlyLJ(Real f_distance);
   };

   /* Lévy walk exploration + search dispersion */
   struct SExploreParams {
      Real Alpha;           /* Pareto tail exponent, typically 1.5 */
      Real MinStep;         /* m */
      Real MaxStep;         /* m */
      /* Central-place foraging: searchers are pushed out of the (ball-free)
       * area around the nest */
      Real NestAvoidRadius; /* m */
      Real NestRepelGain;   /* speed units */
      /* Physicomimetics dispersion: searchers repel each other over the
       * whole RAB range so the swarm spreads over the arena */
      Real DisperseGain;    /* speed units */
      Real DisperseRange;   /* cm (matches RAB range) */
      void Init(TConfigurationNode& t_node);
   };

public:

   CFootBotCollector();
   virtual ~CFootBotCollector() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   /* Called by the loop functions (non-physical ball attachment) */
   bool IsCarrying() const { return m_bCarrying; }
   void PickUp();
   void Drop();

   /* Virtual camera, fed by the loop functions each tick: position of the
    * nearest free ball within sight range (world frame) */
   void SetBallSighting(const CVector2& c_pos);

private:

   void UpdatePose();
   /* Vector fields, all in the robot frame, speed units (cm/s) */
   CVector2 VectorToPoint(const CVector2& c_world_target);
   CVector2 SeparationVector();   /* LJ repulsion from RAB neighbors */
   CVector2 ObstacleVector();     /* repulsion from proximity readings */
   CVector2 LevyWalkVector();
   /* Communication */
   void ProcessMessages();
   void Broadcast();
   /* Behavior helpers */
   void StartExploring();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);
   /* Tabu list: balls I yielded to a better-placed robot. While an entry
    * is active I neither re-claim nor adopt that ball, so a yield really
    * removes me from the contest instead of lasting a single tick. */
   bool IsTabu(const CVector2& c_pos) const;
   void AddTabu(const CVector2& c_pos);

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_LEDsActuator*                 m_pcLEDs;
   CCI_RangeAndBearingActuator*      m_pcRABAct;
   CCI_RangeAndBearingSensor*        m_pcRABSens;
   CCI_FootBotProximitySensor*       m_pcProximity;
   CCI_PositioningSensor*            m_pcPosition;
   CRandom::CRNG*                    m_pcRNG;

   SWheelTurningParams m_sWheelTurningParams;
   SSeparationParams   m_sSeparationParams;
   SExploreParams      m_sExploreParams;

   /* Parameters */
   CVector2 m_cNestPos;
   Real     m_fObstacleGain;   /* proximity repulsion strength */
   Real     m_fGiveUpRange;    /* if closer than this to the claimed ball
                                  and still not seeing it, give up (m) */

   /* Pose (from positioning sensor, updated every tick) */
   CVector2 m_cPos;
   CRadians m_cYaw;

   /* State */
   EState   m_eState;
   bool     m_bCarrying;
   CVector2 m_cTarget;         /* claimed ball (world frame) */
   bool     m_bSightingFresh;  /* did the virtual camera fire this tick? */
   CVector2 m_cSighting;
   UInt32   m_unTargetMisses;  /* ticks near the target without seeing it */

   /* Lévy walk state */
   SInt32   m_nWalkTicksLeft;
   CRadians m_cWalkHeading;    /* world frame */

   /* Dispersion force from neighbors (robot frame, built in
    * ProcessMessages, applied only while exploring) */
   CVector2 m_cDisperse;

   /* Task-allocation identity & memory */
   UInt16 m_unRobotId;   /* hash of the robot id, for claim tie-breaking */
   UInt32 m_unTickCount;
   struct STabuEntry {
      CVector2 Pos;
      UInt32   ExpireTick;
   };
   std::vector<STabuEntry> m_tTabu;
};

#endif
