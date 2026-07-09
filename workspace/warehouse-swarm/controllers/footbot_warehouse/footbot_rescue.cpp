/*
 * Stuck detection and 2-level escalating escape. See the UpdateStuckEscape
 * declaration in footbot_warehouse.h for the full rationale (distance-to-
 * goal tracking vs. raw displacement, why level 2 reverses before it
 * drives). Called from ControlStep() step 2.5 for every active
 * navigation state, not just docking.
 */
#include "footbot_warehouse.h"

/****************************************/
/****************************************/

bool CFootBotWarehouse::UpdateStuckEscape(const CVector2& c_target) {
   m_bRescuing = false;

   /* Continuing a deep rescue: reverse phase first — a straight
    * backward creep pulls the robot off a wedged contact (corner, a
    * neighbor's flank) that turning in place alone cannot clear,
    * because turning can keep re-touching whatever is beside it. */
   if(m_nRescueReverseTicksLeft > 0) {
      --m_nRescueReverseTicksLeft;
      m_bRescuing = true;
      m_pcWheels->SetLinearVelocity(-0.5 * m_sWheelTurningParams.MaxSpeed,
                                    -0.5 * m_sWheelTurningParams.MaxSpeed);
      return true;
   }
   /* Deep rescue, phase 2: drive out on a random heading, longer than
    * the light escape and broadcasting "rescuing" so neighbors yield */
   if(m_nRescueDriveTicksLeft > 0) {
      --m_nRescueDriveTicksLeft;
      m_bRescuing = true;
      CVector2 cGoal(0.7 * m_sWheelTurningParams.MaxSpeed,
                     (m_cRescueHeading - m_cYaw).SignedNormalize());
      SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector() + PillarField());
      return true;
   }
   /* Level-1 light escape: brief random-heading drive */
   if(m_nEscapeTicksLeft > 0) {
      --m_nEscapeTicksLeft;
      CVector2 cGoal(0.7 * m_sWheelTurningParams.MaxSpeed,
                     (m_cEscapeHeading - m_cYaw).SignedNormalize());
      SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector() + PillarField());
      return true;
   }

   Real fCurDist = (c_target - m_cPos).Length();
   /* Sample progress toward the target every 30 ticks (3 s). Tracking
    * DISTANCE-TO-GOAL rather than raw position catches a robot that is
    * moving plenty (jittering, drifting, orbiting a cluster, grinding
    * against a corner) but never actually closing in on its target —
    * a livelock/wedge that pure displacement would miss entirely. */
   if(m_unTickCount - m_unStuckRefTick >= 30) {
      if(m_fStuckRefGoalDist >= 0.0) {
         Real fProgress = m_fStuckRefGoalDist - fCurDist;
         if(fProgress < 0.05) {
            ++m_unStuckStrikes;
         }
         else {
            m_unStuckStrikes = 0;
            m_unEscapeAttempts = 0;   /* real progress: fully healed */
         }
      }
      m_fStuckRefGoalDist = fCurDist;
      m_unStuckRefTick = m_unTickCount;
      /* Two stalled windows in a row (~6 s of no real progress) ->
       * escalate. First couple of times, a light random nudge is
       * usually enough (breaks a 2-robot standoff). If THAT keeps
       * failing (this is the 3rd stall in a row with no progress
       * ever recorded in between), the robot is likely physically
       * wedged (corner, pillar) or buried in a cluster — go deep. */
      if(m_unStuckStrikes >= 2) {
         m_unStuckStrikes = 0;
         ++m_unEscapeAttempts;
         if(m_unEscapeAttempts >= 3) {
            m_unEscapeAttempts = 0;
            m_nRescueReverseTicksLeft = 15;  /* 1.5 s straight back */
            m_nRescueDriveTicksLeft = 30;    /* 3 s drive to clear */
            m_cRescueHeading = CRadians(m_pcRNG->Uniform(CRange<Real>(-ARGOS_PI, ARGOS_PI)));
         }
         else {
            m_nEscapeTicksLeft = 15;
            m_cEscapeHeading = CRadians(m_pcRNG->Uniform(CRange<Real>(-ARGOS_PI, ARGOS_PI)));
         }
      }
   }
   return false;
}
