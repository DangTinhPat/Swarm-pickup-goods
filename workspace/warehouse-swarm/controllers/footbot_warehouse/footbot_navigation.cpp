/*
 * Potential-field steering primitives, reused from the ball-collector core:
 * goal seeking, Lennard-Jones separation (with right-of-way weighting),
 * map-based pillar avoidance, reactive obstacle repulsion, and the
 * differential-drive wheel-speed conversion. ControlStep() (in
 * footbot_warehouse.cpp) sums these vectors each tick; nothing here
 * knows about task/dock/gossip state beyond the neighbor list and
 * priority rank already computed by footbot_comms.cpp.
 */
#include "footbot_warehouse.h"

/****************************************/
/****************************************/

void CFootBotWarehouse::UpdatePose() {
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();
   m_cPos.Set(sReading.Position.GetX(), sReading.Position.GetY());
   CRadians cPitch, cRoll;
   sReading.Orientation.ToEulerAngles(m_cYaw, cPitch, cRoll);
}

/****************************************/
/****************************************/

CVector2 CFootBotWarehouse::VectorToPoint(const CVector2& c_world_target) {
   CRadians cAngle = ((c_world_target - m_cPos).Angle() - m_cYaw).SignedNormalize();
   return CVector2(m_sWheelTurningParams.MaxSpeed, cAngle);
}

/****************************************/
/****************************************/

CVector2 CFootBotWarehouse::SeparationVector() {
   /* Right-of-way separation: when two robots meet, only the lower-
    * priority one swerves (in real warehouses a loaded AMR keeps its
    * line; symmetric mutual dodging wastes both robots' time).
    *   - I outrank the neighbor  -> tiny weight, I barely react
    *   - Neighbor outranks me    -> extra weight, I do all the avoiding
    *   - Same rank               -> lower id yields (deterministic)
    * The weight pair (0.15 / 1.85) sums to 2.0, so the total force in
    * an encounter matches the old symmetric 1.0 + 1.0 scheme. */
   if(m_tNeighbors.empty()) return CVector2();
   UInt8 unMyPrio = PriorityOf((UInt8)m_eState);
   CVector2 cAccum;
   for(size_t i = 0; i < m_tNeighbors.size(); ++i) {
      Real fLJ = m_sSeparationParams.RepulsionOnlyLJ(m_tNeighbors[i].Range);
      Real fWeight;
      /* Deep rescue outranks everything, both ways: a rescuing robot
       * barely reacts to anyone (it needs to drive out decisively), and
       * everyone else treats it as an unavoidable obstacle to clear out
       * of the way for — it did not choose to be there. */
      if(m_bRescuing) {
         fWeight = 0.15;
      }
      else if(m_tNeighbors[i].Rescuing) {
         fWeight = 1.85;
      }
      else {
         UInt8 unTheirPrio = PriorityOf(m_tNeighbors[i].State);
         if(unMyPrio > unTheirPrio) {
            fWeight = 0.15;
         }
         else if(unMyPrio < unTheirPrio) {
            fWeight = 1.85;
         }
         else {
            fWeight = (m_unRobotId < m_tNeighbors[i].Id) ? 1.85 : 0.15;
         }
      }
      cAccum += CVector2(fWeight * fLJ, m_tNeighbors[i].Bearing);
   }
   Real fCap = 1.2 * m_sWheelTurningParams.MaxSpeed;
   if(cAccum.Length() > fCap) {
      cAccum.Normalize();
      cAccum *= fCap;
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotWarehouse::PillarField() {
   /* Map-based avoidance of the structural columns: a radial push away
    * from the pillar plus a tangential "vortex" component with a FIXED
    * handedness — every robot rounds a pillar the same way (like a tiny
    * roundabout), which removes both the dead-ahead stall of pure
    * repulsion and opposite-direction conflicts on the pillar rim. */
   CVector2 cAccum;
   for(size_t p = 0; p < m_cPillars.size(); ++p) {
      CVector2 cToPillar = m_cPillars[p] - m_cPos;
      Real fDist = cToPillar.Length() - m_fPillarRadius;
      if(fDist < m_fPillarRange) {
         Real fMag = 1.2 * m_sWheelTurningParams.MaxSpeed *
                     (1.0 - Max<Real>(fDist, 0.0) / m_fPillarRange);
         CRadians cBearing = (cToPillar.Angle() - m_cYaw).SignedNormalize();
         cAccum += CVector2(-fMag, cBearing);
         cAccum += CVector2(0.8 * fMag, cBearing - CRadians::PI_OVER_TWO);
      }
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotWarehouse::ObstacleVector() {
   const CCI_FootBotProximitySensor::TReadings& tReadings = m_pcProximity->GetReadings();
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   cAccum /= tReadings.size();
   return CVector2(-m_fObstacleGain * cAccum.Length(), cAccum.Angle());
}

/****************************************/
/****************************************/

void CFootBotWarehouse::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* From the official ARGoS flocking example */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   Real fHeadingLength = c_heading.Length();
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }

   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                             m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}
