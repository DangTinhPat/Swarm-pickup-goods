#include "footbot_collector.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <functional>

/****************************************/
/****************************************/

/*
 * RAB packet layout (12 bytes):
 *   [0]     UInt8  state (EState)
 *   [1-2]   UInt16 sender id (hash), used to break claim ties
 *   [3-6]   SInt16 sighting x, y (cm, world frame; NO_VALUE = none)
 *           A free ball the sender currently sees (recruitment).
 *   [7-10]  SInt16 claim x, y (cm, world frame; NO_VALUE = none)
 *           The ball the sender is going to (task allocation).
 *   [11]    UInt8  sender's distance to its claim, in 4 cm units, capped.
 */
static const SInt16 NO_VALUE = 0x7FFF;
static const size_t PACKET_SIZE = 12;
/* Two claims closer than this refer to the same ball (m) */
static const Real CLAIM_MATCH_RADIUS = 0.25;
/* Distances closer than this count as a tie -> lower id wins (m) */
static const Real CLAIM_TIE_MARGIN = 0.05;
/* Tabu entries: match radius (m) and lifetime (ticks) */
static const Real   TABU_RADIUS = 0.3;
static const UInt32 TABU_TICKS  = 100;

/****************************************/
/****************************************/

void CFootBotCollector::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   CDegrees cAngle;
   GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
   HardTurnOnAngleThreshold = ToRadians(cAngle);
   GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
   SoftTurnOnAngleThreshold = ToRadians(cAngle);
   GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
   NoTurnAngleThreshold = ToRadians(cAngle);
   GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   TurningMechanism = NO_TURN;
}

/****************************************/
/****************************************/

void CFootBotCollector::SSeparationParams::Init(TConfigurationNode& t_node) {
   GetNodeAttribute(t_node, "target_distance", TargetDistance);
   GetNodeAttribute(t_node, "gain", Gain);
   GetNodeAttributeOrDefault(t_node, "exponent", Exponent, 2.0);
}

Real CFootBotCollector::SSeparationParams::RepulsionOnlyLJ(Real f_distance) {
   if(f_distance >= TargetDistance) return 0.0;
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   /* Generalized Lennard-Jones; below TargetDistance this is negative,
    * i.e. it pushes away along the bearing to the neighbor */
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

void CFootBotCollector::SExploreParams::Init(TConfigurationNode& t_node) {
   GetNodeAttributeOrDefault(t_node, "levy_alpha", Alpha, 1.5);
   GetNodeAttributeOrDefault(t_node, "min_step", MinStep, 0.6);
   GetNodeAttributeOrDefault(t_node, "max_step", MaxStep, 3.0);
   GetNodeAttributeOrDefault(t_node, "nest_avoid_radius", NestAvoidRadius, 1.2);
   GetNodeAttributeOrDefault(t_node, "nest_repel_gain", NestRepelGain, 15.0);
   GetNodeAttributeOrDefault(t_node, "disperse_gain", DisperseGain, 8.0);
   GetNodeAttributeOrDefault(t_node, "disperse_range", DisperseRange, 150.0);
}

/****************************************/
/****************************************/

CFootBotCollector::CFootBotCollector() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABAct(NULL),
   m_pcRABSens(NULL),
   m_pcProximity(NULL),
   m_pcPosition(NULL),
   m_pcRNG(NULL),
   m_fObstacleGain(30.0),
   m_fGiveUpRange(0.7),
   m_eState(STATE_EXPLORE),
   m_bCarrying(false),
   m_bSightingFresh(false),
   m_unTargetMisses(0),
   m_nWalkTicksLeft(0) {
}

/****************************************/
/****************************************/

void CFootBotCollector::Init(TConfigurationNode& t_node) {
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLEDs      = GetActuator<CCI_LEDsActuator>("leds");
   m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcRABSens   = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   m_pcPosition  = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcRNG       = CRandom::CreateRNG("argos");

   m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   m_sSeparationParams.Init(GetNode(t_node, "separation"));
   m_sExploreParams.Init(GetNode(t_node, "explore"));

   TConfigurationNode& tNav = GetNode(t_node, "navigation");
   GetNodeAttribute(tNav, "nest", m_cNestPos);
   GetNodeAttributeOrDefault(tNav, "obstacle_gain", m_fObstacleGain, m_fObstacleGain);
   GetNodeAttributeOrDefault(tNav, "give_up_range", m_fGiveUpRange, m_fGiveUpRange);

   /* Stable per-robot id for deterministic claim tie-breaking */
   m_unRobotId = std::hash<std::string>{}(GetId()) & 0xFFFF;

   Reset();
}

/****************************************/
/****************************************/

void CFootBotCollector::Reset() {
   m_eState = STATE_EXPLORE;
   m_bCarrying = false;
   m_bSightingFresh = false;
   m_unTargetMisses = 0;
   m_nWalkTicksLeft = 0;
   m_unTickCount = 0;
   m_tTabu.clear();
   m_pcLEDs->SetAllColors(CColor::GREEN);
   m_pcRABAct->ClearData();
}

/****************************************/
/****************************************/

bool CFootBotCollector::IsTabu(const CVector2& c_pos) const {
   for(size_t i = 0; i < m_tTabu.size(); ++i) {
      if((m_tTabu[i].Pos - c_pos).Length() < TABU_RADIUS) return true;
   }
   return false;
}

void CFootBotCollector::AddTabu(const CVector2& c_pos) {
   m_tTabu.push_back({c_pos, m_unTickCount + TABU_TICKS});
}

/****************************************/
/****************************************/

void CFootBotCollector::PickUp() {
   m_bCarrying = true;
   m_eState = STATE_RETURN;
   m_unTargetMisses = 0;
}

void CFootBotCollector::Drop() {
   m_bCarrying = false;
   StartExploring();
}

void CFootBotCollector::SetBallSighting(const CVector2& c_pos) {
   m_bSightingFresh = true;
   m_cSighting = c_pos;
}

void CFootBotCollector::StartExploring() {
   m_eState = STATE_EXPLORE;
   m_nWalkTicksLeft = 0; /* force a new Lévy leg */
}

/****************************************/
/****************************************/

void CFootBotCollector::UpdatePose() {
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();
   m_cPos.Set(sReading.Position.GetX(), sReading.Position.GetY());
   CRadians cPitch, cRoll;
   sReading.Orientation.ToEulerAngles(m_cYaw, cPitch, cRoll);
}

/****************************************/
/****************************************/

void CFootBotCollector::ControlStep() {
   UpdatePose();
   ++m_unTickCount;

   /* Forget expired tabu entries */
   for(size_t i = 0; i < m_tTabu.size(); ) {
      if(m_unTickCount >= m_tTabu[i].ExpireTick) {
         m_tTabu.erase(m_tTabu.begin() + i);
      }
      else ++i;
   }

   /* 1. Listen to neighbors: recruitment + claim conflict resolution */
   ProcessMessages();

   /* 2. Own (virtual) camera: claim the ball I can see — unless I have
    * yielded it to a better-placed robot (tabu) */
   if(!m_bCarrying && m_bSightingFresh && IsTabu(m_cSighting)) {
      m_bSightingFresh = false;
   }
   if(!m_bCarrying && m_bSightingFresh) {
      if(m_eState != STATE_GO_TO_BALL ||
         (m_cSighting - m_cPos).SquareLength() < (m_cTarget - m_cPos).SquareLength() ||
         (m_cSighting - m_cTarget).Length() < CLAIM_MATCH_RADIUS) {
         m_cTarget = m_cSighting;
         m_eState = STATE_GO_TO_BALL;
         m_unTargetMisses = 0;
      }
   }

   /* 3. Give up a claim that no longer pays off: I am close enough that
    * I should see the ball, but I don't (someone else took it) */
   if(m_eState == STATE_GO_TO_BALL &&
      (m_cTarget - m_cPos).Length() < m_fGiveUpRange) {
      if(m_bSightingFresh && (m_cSighting - m_cTarget).Length() < CLAIM_MATCH_RADIUS) {
         m_unTargetMisses = 0;
      }
      else if(++m_unTargetMisses > 5) {
         StartExploring();
      }
   }

   /* 4. Potential-field motion: goal + separation + obstacle repulsion */
   CVector2 cGoal;
   switch(m_eState) {
      case STATE_RETURN:
         cGoal = VectorToPoint(m_cNestPos);
         m_pcLEDs->SetAllColors(CColor::RED);
         break;
      case STATE_GO_TO_BALL:
         cGoal = VectorToPoint(m_cTarget);
         m_pcLEDs->SetAllColors(CColor::CYAN);
         break;
      case STATE_EXPLORE:
      default:
         /* Dispersion only while searching: spread out over the arena */
         cGoal = LevyWalkVector() + m_cDisperse;
         m_pcLEDs->SetAllColors(CColor::GREEN);
         break;
   }
   SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector());

   /* 5. Tell the neighborhood what I know */
   Broadcast();

   /* The camera reading is only valid for one tick */
   m_bSightingFresh = false;
}

/****************************************/
/****************************************/

void CFootBotCollector::ProcessMessages() {
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
   m_cDisperse = CVector2();
   for(size_t i = 0; i < tPackets.size(); ++i) {
      /* Search dispersion: a gentle long-range push away from every
       * neighbor, fading linearly with distance. Applied only while
       * exploring (see ControlStep) so the swarm spreads over the arena
       * instead of crowding one spot. */
      Real fWeight = 1.0 - tPackets[i].Range / m_sExploreParams.DisperseRange;
      if(fWeight > 0.0) {
         m_cDisperse -= CVector2(m_sExploreParams.DisperseGain * fWeight,
                                 tPackets[i].HorizontalBearing);
      }
      CByteArray cData = tPackets[i].Data;
      UInt8 unState;
      UInt16 unSenderId;
      SInt16 nSightX, nSightY, nClaimX, nClaimY;
      UInt8 unClaimDist;
      cData >> unState >> unSenderId
            >> nSightX >> nSightY >> nClaimX >> nClaimY >> unClaimDist;

      /* RECRUITMENT: a carrying robot reports balls it cannot take.
       * If I am free and idle, adopt the sighting as my target —
       * unless I already yielded that ball to someone else. */
      if(!m_bCarrying && m_eState == STATE_EXPLORE &&
         unState == STATE_RETURN && nSightX != NO_VALUE) {
         CVector2 cSight(nSightX / 100.0, nSightY / 100.0);
         if(!IsTabu(cSight)) {
            m_cTarget = cSight;
            m_eState = STATE_GO_TO_BALL;
            m_unTargetMisses = 0;
         }
      }

      /* TASK ALLOCATION: if a neighbor claims my ball, the loser of the
       * comparison yields — closer robot wins; near-ties are broken
       * deterministically by id, so exactly one robot backs off. */
      if(m_eState == STATE_GO_TO_BALL && nClaimX != NO_VALUE) {
         CVector2 cTheirClaim(nClaimX / 100.0, nClaimY / 100.0);
         if((cTheirClaim - m_cTarget).Length() < CLAIM_MATCH_RADIUS) {
            Real fTheirDist = unClaimDist * 0.04;
            Real fMyDist = (m_cTarget - m_cPos).Length();
            bool bYield =
               fTheirDist < fMyDist - CLAIM_TIE_MARGIN ||
               (Abs(fTheirDist - fMyDist) <= CLAIM_TIE_MARGIN &&
                unSenderId < m_unRobotId);
            if(bYield) {
               /* Remember the loss (so my own camera does not make me
                * re-claim it next tick) and walk away from the ball */
               AddTabu(m_cTarget);
               CVector2 cAway = m_cPos - m_cTarget;
               StartExploring();
               m_cWalkHeading = cAway.Angle();
               m_nWalkTicksLeft =
                  Max<SInt32>(1, 1.0 / (m_sWheelTurningParams.MaxSpeed / 1000.0));
            }
         }
      }
   }
}

/****************************************/
/****************************************/

void CFootBotCollector::Broadcast() {
   CByteArray cData;
   cData << (UInt8)m_eState;
   cData << m_unRobotId;
   /* Share a ball I can see right now (world frame, cm) */
   if(m_bSightingFresh) {
      cData << (SInt16)(m_cSighting.GetX() * 100.0)
            << (SInt16)(m_cSighting.GetY() * 100.0);
   }
   else {
      cData << NO_VALUE << NO_VALUE;
   }
   /* Advertise my claim */
   if(m_eState == STATE_GO_TO_BALL) {
      cData << (SInt16)(m_cTarget.GetX() * 100.0)
            << (SInt16)(m_cTarget.GetY() * 100.0);
      Real fDist = (m_cTarget - m_cPos).Length();
      cData << (UInt8)Min<Real>(fDist / 0.04, 255.0);
   }
   else {
      cData << NO_VALUE << NO_VALUE << (UInt8)255;
   }
   m_pcRABAct->SetData(cData);
}

/****************************************/
/****************************************/

CVector2 CFootBotCollector::VectorToPoint(const CVector2& c_world_target) {
   /* World-frame direction, rotated into the robot frame */
   CRadians cAngle = ((c_world_target - m_cPos).Angle() - m_cYaw).SignedNormalize();
   return CVector2(m_sWheelTurningParams.MaxSpeed, cAngle);
}

/****************************************/
/****************************************/

CVector2 CFootBotCollector::SeparationVector() {
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
   if(tPackets.empty()) return CVector2();
   CVector2 cAccum;
   for(size_t i = 0; i < tPackets.size(); ++i) {
      Real fLJ = m_sSeparationParams.RepulsionOnlyLJ(tPackets[i].Range);
      cAccum += CVector2(fLJ, tPackets[i].HorizontalBearing);
   }
   /* Cap so separation never fully overrides the goal (chaos control) */
   Real fCap = 0.6 * m_sWheelTurningParams.MaxSpeed;
   if(cAccum.Length() > fCap) {
      cAccum.Normalize();
      cAccum *= fCap;
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotCollector::ObstacleVector() {
   const CCI_FootBotProximitySensor::TReadings& tReadings = m_pcProximity->GetReadings();
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   cAccum /= tReadings.size();
   /* Push away from the obstacle, proportionally to how close it is */
   return CVector2(-m_fObstacleGain * cAccum.Length(), cAccum.Angle());
}

/****************************************/
/****************************************/

CVector2 CFootBotCollector::LevyWalkVector() {
   Real fNestDist = (m_cPos - m_cNestPos).Length();
   if(m_nWalkTicksLeft <= 0) {
      /* New Lévy leg: heavy-tailed (Pareto) length */
      Real fU = m_pcRNG->Uniform(CRange<Real>(0.001, 1.0));
      Real fLen = m_sExploreParams.MinStep * ::pow(fU, -1.0 / m_sExploreParams.Alpha);
      fLen = Min(fLen, m_sExploreParams.MaxStep);
      /* speed cm/s -> m per tick (10 ticks/s) */
      Real fMetersPerTick = m_sWheelTurningParams.MaxSpeed / 1000.0;
      m_nWalkTicksLeft = Max<SInt32>(1, fLen / fMetersPerTick);
      if(fNestDist < m_sExploreParams.NestAvoidRadius) {
         /* Central-place departure: near the nest (where balls never
          * spawn) head outward, within a +/-60 degree cone */
         m_cWalkHeading = (m_cPos - m_cNestPos).Angle() +
                          CRadians(m_pcRNG->Uniform(CRange<Real>(-1.0, 1.0)));
      }
      else {
         m_cWalkHeading = CRadians(m_pcRNG->Uniform(CRange<Real>(-ARGOS_PI, ARGOS_PI)));
      }
   }
   --m_nWalkTicksLeft;
   /* Follow the leg's world-frame heading */
   CVector2 cGoal(m_sWheelTurningParams.MaxSpeed,
                  (m_cWalkHeading - m_cYaw).SignedNormalize());
   /* Nest repulsion field: fades linearly from the nest center out */
   if(fNestDist < m_sExploreParams.NestAvoidRadius) {
      Real fPush = m_sExploreParams.NestRepelGain *
                   (1.0 - fNestDist / m_sExploreParams.NestAvoidRadius);
      cGoal += CVector2(fPush, ((m_cPos - m_cNestPos).Angle() - m_cYaw).SignedNormalize());
   }
   return cGoal;
}

/****************************************/
/****************************************/

void CFootBotCollector::SetWheelSpeedsFromVector(const CVector2& c_heading) {
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

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotCollector, "footbot_collector_controller")
