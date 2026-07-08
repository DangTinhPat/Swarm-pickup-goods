#include "footbot_warehouse.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <functional>
#include <algorithm>

/****************************************/
/****************************************/

/*
 * RAB packet layout (7 bytes):
 *   [0]   UInt8  state (EState)
 *   [1-2] UInt16 sender id (hash)
 *   [3]   UInt8  belt the sender is heading to (255 = none)
 *   [4]   UInt8  sender's distance to that belt, in 4 cm units, capped
 *   [5]   UInt8  address of the carried parcel (255 = none)
 *   [6]   UInt8  dock slot the sender claims (255 = none)
 */
static const UInt8 NO_BYTE = 255;
/* Docked when closer to the slot than this; re-approach if pushed out */
static const Real DOCKED_DIST = 0.08;
/* Parked robots align to this heading (facing the belts) for neat rows */
static const CRadians DOCK_HEADING = CRadians::ZERO;

/* Utility weights for market-based belt selection */
static const Real W_QUEUE    = 1.5;  /* pull of waiting parcels */
static const Real W_INBOUND  = 1.0;  /* push of colleagues already coming */
static const Real W_DISTANCE = 0.35; /* travel cost, per meter */
static const Real HYSTERESIS = 0.8;  /* keep current belt unless clearly worse */
/* A parked robot leaves its slot only for a clearly worthwhile job; a
 * robot already on the floor takes any non-negative one. This stops the
 * whole dock from surging out at the first parcel and dribbling back. */
static const Real LEAVE_UTILITY = 0.3;

/****************************************/
/****************************************/

void CFootBotWarehouse::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CFootBotWarehouse::SSeparationParams::Init(TConfigurationNode& t_node) {
   GetNodeAttribute(t_node, "target_distance", TargetDistance);
   GetNodeAttribute(t_node, "gain", Gain);
   GetNodeAttributeOrDefault(t_node, "exponent", Exponent, 2.0);
}

Real CFootBotWarehouse::SSeparationParams::RepulsionOnlyLJ(Real f_distance) {
   if(f_distance >= TargetDistance) return 0.0;
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotWarehouse::CFootBotWarehouse() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABAct(NULL),
   m_pcRABSens(NULL),
   m_pcProximity(NULL),
   m_pcPosition(NULL),
   m_pcRNG(NULL),
   m_fObstacleGain(100.0),
   m_fHardAvoidThreshold(0.05),
   m_eState(STATE_IDLE),
   m_nCarryAddr(-1),
   m_nBeltChoice(-1),
   m_nDockSlot(-1),
   m_unTickCount(0) {
}

/****************************************/
/****************************************/

void CFootBotWarehouse::Init(TConfigurationNode& t_node) {
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLEDs      = GetActuator<CCI_LEDsActuator>("leds");
   m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcRABSens   = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   m_pcPosition  = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcRNG       = CRandom::CreateRNG("argos");

   m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   m_sSeparationParams.Init(GetNode(t_node, "separation"));

   TConfigurationNode& tNav = GetNode(t_node, "navigation");
   GetNodeAttributeOrDefault(tNav, "obstacle_gain", m_fObstacleGain, m_fObstacleGain);
   GetNodeAttributeOrDefault(tNav, "hard_avoid_threshold", m_fHardAvoidThreshold, m_fHardAvoidThreshold);

   /* Facility layout: belt bins, address zones, depot */
   TConfigurationNode& tSt = GetNode(t_node, "stations");
   GetNodeAttribute(tSt, "belt0", m_cBelt[0]);
   GetNodeAttribute(tSt, "belt1", m_cBelt[1]);
   GetNodeAttribute(tSt, "belt2", m_cBelt[2]);
   GetNodeAttribute(tSt, "addr_a", m_cAddr[0]);
   GetNodeAttribute(tSt, "addr_b", m_cAddr[1]);
   GetNodeAttribute(tSt, "addr_c", m_cAddr[2]);
   GetNodeAttribute(tSt, "addr_d", m_cAddr[3]);
   GetNodeAttribute(tSt, "addr_e", m_cAddr[4]);
   /* Docking grid: neat rows of parking slots for idle robots */
   UInt32 unRows, unCols;
   Real fSpacing;
   GetNodeAttribute(tSt, "dock_center", m_cDockCenter);
   GetNodeAttribute(tSt, "dock_rows", unRows);
   GetNodeAttribute(tSt, "dock_cols", unCols);
   GetNodeAttribute(tSt, "dock_spacing", fSpacing);
   for(UInt32 r = 0; r < unRows; ++r) {
      for(UInt32 c = 0; c < unCols; ++c) {
         m_cDockSlots.push_back(m_cDockCenter + CVector2(
            (Real)((SInt32)r - ((SInt32)unRows - 1) / 2.0) * fSpacing,
            (Real)((SInt32)c - ((SInt32)unCols - 1) / 2.0) * fSpacing));
      }
   }
   m_bSlotTaken.resize(m_cDockSlots.size(), false);

   m_unRobotId = std::hash<std::string>{}(GetId()) & 0xFFFF;

   Reset();
}

/****************************************/
/****************************************/

void CFootBotWarehouse::Reset() {
   m_eState = STATE_IDLE;
   m_nCarryAddr = -1;
   m_nBeltChoice = -1;
   m_nDockSlot = -1;
   m_unTickCount = 0;
   for(UInt32 i = 0; i < NUM_BELTS; ++i) {
      m_unQueues[i] = 0;
      m_unInbound[i] = 0;
   }
   m_pcLEDs->SetAllColors(CColor::GREEN);
   m_pcRABAct->ClearData();
}

/****************************************/
/****************************************/

void CFootBotWarehouse::AssignItem(UInt8 un_addr) {
   m_nCarryAddr = un_addr;
   m_eState = STATE_DELIVER;
   m_nBeltChoice = -1;
}

void CFootBotWarehouse::Deliver() {
   m_nCarryAddr = -1;
   m_eState = STATE_IDLE;
   m_nBeltChoice = -1;   /* re-bid next tick */
}

void CFootBotWarehouse::SetBeltQueues(const UInt8* pun_queues) {
   for(UInt32 i = 0; i < NUM_BELTS; ++i) {
      m_unQueues[i] = pun_queues[i];
   }
}

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

void CFootBotWarehouse::ControlStep() {
   UpdatePose();
   ++m_unTickCount;

   /* 1. Listen: count colleagues inbound to each belt */
   for(UInt32 i = 0; i < NUM_BELTS; ++i) m_unInbound[i] = 0;
   ProcessMessages();

   /* 2. Bid for work (unless carrying): re-evaluate when uncommitted,
    * and periodically so stale commitments adapt to new information */
   if(m_eState != STATE_DELIVER) {
      /* Re-bid on a staggered schedule (by robot id), including while
       * idle: decisions trickle out one robot at a time instead of the
       * whole dock reacting to the same parcel on the same tick with
       * nobody's claims heard yet. Uncommitted robots check often,
       * committed ones only occasionally. */
      bool bEvaluate = (m_nBeltChoice < 0)
                       ? ((m_unTickCount + m_unRobotId) % 3 == 0)
                       : ((m_unTickCount + m_unRobotId) % 10 == 0);
      if(bEvaluate) {
         ChooseBelt();
      }
      m_eState = (m_nBeltChoice >= 0) ? STATE_TO_BELT : STATE_IDLE;
      /* Got work: release my parking slot */
      if(m_eState == STATE_TO_BELT) {
         m_nDockSlot = -1;
      }
   }

   /* 3. Collision imminent? Two-stage override (same as ball-collector):
    * moving turn above threshold, pivot in place above 3x threshold. */
   {
      const CCI_FootBotProximitySensor::TReadings& tProx = m_pcProximity->GetReadings();
      CVector2 cProxAccum;
      for(size_t i = 0; i < tProx.size(); ++i) {
         cProxAccum += CVector2(tProx[i].Value, tProx[i].Angle);
      }
      cProxAccum /= tProx.size();
      Real fProxLen = cProxAccum.Length();
      if(fProxLen > m_fHardAvoidThreshold) {
         Real fOuter = fProxLen > 3.0 * m_fHardAvoidThreshold
                       ? 0.5 * m_sWheelTurningParams.MaxSpeed
                       : m_sWheelTurningParams.MaxSpeed;
         Real fInner = -0.5 * m_sWheelTurningParams.MaxSpeed;
         if(cProxAccum.Angle() > CRadians::ZERO) {
            m_pcWheels->SetLinearVelocity(fOuter, fInner);
         }
         else {
            m_pcWheels->SetLinearVelocity(fInner, fOuter);
         }
         Broadcast();
         return;
      }
   }

   /* 4. Potential-field motion */
   CVector2 cGoal;
   switch(m_eState) {
      case STATE_DELIVER:
         cGoal = VectorToPoint(m_cAddr[m_nCarryAddr]);
         m_pcLEDs->SetAllColors(CColor::RED);
         break;
      case STATE_TO_BELT:
         cGoal = VectorToPoint(m_cBelt[m_nBeltChoice]);
         /* Approach the bin gently so LJ separation can order the
          * waiting robots instead of them shoving each other */
         if((m_cBelt[m_nBeltChoice] - m_cPos).Length() < 0.6) {
            cGoal *= 0.5;
         }
         m_pcLEDs->SetAllColors(CColor::CYAN);
         break;
      case STATE_IDLE:
      default: {
         /* Dock: claim a free parking slot and park in it, neatly */
         m_pcLEDs->SetAllColors(CColor::GREEN);
         if(m_nDockSlot < 0) {
            ChooseDockSlot();
         }
         if(m_nDockSlot < 0) {
            /* No free slot heard of (transient): hold near the dock */
            cGoal = VectorToPoint(m_cDockCenter);
            cGoal *= 0.3;
            break;
         }
         Real fDist = (m_cDockSlots[m_nDockSlot] - m_cPos).Length();
         if(fDist < DOCKED_DIST) {
            /* Parked. Align to the common heading, then freeze — do NOT
             * follow the potential field, or passing robots would drag
             * the whole row around. The proximity reflex above still
             * protects against real emergencies. */
            CRadians cErr = (DOCK_HEADING - m_cYaw).SignedNormalize();
            if(Abs(cErr) > CRadians(0.15)) {
               Real fTurn = 0.2 * m_sWheelTurningParams.MaxSpeed;
               if(cErr > CRadians::ZERO) m_pcWheels->SetLinearVelocity(-fTurn, fTurn);
               else                      m_pcWheels->SetLinearVelocity(fTurn, -fTurn);
            }
            else {
               m_pcWheels->SetLinearVelocity(0.0, 0.0);
            }
            Broadcast();
            return;
         }
         /* Approach with arrival damping so the robot settles exactly
          * on the slot instead of orbiting it at full speed */
         cGoal = VectorToPoint(m_cDockSlots[m_nDockSlot]);
         cGoal *= Min<Real>(1.0, fDist / 0.4);
         break;
      }
   }
   SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector());

   /* 5. Tell the neighborhood what I am doing */
   Broadcast();
}

/****************************************/
/****************************************/

UInt8 CFootBotWarehouse::PriorityOf(UInt8 un_state) {
   switch(un_state) {
      case STATE_DELIVER: return 2;  /* loaded: right of way */
      case STATE_TO_BELT: return 1;  /* fetching */
      default:            return 0;  /* idle */
   }
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ProcessMessages() {
   m_tNeighbors.clear();
   std::fill(m_bSlotTaken.begin(), m_bSlotTaken.end(), false);
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
   for(size_t i = 0; i < tPackets.size(); ++i) {
      CByteArray cData = tPackets[i].Data;
      UInt8 unState, unBelt, unDist, unAddr, unDock;
      UInt16 unSenderId;
      cData >> unState >> unSenderId >> unBelt >> unDist >> unAddr >> unDock;
      /* Remember the neighbor for priority-aware separation */
      m_tNeighbors.push_back({tPackets[i].Range,
                              tPackets[i].HorizontalBearing,
                              unState, unSenderId});
      /* Dock bookkeeping: mark claimed slots; if a neighbor claims MY
       * slot, the lower id keeps it and the other re-picks */
      if(unDock < m_bSlotTaken.size()) {
         m_bSlotTaken[unDock] = true;
         if((SInt8)unDock == m_nDockSlot && unSenderId < m_unRobotId) {
            m_nDockSlot = -1;
         }
      }
      /* Count only colleagues CLOSER to that belt than me: they are
       * ahead of me in the implicit queue, the ones behind me are not
       * my problem. This makes the swarm sort itself by distance. */
      if(unState == STATE_TO_BELT && unBelt < NUM_BELTS) {
         Real fTheirDist = unDist * 0.04;
         Real fMyDist = (m_cBelt[unBelt] - m_cPos).Length();
         if(fTheirDist < fMyDist) {
            ++m_unInbound[unBelt];
         }
      }
   }
}

/****************************************/
/****************************************/

void CFootBotWarehouse::Broadcast() {
   CByteArray cData;
   cData << (UInt8)m_eState;
   cData << m_unRobotId;
   if(m_eState == STATE_TO_BELT && m_nBeltChoice >= 0) {
      cData << (UInt8)m_nBeltChoice;
      Real fDist = (m_cBelt[m_nBeltChoice] - m_cPos).Length();
      cData << (UInt8)Min<Real>(fDist / 0.04, 255.0);
   }
   else {
      cData << NO_BYTE << NO_BYTE;
   }
   cData << (UInt8)(m_nCarryAddr >= 0 ? m_nCarryAddr : NO_BYTE);
   cData << (UInt8)(m_nDockSlot >= 0 ? m_nDockSlot : NO_BYTE);
   m_pcRABAct->SetData(cData);
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ChooseDockSlot() {
   /* Nearest slot nobody claims (per the last round of broadcasts) */
   SInt8 nBest = -1;
   Real fBestDist = 1.0e9;
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(m_bSlotTaken[s]) continue;
      Real fDist = (m_cDockSlots[s] - m_cPos).Length();
      if(fDist < fBestDist) {
         fBestDist = fDist;
         nBest = s;
      }
   }
   m_nDockSlot = nBest;
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ChooseBelt() {
   SInt8 nBest = -1;
   Real fBestU = -1.0e9;
   Real fCurrentU = -1.0e9;
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(m_unQueues[b] == 0) continue;   /* nothing to fetch there */
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      Real fU = W_QUEUE * m_unQueues[b]
              - W_INBOUND * m_unInbound[b]
              - W_DISTANCE * fDist
              /* small noise breaks the symmetry when identical robots
               * would otherwise all make the same choice on the same tick */
              + m_pcRNG->Uniform(CRange<Real>(0.0, 0.3));
      if((SInt8)b == m_nBeltChoice) fCurrentU = fU;
      if(fU > fBestU) {
         fBestU = fU;
         nBest = b;
      }
   }
   /* Work already covered by closer colleagues? Wait at the dock
    * instead of crowding the bins. Parked robots additionally demand a
    * clearly worthwhile job before giving up their slot. */
   bool bParked = m_nDockSlot >= 0 &&
                  (m_cDockSlots[m_nDockSlot] - m_cPos).Length() < 0.3;
   Real fCommitBar = bParked ? LEAVE_UTILITY : 0.0;
   if(nBest >= 0 && fBestU < fCommitBar) {
      nBest = -1;
   }
   /* Hysteresis: stick with the current belt unless clearly beaten */
   if(m_nBeltChoice >= 0 && m_unQueues[m_nBeltChoice] > 0 &&
      fCurrentU >= 0.0 && fCurrentU + HYSTERESIS >= fBestU) {
      return;
   }
   m_nBeltChoice = nBest;
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
      UInt8 unTheirPrio = PriorityOf(m_tNeighbors[i].State);
      Real fWeight;
      if(unMyPrio > unTheirPrio) {
         fWeight = 0.15;
      }
      else if(unMyPrio < unTheirPrio) {
         fWeight = 1.85;
      }
      else {
         fWeight = (m_unRobotId < m_tNeighbors[i].Id) ? 1.85 : 0.15;
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

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotWarehouse, "footbot_warehouse_controller")
