/*
 * Core lifecycle (Init/Reset) + the ControlStep state-machine orchestrator.
 * Behavior-specific logic lives in the sibling footbot_*.cpp files listed
 * in footbot_warehouse.h; this file wires them together per tick.
 */
#include "footbot_warehouse.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <functional>

/****************************************/
/****************************************/

/* Parked robots align to this heading (facing the belts) for neat rows */
static const CRadians DOCK_HEADING = CRadians::ZERO;
/* Inside this distance of the bay the robot drops the potential field and
 * creeps straight onto the pad (see the final-approach branch below). */
static const Real PRECISE_DOCK_DIST = 0.35;
/* After a scouting trip an idle robot rests parked this many ticks before it
 * may patrol again (jittered by id so the fleet doesn't scout in lockstep). */
static const UInt32 PATROL_COOLDOWN = 250;

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
   m_pcBattery(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL),
   m_fObstacleGain(100.0),
   m_fHardAvoidThreshold(0.05),
   m_fResumeCharge(0.9),
   m_fMinWorkCharge(0.3),
   m_fHardChargeThreshold(0.10),
   m_fCharge(1.0),
   m_fBeltSenseRange(1.2),
   m_eState(STATE_IDLE),
   m_nCarryAddr(-1),
   m_nBeltChoice(-1),
   m_unDecisionLockUntil(0),
   m_unPatrolHoldUntil(0),
   m_nAvoidSign(0),
   m_nDockSlot(-1),
   m_fStuckRefGoalDist(-1.0),
   m_unStuckRefTick(0),
   m_unStuckStrikes(0),
   m_unEscapeAttempts(0),
   m_nEscapeTicksLeft(0),
   m_nRescueReverseTicksLeft(0),
   m_nRescueDriveTicksLeft(0),
   m_bRescuing(false),
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
   m_pcBattery   = GetSensor<CCI_BatterySensor>("battery");
   m_pcGround    = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");
   m_pcRNG       = CRandom::CreateRNG("argos");

   m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   m_sSeparationParams.Init(GetNode(t_node, "separation"));

   TConfigurationNode& tNav = GetNode(t_node, "navigation");
   GetNodeAttributeOrDefault(tNav, "obstacle_gain", m_fObstacleGain, m_fObstacleGain);
   GetNodeAttributeOrDefault(tNav, "hard_avoid_threshold", m_fHardAvoidThreshold, m_fHardAvoidThreshold);
   GetNodeAttributeOrDefault(tNav, "belt_sense_range", m_fBeltSenseRange, m_fBeltSenseRange);

   /* Energy policy (flat thresholds) */
   TConfigurationNode& tEnergy = GetNode(t_node, "energy");
   GetNodeAttributeOrDefault(tEnergy, "resume_charge", m_fResumeCharge, m_fResumeCharge);
   GetNodeAttributeOrDefault(tEnergy, "min_work_charge", m_fMinWorkCharge, m_fMinWorkCharge);
   GetNodeAttributeOrDefault(tEnergy, "hard_charge_threshold", m_fHardChargeThreshold, m_fHardChargeThreshold);

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
   /* Structural pillars (facility map) */
   GetNodeAttributeOrDefault(tSt, "pillar_radius", m_fPillarRadius, 0.15);
   GetNodeAttributeOrDefault(tSt, "pillar_range", m_fPillarRange, 0.8);
   for(UInt32 p = 0; ; ++p) {
      std::string strAttr = "pillar" + std::to_string(p);
      if(!NodeAttributeExists(tSt, strAttr)) break;
      CVector2 cPillar;
      GetNodeAttribute(tSt, strAttr, cPillar);
      m_cPillars.push_back(cPillar);
   }

   /* Docking/charging bays: rows along x, cols along y, with separate
    * spacings — cols=2 with a large y spacing yields the two bay rows
    * on the left and right sides of the floor */
   UInt32 unRows, unCols;
   Real fSpacingX, fSpacingY;
   CVector2 cDockCenter;
   GetNodeAttribute(tSt, "dock_center", cDockCenter);
   GetNodeAttribute(tSt, "dock_rows", unRows);
   GetNodeAttribute(tSt, "dock_cols", unCols);
   GetNodeAttribute(tSt, "dock_spacing_x", fSpacingX);
   GetNodeAttribute(tSt, "dock_spacing_y", fSpacingY);
   for(UInt32 r = 0; r < unRows; ++r) {
      for(UInt32 c = 0; c < unCols; ++c) {
         m_cDockSlots.push_back(cDockCenter + CVector2(
            (Real)((SInt32)r - ((SInt32)unRows - 1) / 2.0) * fSpacingX,
            (Real)((SInt32)c - ((SInt32)unCols - 1) / 2.0) * fSpacingY));
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
   m_unDecisionLockUntil = 0;
   m_unPatrolHoldUntil = 0;
   m_nDockSlot = -1;
   m_unTickCount = 0;
   for(UInt32 i = 0; i < NUM_BELTS; ++i) {
      m_unInbound[i] = 0;
      m_unTrueQueue[i] = 0;
      m_bTrueBlocked[i] = false;
      /* Nothing known yet: OriginTick=0 -> BeltAge() reads as huge, so a
       * freshly (re)started robot never bids on a phantom belt */
      m_tBeltBelief[i].Queue = 0;
      m_tBeltBelief[i].Blocked = false;
      m_tBeltBelief[i].OriginTick = 0;
   }
   m_pcLEDs->SetAllColors(CColor::GREEN);
   m_pcRABAct->ClearData();
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ControlStep() {
   UpdatePose();
   ++m_unTickCount;
   m_fCharge = m_pcBattery->GetReading().AvailableCharge;
   SenseBelts();

   /* Battery empty: brick in place. Neighbors still avoid the corpse via
    * separation + proximity, and it must release any charging bay it held. */
   if(m_fCharge <= 0.005) {
      m_eState = STATE_DEAD;
      m_nDockSlot = -1;
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      m_pcLEDs->SetAllColors(CColor::BLACK);
      Broadcast();
      return;
   }

   for(UInt32 i = 0; i < NUM_BELTS; ++i) m_unInbound[i] = 0;
   ProcessMessages();

   /* Energy policy: a flat threshold (the battery is large relative to the
    * facility, so no per-trip distance prediction is needed). A carrying
    * robot is never diverted to charge. */
   if(m_eState == STATE_CHARGE) {
      if(m_fCharge >= m_fResumeCharge) m_eState = STATE_IDLE;
   }
   else if(m_eState != STATE_DELIVER && m_fCharge < m_fHardChargeThreshold) {
      m_eState = STATE_CHARGE;
      m_nBeltChoice = -1;
   }

   /* Bid for work. The re-bid cadence is staggered by robot id so decisions
    * trickle out one robot at a time rather than the whole dock reacting to
    * the same parcel on the same tick. ChooseBelt itself is anti-dither
    * (sticky commitment); this only paces how often it runs. */
   if(m_eState == STATE_IDLE || m_eState == STATE_TO_BELT) {
      bool bEvaluate = (m_nBeltChoice < 0)
                       ? ((m_unTickCount + m_unRobotId) % 3 == 0)
                       : ((m_unTickCount + m_unRobotId) % 10 == 0);
      if(bEvaluate) ChooseBelt();
      m_eState = (m_nBeltChoice >= 0) ? STATE_TO_BELT : STATE_IDLE;
      if(m_eState == STATE_TO_BELT) m_nDockSlot = -1;
   }

   /* Stuck/rescue check — runs BEFORE the proximity reflex and for every
    * active state. A robot wedged in a corner keeps the reflex firing
    * (proximity never clears), which would return early forever and starve
    * the escape logic; checking first guarantees it always gets a turn. */
   {
      CVector2 cTarget;
      bool bExempt = false;
      switch(m_eState) {
         case STATE_DELIVER:
            cTarget = m_cAddr[m_nCarryAddr];
            break;
         case STATE_TO_BELT:
            cTarget = m_cBelt[m_nBeltChoice];
            bExempt = (cTarget - m_cPos).Length() < 0.45;  /* queueing != stuck */
            break;
         case STATE_IDLE: {
            CVector2 cPatrol;
            if(PatrolReady() && NeedsBeltPatrol(cPatrol)) {
               cTarget = cPatrol;
               bExempt = (cTarget - m_cPos).Length() < m_fBeltSenseRange;
               m_nDockSlot = -1;
            }
            else {
               cTarget = ComputeDockTarget(bExempt);
            }
            break;
         }
         case STATE_CHARGE:
         default:
            cTarget = ComputeDockTarget(bExempt);
            break;
      }
      if(!bExempt && UpdateStuckEscape(cTarget)) {
         Broadcast();
         return;
      }
   }

   /* Emergency collision reflex: two-stage override (moving turn, then
    * pivot in place above 3x threshold). Suppressed only once TRULY parked
    * so a docked robot ignores a neighbor parking next door; while still
    * creeping in it stays on and simply turns (never a bespoke wait, which
    * once let two robots converging on adjacent bays freeze each other). */
   bool bTrulyParked =
      (m_eState == STATE_CHARGE || m_eState == STATE_IDLE) &&
      m_nDockSlot >= 0 &&
      (m_cDockSlots[m_nDockSlot] - m_cPos).Length() < DOCKED_DIST;
   if(!bTrulyParked) {
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
         /* Commit to one turn direction: a dead-ahead obstacle flips the
          * accumulator's sign every tick, so following it jitters forever. */
         if(m_nAvoidSign == 0) {
            m_nAvoidSign = (cProxAccum.Angle() > CRadians::ZERO) ? -1 : 1;
         }
         if(m_nAvoidSign < 0) m_pcWheels->SetLinearVelocity(fOuter, fInner);
         else                 m_pcWheels->SetLinearVelocity(fInner, fOuter);
         Broadcast();
         return;
      }
      m_nAvoidSign = 0;
   }

   /* Potential-field motion */
   CVector2 cGoal;
   switch(m_eState) {
      case STATE_DELIVER:
         cGoal = VectorToPoint(m_cAddr[m_nCarryAddr]);
         m_pcLEDs->SetAllColors(CColor::RED);
         break;
      case STATE_TO_BELT:
         cGoal = VectorToPoint(m_cBelt[m_nBeltChoice]);
         if((m_cBelt[m_nBeltChoice] - m_cPos).Length() < 0.6) cGoal *= 0.5;
         m_pcLEDs->SetAllColors(CColor::CYAN);
         break;
      case STATE_IDLE: {
         CVector2 cPatrol;
         if(PatrolReady() && NeedsBeltPatrol(cPatrol)) {
            Real fD = (cPatrol - m_cPos).Length();
            cGoal = VectorToPoint(cPatrol);
            if(fD < 0.6) cGoal *= 0.5;
            /* Reached the belt: info refreshed — now rest before scouting
             * again so a surplus robot settles rather than shuffling. */
            if(fD < m_fBeltSenseRange) {
               m_unPatrolHoldUntil = m_unTickCount + PATROL_COOLDOWN + (m_unRobotId % 60);
            }
            m_pcLEDs->SetAllColors(CColor::YELLOW);
            break;
         }
         [[fallthrough]];   /* nothing stale (or resting): idle at the dock */
      }
      case STATE_CHARGE:
      default: {
         /* Dock bays double as chargers: an idle robot (green) tops up
          * opportunistically, a STATE_CHARGE robot (blue) is required to. */
         m_pcLEDs->SetAllColors(m_eState == STATE_CHARGE ? CColor::BLUE
                                                         : CColor::GREEN);
         if(m_nDockSlot < 0) {
            /* No free slot heard of yet: hold near the nearest bay row. */
            Real fBest = 1.0e9;
            CVector2 cNearest;
            for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
               Real fD = (m_cDockSlots[s] - m_cPos).Length();
               if(fD < fBest) { fBest = fD; cNearest = m_cDockSlots[s]; }
            }
            cGoal = VectorToPoint(cNearest);
            cGoal *= 0.3;
            break;
         }
         Real fDist = (m_cDockSlots[m_nDockSlot] - m_cPos).Length();
         if(fDist < DOCKED_DIST) {
            /* Parked: align to the common heading then freeze (following the
             * field here would let passing robots drag the whole row). */
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
         /* Final approach: creep straight onto the pad with the field OFF so
          * a parked neighbor's separation can't hold us outside the radius. */
         if(fDist < PRECISE_DOCK_DIST) {
            cGoal = VectorToPoint(m_cDockSlots[m_nDockSlot]);
            cGoal *= Max<Real>(0.25, Min<Real>(1.0, fDist / 0.4));
            SetWheelSpeedsFromVector(cGoal);
            Broadcast();
            return;
         }
         cGoal = VectorToPoint(m_cDockSlots[m_nDockSlot]);
         cGoal *= Min<Real>(1.0, fDist / 0.4);   /* arrival damping */
         break;
      }
   }
   SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector() + PillarField());

   Broadcast();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotWarehouse, "footbot_warehouse_controller")
