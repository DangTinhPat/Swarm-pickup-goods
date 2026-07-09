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
/* Final docking maneuver: inside this distance of the target bay the
 * robot ignores the potential field and creeps straight onto the pad —
 * otherwise separation pushes from parked neighbors can hold it in an
 * equilibrium just outside the charging radius, "parked" but not
 * charging. */
static const Real PRECISE_DOCK_DIST = 0.35;

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

   /* 0a. Battery empty: the robot bricks where it stands. Neighbors
    * still avoid it via separation + proximity (it became an obstacle). */
   if(m_fCharge <= 0.005) {
      m_eState = STATE_DEAD;
      m_nDockSlot = -1;   /* a corpse must not keep a charging bay claimed */
      m_pcWheels->SetLinearVelocity(0.0, 0.0);
      m_pcLEDs->SetAllColors(CColor::BLACK);
      Broadcast();
      return;
   }

   /* 1. Listen: count colleagues inbound to each belt */
   for(UInt32 i = 0; i < NUM_BELTS; ++i) m_unInbound[i] = 0;
   ProcessMessages();

   /* 0b. Energy policy: a flat threshold, not a distance prediction.
    * With the fleet's battery sized generously relative to the facility
    * (a full lap of the 8x8 m floor costs a small fraction of a charge),
    * there is no need to estimate the trip cost — going to charge only
    * once truly low is simpler and, at this capacity, still safe.
    * Deliveries are never abandoned: charge is only checked for
    * robots that are not already carrying a parcel. */
   if(m_eState == STATE_CHARGE) {
      if(m_fCharge >= m_fResumeCharge) {
         m_eState = STATE_IDLE;   /* stay parked; bidding resumes below */
      }
   }
   else if(m_eState != STATE_DELIVER && m_fCharge < m_fHardChargeThreshold) {
      m_eState = STATE_CHARGE;
      m_nBeltChoice = -1;
   }

   /* 2. Bid for work (only when free and sufficiently charged):
    * re-evaluate when uncommitted, and periodically so stale
    * commitments adapt to new information */
   if(m_eState == STATE_IDLE || m_eState == STATE_TO_BELT) {
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

   /* 2.5. Unified stuck/rescue check — evaluated BEFORE the emergency
    * proximity reflex (step 3) and for EVERY active navigation state,
    * not just docking. Without this ordering, a robot wedged against a
    * wall/pillar corner keeps the reflex perpetually re-triggering
    * (proximity never clears), which returns early every single tick
    * and starves this check of any chance to ever run — exactly how a
    * robot ends up permanently stuck at a corner despite the mechanism
    * existing. Checking first guarantees it always gets sampled, and
    * once it decides to escalate, it owns the wheels for its own escape
    * duration, overriding the reflex for those ticks. */
   {
      CVector2 cTarget;
      bool bExempt = false;
      switch(m_eState) {
         case STATE_DELIVER:
            cTarget = m_cAddr[m_nCarryAddr];
            break;
         case STATE_TO_BELT:
            cTarget = m_cBelt[m_nBeltChoice];
            /* Normal queueing near a busy bin isn't "stuck" */
            bExempt = (cTarget - m_cPos).Length() < 0.45;
            break;
         case STATE_IDLE: {
            /* Stale info on some belt? Go look — a genuinely idle robot
             * that never patrols would leave the fleet permanently blind
             * to any belt no one has driven past recently (see
             * NeedsBeltPatrol). A robot in STATE_CHARGE never does this
             * — it heads straight for the dock regardless. */
            CVector2 cPatrol;
            if(NeedsBeltPatrol(cPatrol)) {
               cTarget = cPatrol;
               bExempt = (cTarget - m_cPos).Length() < m_fBeltSenseRange;
               /* Don't hog a parking bay while off patrolling */
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

   /* 3. Collision imminent? Two-stage override (same as ball-collector):
    * moving turn above threshold, pivot in place above 3x threshold.
    * SUPPRESSED only once TRULY parked (< DOCKED_DIST of my own bay —
    * matches the freeze branch in step 4): a stationary docked robot
    * must not react to a neighbor parking next door. While still
    * CREEPING IN (DOCKED_DIST..PRECISE_DOCK_DIST) the reflex stays ON,
    * using the same committed-turn logic as everywhere else in the
    * facility — this used to be a bespoke "stop and wait for the
    * blocker to move" rule with no escape: two robots converging on
    * adjacent bays at the same moment could freeze each other forever
    * and starve to death right next to an empty charger. The ordinary
    * reflex always resolves (it turns, never just waits), so reusing it
    * here removes the deadlock. */
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
         /* Commit to one turn direction for the whole encounter: a
          * dead-ahead obstacle flips the accumulator's angle sign every
          * tick, and turning with it would jitter in place forever */
         if(m_nAvoidSign == 0) {
            m_nAvoidSign = (cProxAccum.Angle() > CRadians::ZERO) ? -1 : 1;
         }
         if(m_nAvoidSign < 0) {
            m_pcWheels->SetLinearVelocity(fOuter, fInner);
         }
         else {
            m_pcWheels->SetLinearVelocity(fInner, fOuter);
         }
         Broadcast();
         return;
      }
      /* Obstacle cleared: forget the committed turn direction */
      m_nAvoidSign = 0;
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
      case STATE_IDLE: {
         CVector2 cPatrol;
         if(NeedsBeltPatrol(cPatrol)) {
            /* Off to sense a belt whose info has gone stale — plain
             * potential-field travel, same as heading to a belt with a
             * confirmed job, just without one yet */
            cGoal = VectorToPoint(cPatrol);
            if((cPatrol - m_cPos).Length() < 0.6) {
               cGoal *= 0.5;
            }
            m_pcLEDs->SetAllColors(CColor::YELLOW);
            break;
         }
         /* Nothing stale: normal dock idling, same as STATE_CHARGE below */
         [[fallthrough]];
      }
      case STATE_CHARGE:
      default: {
         /* Dock: claim a free parking slot and park in it, neatly.
          * Parking bays double as chargers, so an idle robot tops up
          * opportunistically; a STATE_CHARGE robot is just one that is
          * REQUIRED to be here (blue LED) until resume_charge. */
         m_pcLEDs->SetAllColors(m_eState == STATE_CHARGE ? CColor::BLUE
                                                         : CColor::GREEN);
         /* m_nDockSlot was already resolved in step 2.5 above (it needs
          * to exist before the stuck-check can pick a target) */
         if(m_nDockSlot < 0) {
            /* No free slot heard of (transient): hold near the nearest
             * bay row until claims sort themselves out */
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
         /* Final docking maneuver: creep straight onto the pad with the
          * potential field OFF — separation from parked neighbors must
          * not be able to hold us just outside the charging radius.
          * (Bays are 0.55 apart, physics cannot make bodies overlap.) */
         if(fDist < PRECISE_DOCK_DIST) {
            cGoal = VectorToPoint(m_cDockSlots[m_nDockSlot]);
            cGoal *= Max<Real>(0.25, Min<Real>(1.0, fDist / 0.4));
            SetWheelSpeedsFromVector(cGoal);
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
   SetWheelSpeedsFromVector(cGoal + SeparationVector() + ObstacleVector() + PillarField());

   /* 5. Tell the neighborhood what I am doing */
   Broadcast();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotWarehouse, "footbot_warehouse_controller")
