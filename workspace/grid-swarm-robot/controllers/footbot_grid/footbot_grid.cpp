/**
 * footbot_grid.cpp — Vòng đời controller: Init/Reset/ControlStep,
 * LED trạng thái và các tiện ích chung.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

/****************************************/
/****************************************/

CFootBotGrid::CFootBotGrid() :
   m_cHardTurn(ToRadians(CDegrees(55.0))) {}

/****************************************/
/****************************************/

void CFootBotGrid::Init(TConfigurationNode& t_node) {
   m_pcWheels     = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcWheelsSens = GetSensor  <CCI_DifferentialSteeringSensor>  ("differential_steering");
   m_pcRABAct     = GetActuator<CCI_RangeAndBearingActuator>     ("range_and_bearing");
   m_pcRABSens    = GetSensor  <CCI_RangeAndBearingSensor>       ("range_and_bearing");
   m_pcPosSens    = GetSensor  <CCI_PositioningSensor>           ("positioning");
   m_pcLEDs       = GetActuator<CCI_LEDsActuator>                ("leds");
   m_pcBattery    = GetSensor  <CCI_BatterySensor>               ("battery");
   m_pcGround     = GetSensor  <CCI_FootBotMotorGroundSensor>    ("footbot_motor_ground");

   Real fHardTurnDeg = 55.0;
   if(NodeExists(t_node, "wheel")) {
      TConfigurationNode& tWheel = GetNode(t_node, "wheel");
      GetNodeAttributeOrDefault(tWheel, "cruise_speed_cms", m_fCruiseSpeed, m_fCruiseSpeed);
      GetNodeAttributeOrDefault(tWheel, "pivot_speed_cms",  m_fPivotSpeed,  m_fPivotSpeed);
      GetNodeAttributeOrDefault(tWheel, "kp_heading",       m_fKpHeading,   m_fKpHeading);
      GetNodeAttributeOrDefault(tWheel, "hard_turn_deg",    fHardTurnDeg,   fHardTurnDeg);
      GetNodeAttributeOrDefault(tWheel, "waypoint_tol",     m_fWaypointTol, m_fWaypointTol);
      GetNodeAttributeOrDefault(tWheel, "goal_tol",         m_fGoalTol,     m_fGoalTol);
      GetNodeAttributeOrDefault(tWheel, "qr_snap_radius",   m_fQrSnapRadius, m_fQrSnapRadius);
   }
   m_cHardTurn = ToRadians(CDegrees(fHardTurnDeg));
   if(NodeExists(t_node, "battery")) {
      TConfigurationNode& tBatt = GetNode(t_node, "battery");
      GetNodeAttributeOrDefault(tBatt, "low_threshold",   m_fLowBatt,   m_fLowBatt);
      GetNodeAttributeOrDefault(tBatt, "leave_threshold", m_fLeaveBatt, m_fLeaveBatt);
   }
   if(NodeExists(t_node, "timing")) {
      TConfigurationNode& tTim = GetNode(t_node, "timing");
      GetNodeAttributeOrDefault(tTim, "pick_ticks",      m_unPickTicks,       m_unPickTicks);
      GetNodeAttributeOrDefault(tTim, "drop_ticks",      m_unDropTicks,       m_unDropTicks);
      GetNodeAttributeOrDefault(tTim, "idle_rest_ticks", m_unIdleRestTimeout, m_unIdleRestTimeout);
   }

   /* "fb7" -> 7: dùng làm chủ đặt chỗ và luật phá hòa ưu tiên */
   const std::string& strId = GetId();
   size_t unDigit = strId.find_first_of("0123456789");
   m_unId = (unDigit == std::string::npos)
               ? 0 : static_cast<UInt8>(std::stoul(strId.substr(unDigit)));

   Reset();
}

/****************************************/
/****************************************/

void CFootBotGrid::Reset() {
   m_bPoseInit      = false;
   m_bFirstStep     = true;
   m_fOdoSinceFix   = 0.0;
   m_sLastSnapCell  = SGridCell();
   m_unSnapCount    = 0;
   m_unRelocCount   = 0;
   m_fBattery       = 1.0;
   m_eState         = STATE_RESTING;
   m_sTask.Clear();
   m_unActionTimer  = 0;
   m_unIdleTicks    = 0;
   m_nTargetDock    = -1;
   m_bHaveGoal      = false;
   m_bGoalReached   = false;
   m_vecPath.clear();
   m_unPathIdx      = 0;
   m_vecNeighbors.clear();
   m_eTraffic       = TRAFFIC_NONE;
   m_vecDetourPath.clear();
   m_unDetourIdx    = 0;
   m_unYieldTimer   = 0;
   m_unBlockedTicks = 0;
   m_unDetourCount  = 0;
   m_pcRABAct->ClearData();
   m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

CGridLoopFunctions& CFootBotGrid::LF() {
   if(m_pcLF == nullptr) {
      m_pcLF = &dynamic_cast<CGridLoopFunctions&>(
         CSimulator::GetInstance().GetLoopFunctions());
   }
   return *m_pcLF;
}

/****************************************/
/****************************************/

void CFootBotGrid::ControlStep() {
   UpdateLocalization();
   m_fBattery = m_pcBattery->GetReading().AvailableCharge;
   ParseNeighbors();

   if(m_bFirstStep) {
      m_bFirstStep = false;
      m_sPrevCell  = m_sCurCell;
      for(SInt32 i = 0; i < NUM_DOCKS; ++i)
         if(DockCell(i) == m_sCurCell) { m_nTargetDock = i; break; }
      m_eState = (m_fBattery >= m_fLeaveBatt) ? STATE_IDLE : STATE_RESTING;
   }

   RunStateMachine();
   StepMovement();
   BroadcastState();
   UpdateLed();
}

/****************************************/
/****************************************/

const char* CFootBotGrid::GetStateName() const {
   switch(m_eState) {
      case STATE_IDLE:             return "IDLE";
      case STATE_TO_PICKUP:        return "TO_PICKUP";
      case STATE_PICKING:          return "PICKING";
      case STATE_DELIVERING:       return "DELIVERING";
      case STATE_DROPPING:         return "DROPPING";
      case STATE_RETURNING:        return "RETURNING";
      case STATE_RESTING:          return "RESTING";
      case STATE_EMERGENCY_CHARGE: return "EMERGENCY_CHARGE";
   }
   return "?";
}

/****************************************/
/****************************************/

void CFootBotGrid::UpdateLed() {
   if(m_eState == STATE_EMERGENCY_CHARGE)
      m_pcLEDs->SetAllColors(CColor::MAGENTA);
   else if(m_eState == STATE_IDLE || m_eState == STATE_RESTING)
      m_pcLEDs->SetAllColors(CColor::ORANGE);
   else if(m_sTask.HasBox)
      m_pcLEDs->SetAllColors(BoxCColor(m_sTask.Color));
   else
      m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotGrid, "footbot_grid_controller")

}  /* namespace argos */
