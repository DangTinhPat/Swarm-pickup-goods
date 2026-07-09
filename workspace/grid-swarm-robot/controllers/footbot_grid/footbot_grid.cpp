/**
 * footbot_grid.cpp — LÕI ĐIỀU KHIỂN: vòng đời ControlStep, máy trạng
 * thái nhiệm vụ đặt tên đúng đặc tả (IDLE / TO_PICKUP / PICKING /
 * DELIVERING / DROPPING / RETURNING / RESTING / EMERGENCY_CHARGE),
 * chính sách pin trễ (hysteresis 20% / 70%) và cơ chế robot TỰ nhận
 * việc trên bảng đen chia sẻ (không có phân việc tập trung).
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
      GetNodeAttributeOrDefault(tTim, "pick_ticks",    m_unPickTicks,       m_unPickTicks);
      GetNodeAttributeOrDefault(tTim, "drop_ticks",    m_unDropTicks,       m_unDropTicks);
      GetNodeAttributeOrDefault(tTim, "idle_rest_ticks", m_unIdleRestTimeout, m_unIdleRestTimeout);
   }

   const std::string& strId = GetId();
   size_t unDigit = strId.find_first_of("0123456789");
   m_unId = (unDigit == std::string::npos)
               ? 0 : static_cast<UInt8>(std::stoul(strId.substr(unDigit)));

   Reset();
}

/****************************************/
/****************************************/

void CFootBotGrid::Reset() {
   m_bPoseInit     = false;
   m_bFirstStep    = true;
   m_fOdoSinceFix  = 0.0;
   m_sLastSnapCell = SGridCell();
   m_unSnapCount   = 0;
   m_unRelocCount  = 0;
   m_fBattery      = 1.0;
   m_eState        = STATE_RESTING;
   m_sTask.Clear();
   m_unActionTimer = 0;
   m_unIdleTicks   = 0;
   m_nTargetDock   = -1;
   m_bHaveGoal     = false;
   m_bGoalReached  = false;
   m_vecPath.clear();
   m_unPathIdx     = 0;
   m_vecNeighbors.clear();
   m_eTraffic      = TRAFFIC_NONE;
   m_vecDetourPath.clear();
   m_unDetourIdx   = 0;
   m_unYieldTimer  = 0;
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

void CFootBotGrid::CheckBatteryEmergency() {
   if(m_fBattery >= m_fLowBatt) return;
   if(m_eState == STATE_EMERGENCY_CHARGE) return;

   /* Đã đậu tại dock (IDLE/RESTING): không cần di chuyển, chỉ đổi
    * nhãn trạng thái — dock đã đang nạp pin cho robot rồi. */
   if(m_eState == STATE_IDLE || m_eState == STATE_RESTING) {
      m_eState = STATE_EMERGENCY_CHARGE;
      RetargetDock(true);
      return;
   }

   /* NGƯỠNG XẢ CỨNG 20%: hủy/bàn giao nhiệm vụ ngay lập tức.
    * - Chưa bốc hộp -> trả cả hộp lẫn yêu cầu về bảng đen.
    * - Đã ôm hộp    -> giữ quyền trên ô mặt kệ, sạc xong đi giao nốt. */
   if(m_sTask.IsValid()) {
      LF().AbandonTask(m_unId, m_sTask.HasBox);
      if(!m_sTask.HasBox) m_sTask.Clear();
   }
   RetargetDock(true);
   m_eState = STATE_EMERGENCY_CHARGE;
   LF().NotifyEmergency(m_unId);
   LOG << "[fb" << (int)m_unId << "] KHAN CAP pin "
       << (int)(m_fBattery * 100.0) << "% -> dock " << m_nTargetDock << std::endl;
}

/****************************************/
/****************************************/

void CFootBotGrid::RunStateMachine() {
   CheckBatteryEmergency();
   const UInt32 unTick = LF().Tick();

   switch(m_eState) {

      case STATE_IDLE: {
         if((unTick + m_unId) % 10 == 0
            && m_fBattery >= m_fLeaveBatt
            && TryClaimBestTask()) {
            m_eState = STATE_TO_PICKUP;
            m_unIdleTicks = 0;
            break;
         }
         /* Quá 100 tick không có việc -> ngủ đông tiết kiệm năng lượng */
         if(++m_unIdleTicks > m_unIdleRestTimeout) {
            m_eState = STATE_RESTING;
            m_unIdleTicks = 0;
         }
         break;
      }

      case STATE_RESTING: {
         /* Vẫn ở dock, vẫn được sạc thụ động; dò việc thưa hơn IDLE
          * (nhịp quét dài hơn) đúng tinh thần "ngủ đông tiết kiệm". */
         if((unTick + m_unId) % 20 == 0
            && m_fBattery >= m_fLeaveBatt
            && TryClaimBestTask()) {
            m_eState = STATE_TO_PICKUP;
         }
         break;
      }

      case STATE_TO_PICKUP: {
         if(m_bGoalReached) {
            m_eState        = STATE_PICKING;
            m_unActionTimer = m_unPickTicks;
         }
         break;
      }

      case STATE_PICKING: {
         if(m_unActionTimer > 0) { --m_unActionTimer; break; }
         UInt8 unColor = 0;
         if(LF().PickUpBox(m_unId, m_sTask.ConveyorIdx, unColor)) {
            m_sTask.HasBox = true;
            m_sTask.Color  = unColor;
            m_eState       = STATE_DELIVERING;
            const CGridLoopFunctions::SDemand& sDem = LF().GetDemands()[m_sTask.DemandIdx];
            SetGoal(StackFaceCell(sDem.Cell, m_sTask.FarSide));
         }
         else {
            LF().AbandonTask(m_unId, false);
            m_sTask.Clear();
            AfterTaskDone();
         }
         break;
      }

      case STATE_DELIVERING: {
         if(m_bGoalReached) {
            m_eState        = STATE_DROPPING;
            m_unActionTimer = m_unDropTicks;
         }
         break;
      }

      case STATE_DROPPING: {
         if(m_unActionTimer > 0) { --m_unActionTimer; break; }
         if(!LF().DeliverBox(m_unId, m_sTask.DemandIdx, m_sTask.Color)) {
            LOGERR << "[fb" << (int)m_unId << "] giao hang that bai?!" << std::endl;
         }
         m_sTask.Clear();
         AfterTaskDone();
         break;
      }

      case STATE_RETURNING: {
         if((unTick + m_unId) % 15 == 0
            && m_fBattery > m_fLowBatt + 0.05
            && TryClaimBestTask()) {
            m_eState = STATE_TO_PICKUP;
            break;
         }
         if((unTick + m_unId) % 20 == 0
            && (m_nTargetDock < 0 || !LF().DockFree(m_nTargetDock, m_unId))) {
            RetargetDock(false);
         }
         if(m_bGoalReached && m_nTargetDock >= 0
            && m_sCurCell == DockCell(m_nTargetDock)) {
            m_eState = (m_fBattery >= m_fLeaveBatt) ? STATE_IDLE : STATE_RESTING;
         }
         break;
      }

      case STATE_EMERGENCY_CHARGE: {
         if((unTick + m_unId) % 20 == 0
            && (m_nTargetDock < 0 || !LF().DockFree(m_nTargetDock, m_unId))) {
            RetargetDock(true);
         }
         /* Chỉ giải phóng về IDLE khi ĐANG Ở dock VÀ pin đã phục hồi >=70% */
         if(m_bGoalReached && m_nTargetDock >= 0
            && m_sCurCell == DockCell(m_nTargetDock)
            && m_fBattery >= m_fLeaveBatt) {
            m_eState = STATE_IDLE;
         }
         break;
      }
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::TryClaimBestTask() {
   const auto& vecConv = LF().GetConveyors();
   const auto& vecDem  = LF().GetDemands();

   SInt32 nBestConv = -1, nBestDem = -1, nBestCost = 0;
   bool   bBestFar   = false;
   UInt8  unBestColor = 0;
   for(size_t c = 0; c < vecConv.size(); ++c) {
      /* Băng chuyền trưng bày tối đa 3 hộp — chỉ cần MỘT hộp chưa ai
       * nhận trong hàng đợi khớp màu là đủ điều kiện xét (không quan
       * tâm thứ tự trong Queue). */
      bool bHasAnyFreeBox = false;
      for(SInt32 nOwner : vecConv[c].ClaimedBy)
         if(nOwner < 0) { bHasAnyFreeBox = true; break; }
      if(!bHasAnyFreeBox) continue;

      for(size_t d = 0; d < vecDem.size(); ++d) {
         if(!vecDem[d].Active || vecDem[d].ClaimedBy >= 0) continue;
         bool bColorAvailable = false;
         for(size_t k = 0; k < vecConv[c].Queue.size(); ++k)
            if(vecConv[c].ClaimedBy[k] < 0 && vecConv[c].Queue[k] == vecDem[d].Color)
               { bColorAvailable = true; break; }
         if(!bColorAvailable) continue;
         /* Ô ngăn xếp là vật cản: chi phí thật là tới MẶT KỆ gần hơn
          * trong 2 mặt (trước/sau dải obstacle) — không phải tới ô đó. */
         SGridCell sFaceNear = StackFaceCell(vecDem[d].Cell, false);
         SGridCell sFaceFar  = StackFaceCell(vecDem[d].Cell, true);
         SInt32 nCostNear = vecConv[c].Cell.ManhattanTo(sFaceNear);
         SInt32 nCostFar  = vecConv[c].Cell.ManhattanTo(sFaceFar);
         bool   bFar      = nCostFar < nCostNear;
         SInt32 nLegCost  = bFar ? nCostFar : nCostNear;
         SInt32 nCost = m_sCurCell.ManhattanTo(vecConv[c].Cell) + nLegCost;
         if(nBestConv < 0 || nCost < nBestCost) {
            nBestConv = c; nBestDem = d; nBestCost = nCost; bBestFar = bFar;
            unBestColor = vecDem[d].Color;
         }
      }
   }
   if(nBestConv < 0) return false;
   if(!LF().TryClaimTask(m_unId, nBestConv, nBestDem)) return false;

   m_sTask.ConveyorIdx = nBestConv;
   m_sTask.DemandIdx   = nBestDem;
   m_sTask.Color       = unBestColor;
   m_sTask.HasBox      = false;
   m_sTask.FarSide     = bBestFar;
   SetGoal(vecConv[nBestConv].Cell);
   return true;
}

/****************************************/
/****************************************/

void CFootBotGrid::AfterTaskDone() {
   if(m_fBattery > m_fLowBatt + 0.05 && TryClaimBestTask()) {
      m_eState = STATE_TO_PICKUP;
      return;
   }
   m_eState = STATE_RETURNING;
   if(!RetargetDock(false)) {
      SetGoal(m_sCurCell);
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::RetargetDock(bool b_must_find) {
   SInt32 nDock = LF().NearestFreeDock(m_sCurCell, m_unId);
   if(nDock < 0) {
      if(b_must_find && m_nTargetDock >= 0) return true;
      return false;
   }
   m_nTargetDock = nDock;
   SetGoal(DockCell(nDock));
   return true;
}

/****************************************/
/****************************************/

void CFootBotGrid::SetGoal(const SGridCell& s_cell) {
   m_sGoalCell    = s_cell;
   m_bHaveGoal    = true;
   m_bGoalReached = (m_sCurCell == s_cell);
   m_vecPath.clear();
   m_unPathIdx    = 0;
}

/****************************************/
/****************************************/

UInt8 CFootBotGrid::GetPriority() const {
   /* Ưu tiên bất đối xứng, hàm thuần của FSM (không nhấp nháy giữa đường):
    *   1 - STATE_EMERGENCY_CHARGE (pin<=20% đang/đã tìm trạm sạc)
    *   2 - STATE_DELIVERING (đang chở hộp hàng màu đi giao)
    *   3 - còn lại (IDLE/RETURNING/RESTING/TO_PICKUP/PICKING/DROPPING) */
   if(m_eState == STATE_EMERGENCY_CHARGE) return PRIO_EMERGENCY;
   if(m_eState == STATE_DELIVERING)       return PRIO_DELIVERING;
   return PRIO_IDLE;
}

/****************************************/
/****************************************/

const char* CFootBotGrid::GetStateName() const {
   switch(m_eState) {
      case STATE_IDLE:              return "IDLE";
      case STATE_TO_PICKUP:         return "TO_PICKUP";
      case STATE_PICKING:           return "PICKING";
      case STATE_DELIVERING:        return "DELIVERING";
      case STATE_DROPPING:          return "DROPPING";
      case STATE_RETURNING:         return "RETURNING";
      case STATE_RESTING:           return "RESTING";
      case STATE_EMERGENCY_CHARGE:  return "EMERGENCY_CHARGE";
   }
   return "?";
}

/****************************************/
/****************************************/

void CFootBotGrid::UpdateLed() {
   if(m_eState == STATE_EMERGENCY_CHARGE)     m_pcLEDs->SetAllColors(CColor::MAGENTA);
   else if(m_eState == STATE_IDLE || m_eState == STATE_RESTING)
                                               m_pcLEDs->SetAllColors(CColor::ORANGE);
   else if(m_sTask.HasBox)                    m_pcLEDs->SetAllColors(BoxCColor(m_sTask.Color));
   else                                       m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotGrid, "footbot_grid_controller")

}  /* namespace argos */
