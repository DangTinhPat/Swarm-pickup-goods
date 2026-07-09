/**
 * footbot_grid.cpp — LÕI ĐIỀU KHIỂN: vòng đời ControlStep, máy trạng
 * thái nhiệm vụ, chính sách pin trễ (hysteresis 20% / 70%) và cơ chế
 * robot TỰ nhận việc trên bảng đen chia sẻ.
 *
 * Hai mô-đun còn lại của controller:
 *   - footbot_grid_nav.cpp     : định vị QR sàn + A* + bám tâm ô
 *   - footbot_grid_traffic.cpp : đặt chỗ ô + RAB + nhường đường ưu tiên
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
   m_cHardTurn(ToRadians(CDegrees(65.0))) {}

/****************************************/
/****************************************/

void CFootBotGrid::Init(TConfigurationNode& t_node) {
   /* Thiết bị */
   m_pcWheels     = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcWheelsSens = GetSensor  <CCI_DifferentialSteeringSensor>  ("differential_steering");
   m_pcRABAct     = GetActuator<CCI_RangeAndBearingActuator>     ("range_and_bearing");
   m_pcRABSens    = GetSensor  <CCI_RangeAndBearingSensor>       ("range_and_bearing");
   m_pcPosSens    = GetSensor  <CCI_PositioningSensor>           ("positioning");
   m_pcLEDs       = GetActuator<CCI_LEDsActuator>                ("leds");
   m_pcBattery    = GetSensor  <CCI_BatterySensor>               ("battery");
   m_pcGround     = GetSensor  <CCI_FootBotMotorGroundSensor>    ("footbot_motor_ground");

   /* Tham số XML (đều có mặc định hợp lý) */
   Real fHardTurnDeg = 65.0;
   TConfigurationNode* ptWheel = nullptr;
   if(NodeExists(t_node, "wheel")) {
      ptWheel = &GetNode(t_node, "wheel");
      GetNodeAttributeOrDefault(*ptWheel, "max_speed_cms",  m_fMaxSpeed,    m_fMaxSpeed);
      GetNodeAttributeOrDefault(*ptWheel, "turn_speed_cms", m_fTurnSpeed,   m_fTurnSpeed);
      GetNodeAttributeOrDefault(*ptWheel, "kp_heading",     m_fKpHeading,   m_fKpHeading);
      GetNodeAttributeOrDefault(*ptWheel, "hard_turn_deg",  fHardTurnDeg,   fHardTurnDeg);
      GetNodeAttributeOrDefault(*ptWheel, "waypoint_tol",   m_fWaypointTol, m_fWaypointTol);
      GetNodeAttributeOrDefault(*ptWheel, "goal_tol",       m_fGoalTol,     m_fGoalTol);
   }
   m_cHardTurn = ToRadians(CDegrees(fHardTurnDeg));
   if(NodeExists(t_node, "battery")) {
      TConfigurationNode& tBatt = GetNode(t_node, "battery");
      GetNodeAttributeOrDefault(tBatt, "low_threshold",   m_fLowBatt,   m_fLowBatt);
      GetNodeAttributeOrDefault(tBatt, "leave_threshold", m_fLeaveBatt, m_fLeaveBatt);
   }
   if(NodeExists(t_node, "timing")) {
      TConfigurationNode& tTim = GetNode(t_node, "timing");
      GetNodeAttributeOrDefault(tTim, "pick_ticks", m_unPickTicks, m_unPickTicks);
      GetNodeAttributeOrDefault(tTim, "drop_ticks", m_unDropTicks, m_unDropTicks);
   }

   /* Định danh số: "fb7" -> 7 (dùng làm chủ sở hữu đặt chỗ + phá hòa) */
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
   m_eState        = STATE_CHARGING;
   m_sTask.Clear();
   m_unActionTimer = 0;
   m_nTargetDock   = -1;
   m_bHaveGoal     = false;
   m_bGoalReached  = false;
   m_vecPath.clear();
   m_unPathIdx     = 0;
   m_vecNeighbors.clear();
   m_eYield        = YIELD_NONE;
   m_unYieldTimer  = 0;
   m_unBlockedTicks = 0;
   m_unSideCooldown = 0;
   m_pcRABAct->ClearData();
   m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

CGridLoopFunctions& CFootBotGrid::LF() {
   /* Lấy trễ: Loop Functions được ARGoS khởi tạo SAU các controller,
    * nên con trỏ chỉ an toàn từ tick đầu tiên trở đi. */
   if(m_pcLF == nullptr) {
      m_pcLF = &dynamic_cast<CGridLoopFunctions&>(
         CSimulator::GetInstance().GetLoopFunctions());
   }
   return *m_pcLF;
}

/****************************************/
/****************************************/

void CFootBotGrid::ControlStep() {
   /* 1. Định vị: tích phân encoder + chốt lại bằng "QR sàn" */
   UpdateLocalization();

   /* 2. Đọc pin (phân số 0..1 do mô hình time_motion của ARGoS xả,
    *    Loop Functions nạp lại khi robot đứng trong ô dock) */
   m_fBattery = m_pcBattery->GetReading().AvailableCharge;

   /* 3. Nghe hàng xóm quanh bán kính RAB */
   ParseNeighbors();

   /* 4. Tick đầu: robot sinh ra ngay trên dock của mình */
   if(m_bFirstStep) {
      m_bFirstStep = false;
      m_sPrevCell  = m_sCurCell;
      /* Dock bên dưới chân chính là dock mục tiêu ban đầu */
      for(SInt32 i = 0; i < NUM_DOCKS; ++i)
         if(DockCell(i) == m_sCurCell) { m_nTargetDock = i; break; }
      m_eState = (m_fBattery < m_fLeaveBatt) ? STATE_CHARGING : STATE_IDLE_DOCK;
   }

   /* 5. Quyết định nhiệm vụ (FSM) rồi 6. thi hành di chuyển an toàn */
   RunStateMachine();
   StepMovement();

   /* 7. Phát bản tin ưu tiên/ý định cho hàng xóm */
   BroadcastState();
   UpdateLed();
}

/****************************************/
/****************************************/

void CFootBotGrid::CheckBatteryEmergency() {
   if(m_fBattery >= m_fLowBatt) return;
   if(m_eState == STATE_CHARGING || m_eState == STATE_TO_CHARGE) return;

   /* Đang đậu sẵn trên dock thì chỉ cần chuyển sang chế độ sạc */
   if(m_eState == STATE_IDLE_DOCK) {
      m_eState = STATE_CHARGING;
      return;
   }

   /* NGƯỠNG XẢ CỨNG 20%: hủy/bàn giao nhiệm vụ ngay lập tức.
    * - Chưa bốc hộp  -> trả cả hộp lẫn yêu cầu về bảng đen cho robot khác.
    * - Đã ôm hộp     -> giữ quyền trên ô ngăn xếp (bàn giao dở dang),
    *                    sạc xong sẽ đi giao nốt chính hộp đang ôm.       */
   if(m_sTask.IsValid()) {
      LF().AbandonTask(m_unId, m_sTask.HasBox);
      if(!m_sTask.HasBox) m_sTask.Clear();
   }
   RetargetDock(true);
   m_eState = STATE_TO_CHARGE;
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

      case STATE_CHARGING: {
         /* NGƯỠNG SẠC CỨNG 70%: bám dock cho tới khi vượt ngưỡng.
          * (Loop Functions nạp pin cho MỌI robot đứng trong ô dock —
          * dock ẩn danh, không cần bắt tay, ai đến trước dùng trước.) */
         if(m_fBattery >= m_fLeaveBatt) {
            if(m_sTask.HasBox) {
               /* Tiếp tục chuyến giao hàng đã gác lại lúc khẩn cấp */
               m_eState = STATE_TO_DELIVER;
               SetGoal(LF().GetDemands()[m_sTask.DemandIdx].Cell);
            }
            else {
               m_eState = STATE_IDLE_DOCK;
            }
         }
         break;
      }

      case STATE_IDLE_DOCK: {
         /* Đậu tại dock (vẫn được sạc tiếp lên 100%). So le nhịp quét
          * bảng đen theo Id để 10 robot không tranh việc cùng một tick. */
         if((unTick + m_unId) % 10 == 0
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
            m_eState       = STATE_TO_DELIVER;
            SetGoal(LF().GetDemands()[m_sTask.DemandIdx].Cell);
         }
         else {
            /* Hộp biến mất (không thể xảy ra khi claim đúng) — nhả việc */
            LF().AbandonTask(m_unId, false);
            m_sTask.Clear();
            AfterTaskDone();
         }
         break;
      }

      case STATE_TO_DELIVER: {
         if(m_bGoalReached) {
            m_eState        = STATE_DELIVERING;
            m_unActionTimer = m_unDropTicks;
         }
         break;
      }

      case STATE_DELIVERING: {
         if(m_unActionTimer > 0) { --m_unActionTimer; break; }
         if(!LF().DeliverBox(m_unId, m_sTask.DemandIdx, m_sTask.Color)) {
            LOGERR << "[fb" << (int)m_unId << "] giao hang that bai?!" << std::endl;
         }
         m_sTask.Clear();
         AfterTaskDone();
         break;
      }

      case STATE_TO_REST: {
         /* Nghỉ khi khan việc — nhưng vẫn nghe ngóng: có hàng mới và
          * còn đủ pin thì quay xe nhận việc luôn, khỏi về tới dock. */
         if((unTick + m_unId) % 15 == 0
            && m_fBattery > m_fLowBatt + 0.05
            && TryClaimBestTask()) {
            m_eState = STATE_TO_PICKUP;
            break;
         }
         /* Dock nhắm tới vừa bị robot khác chiếm? -> chọn dock trống khác */
         if((unTick + m_unId) % 20 == 0
            && (m_nTargetDock < 0 || !LF().DockFree(m_nTargetDock, m_unId))) {
            RetargetDock(false);
         }
         if(m_bGoalReached && m_nTargetDock >= 0
            && m_sCurCell == DockCell(m_nTargetDock)) {
            m_eState = (m_fBattery < m_fLeaveBatt) ? STATE_CHARGING
                                                   : STATE_IDLE_DOCK;
         }
         break;
      }

      case STATE_TO_CHARGE: {
         /* Ưu tiên 1 — kiểm tra định kỳ dock còn trống không */
         if((unTick + m_unId) % 20 == 0
            && (m_nTargetDock < 0 || !LF().DockFree(m_nTargetDock, m_unId))) {
            RetargetDock(true);
         }
         if(m_bGoalReached && m_nTargetDock >= 0
            && m_sCurCell == DockCell(m_nTargetDock)) {
            m_eState = STATE_CHARGING;
         }
         break;
      }
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::TryClaimBestTask() {
   /* PHI TẬP TRUNG: robot tự duyệt bảng đen (chỉ-đọc), tự chấm điểm
    * từng cặp (hộp chờ, ô đang cần đúng màu đó) theo tổng quãng đường
    * Manhattan robot->băng chuyền->ngăn xếp, rồi xin ghi claim nguyên
    * tử. Bảng đen không phân việc cho ai — ai chấm xong trước thì
    * giành được trước (first-come-first-served). */
   const auto& vecConv = LF().GetConveyors();
   const auto& vecDem  = LF().GetDemands();

   SInt32 nBestConv = -1, nBestDem = -1, nBestCost = 0;
   for(size_t c = 0; c < vecConv.size(); ++c) {
      if(!vecConv[c].HasBox || vecConv[c].ClaimedBy >= 0) continue;
      for(size_t d = 0; d < vecDem.size(); ++d) {
         if(!vecDem[d].Active || vecDem[d].ClaimedBy >= 0) continue;
         if(vecDem[d].Color != vecConv[c].Color) continue;
         SInt32 nCost = m_sCurCell.ManhattanTo(vecConv[c].Cell)
                      + vecConv[c].Cell.ManhattanTo(vecDem[d].Cell);
         if(nBestConv < 0 || nCost < nBestCost) {
            nBestConv = c; nBestDem = d; nBestCost = nCost;
         }
      }
   }
   if(nBestConv < 0) return false;
   if(!LF().TryClaimTask(m_unId, nBestConv, nBestDem)) return false;

   m_sTask.ConveyorIdx = nBestConv;
   m_sTask.DemandIdx   = nBestDem;
   m_sTask.Color       = vecConv[nBestConv].Color;
   m_sTask.HasBox      = false;
   SetGoal(vecConv[nBestConv].Cell);
   return true;
}

/****************************************/
/****************************************/

void CFootBotGrid::AfterTaskDone() {
   /* Còn việc và còn pin -> làm tiếp; hết việc -> về dock trống gần
    * nhất nghỉ (KHÔNG chạy lòng vòng đốt pin). */
   if(m_fBattery > m_fLowBatt + 0.05 && TryClaimBestTask()) {
      m_eState = STATE_TO_PICKUP;
      return;
   }
   m_eState = STATE_TO_REST;
   if(!RetargetDock(false)) {
      /* Không còn dock trống (hiếm): đứng chờ tại chỗ, 20 tick sau
       * RunStateMachine sẽ thử chọn lại. */
      SetGoal(m_sCurCell);
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::RetargetDock(bool b_must_find) {
   /* DOCK ẨN DANH PHI TẬP TRUNG Ở HAI BIÊN BẢN ĐỒ:
    * hỏi bảng đen "trong 10 ô dock (5 trái + 5 phải), ô nào chưa có ai
    * đặt chỗ và chưa có robot đứng?", rồi tự chọn ô GẦN NHẤT theo
    * Manhattan — không hề có sự phân phối tập trung; hai robot cùng
    * nhắm một dock sẽ được phân xử bằng đặt chỗ ô khi tới gần, kẻ thua
    * tự động chọn dock trống khác. */
   SInt32 nDock = LF().NearestFreeDock(m_sCurCell, m_unId);
   if(nDock < 0) {
      if(b_must_find && m_nTargetDock >= 0) return true; /* giữ mục tiêu cũ */
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
   m_vecPath.clear();          /* buộc StepMovement vẽ lại đường */
   m_unPathIdx    = 0;
}

/****************************************/
/****************************************/

UInt8 CFootBotGrid::GetPriority() const {
   /* Bảng ưu tiên 3 mức của đề bài (số nhỏ = quyền cao):
    *   1 - khẩn cấp dưới 20% pin đang tìm trạm sạc
    *   2 - đang chở hộp hàng đi giao
    *   3 - chạy không tải / về dock nghỉ                            */
   if(m_eState == STATE_TO_CHARGE) return PRIO_EMERGENCY;
   if(m_sTask.HasBox)              return PRIO_CARRYING;
   return PRIO_IDLE;
}

/****************************************/
/****************************************/

const char* CFootBotGrid::GetStateName() const {
   switch(m_eState) {
      case STATE_CHARGING:   return "SAC";
      case STATE_IDLE_DOCK:  return "CHO";
      case STATE_TO_PICKUP:  return "DI-LAY";
      case STATE_PICKING:    return "BOC";
      case STATE_TO_DELIVER: return "DI-GIAO";
      case STATE_DELIVERING: return "HA";
      case STATE_TO_REST:    return "VE-NGHI";
      case STATE_TO_CHARGE:  return "KHAN-CAP";
   }
   return "?";
}

/****************************************/
/****************************************/

void CFootBotGrid::UpdateLed() {
   if(m_eState == STATE_TO_CHARGE)      m_pcLEDs->SetAllColors(CColor::MAGENTA);
   else if(m_eState == STATE_CHARGING)  m_pcLEDs->SetAllColors(CColor::ORANGE);
   else if(m_sTask.HasBox)              m_pcLEDs->SetAllColors(BoxCColor(m_sTask.Color));
   else                                 m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotGrid, "footbot_grid_controller")

}  /* namespace argos */
