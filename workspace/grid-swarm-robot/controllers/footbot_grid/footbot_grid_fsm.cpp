/**
 * footbot_grid_fsm.cpp — Máy trạng thái nhiệm vụ, chính sách pin trễ
 * (hysteresis 20%/70%) và cơ chế tự nhận việc trên bảng đen.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::CheckBatteryEmergency() {
   if(m_fBattery >= m_fLowBatt) return;
   if(m_eState == STATE_EMERGENCY_CHARGE) return;

   /* Đã đậu ở dock: chỉ đổi nhãn, dock đang nạp sẵn rồi */
   if(m_eState == STATE_IDLE || m_eState == STATE_RESTING) {
      m_eState = STATE_EMERGENCY_CHARGE;
      RetargetDock(true);
      return;
   }

   /* Chưa bốc hộp: trả cả hộp lẫn yêu cầu về bảng đen.
    * Đã ôm hộp: giữ quyền trên ô kệ, sạc xong đi giao nốt. */
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

void CFootBotGrid::CheckOperatorRecall() {
   /* Triệu hồi của operator, ngữ nghĩa "giao nốt rồi mới về":
    * - đang ôm hộp -> để yên cho giao xong (không bao giờ vứt hàng);
    *   các cổng nhận việc bên dưới chặn mọi chuyến MỚI, nên giao xong
    *   AfterTaskDone sẽ tự rơi vào RETURNING -> về dock -> bị ghim.
    * - đã nhận việc nhưng chưa bốc -> trả việc về bảng đen, về dock.
    * - đang đậu (IDLE/RESTING) -> đứng yên, cổng nhận việc ghim tại chỗ.
    * - đang sạc khẩn cấp -> giữ nguyên (điều kiện thả đã bị khoá). */
   if(m_eOverride != OP_RECALL) return;
   if(m_sTask.HasBox) return;
   if(m_eState == STATE_IDLE || m_eState == STATE_RESTING
      || m_eState == STATE_EMERGENCY_CHARGE
      || m_eState == STATE_RETURNING) return;

   if(m_sTask.IsValid()) {
      LF().AbandonTask(m_unId, false);
      m_sTask.Clear();
   }
   m_eState = STATE_RETURNING;
   if(!RetargetDock(false)) {
      SetGoal(m_sCurCell);   /* hết dock trống: chờ tại chỗ, RETURNING tự thử lại */
   }
}

/****************************************/
/****************************************/

void CFootBotGrid::RunStateMachine() {
   CheckBatteryEmergency();
   CheckOperatorRecall();
   const UInt32 unTick = LF().Tick();

   switch(m_eState) {

      case STATE_IDLE: {
         if(m_eOverride == OP_AUTO
            && (unTick + m_unId) % 10 == 0
            && m_fBattery >= m_fLeaveBatt
            && TryClaimBestTask()) {
            m_eState = STATE_TO_PICKUP;
            m_unIdleTicks = 0;
            break;
         }
         if(++m_unIdleTicks > m_unIdleRestTimeout) {
            m_eState = STATE_RESTING;
            m_unIdleTicks = 0;
         }
         break;
      }

      case STATE_RESTING: {
         /* Ngủ đông: quét bảng việc thưa hơn IDLE, vẫn sạc thụ động */
         if(m_eOverride == OP_AUTO
            && (unTick + m_unId) % 20 == 0
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
         /* Trên đường về vẫn nghe ngóng: có việc + đủ pin thì quay xe */
         if(m_eOverride == OP_AUTO
            && (unTick + m_unId) % 15 == 0
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
         /* Chỉ giải phóng khi ĐANG Ở dock VÀ pin đã hồi >= 70%; robot bị
          * operator triệu hồi thì bị ghim ở đây tới khi được thả */
         if(m_bGoalReached && m_nTargetDock >= 0
            && m_sCurCell == DockCell(m_nTargetDock)
            && m_fBattery >= m_fLeaveBatt
            && m_eOverride != OP_RECALL) {
            /* Đang ôm hộp (bỏ dở vì cạn pin): đi giao NỐT — trước đây
             * nhánh này rơi về IDLE, hộp trên lưng bị bỏ quên và có thể
             * bị TryClaimBestTask ghi đè (mất hộp + ô kệ bị claim treo
             * vĩnh viễn). Transition vào DELIVERING duy nhất nằm ở
             * PICKING nên phải nối lại ở đây. */
            if(m_sTask.HasBox) {
               m_eState = STATE_DELIVERING;
               const CGridLoopFunctions::SDemand& sDem =
                  LF().GetDemands()[m_sTask.DemandIdx];
               SetGoal(StackFaceCell(sDem.Cell, m_sTask.FarSide));
            }
            else {
               m_eState = STATE_IDLE;
            }
         }
         break;
      }
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::TryClaimBestTask() {
   /* Greedy phi tập trung: tự chấm điểm mọi cặp (hộp chưa ai nhận, ô kệ
    * cần đúng màu) theo tổng Manhattan robot->băng->mặt kệ, rồi xin ghi
    * claim nguyên tử — bảng đen không phân việc cho ai. */
   const auto& vecConv = LF().GetConveyors();
   const auto& vecDem  = LF().GetDemands();

   SInt32 nBestConv = -1, nBestDem = -1, nBestCost = 0;
   bool   bBestFar   = false;
   UInt8  unBestColor = 0;
   for(size_t c = 0; c < vecConv.size(); ++c) {
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
   /* Biên an toàn +5% trên ngưỡng khẩn cấp trước khi nhận chuyến mới */
   if(m_fBattery > m_fLowBatt + 0.05 && TryClaimBestTask()) {
      m_eState = STATE_TO_PICKUP;
      return;
   }
   m_eState = STATE_RETURNING;
   if(!RetargetDock(false)) {
      SetGoal(m_sCurCell);   /* không còn dock trống: chờ tại chỗ, thử lại sau */
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::RetargetDock(bool b_must_find) {
   SInt32 nDock = LF().NearestFreeDock(m_sCurCell, m_unId);
   if(nDock < 0) {
      if(b_must_find && m_nTargetDock >= 0) return true;   /* giữ mục tiêu cũ */
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
   if(m_eState == STATE_EMERGENCY_CHARGE) return PRIO_EMERGENCY;
   if(m_eState == STATE_DELIVERING)       return PRIO_DELIVERING;
   return PRIO_IDLE;
}

}  /* namespace argos */
