/**
 * footbot_grid_traffic.cpp — Vòng di chuyển an toàn: nguyên tắc
 * "đặt-trước-đi-sau" trên ma trận chiếm dụng theo tick, quét xung đột
 * qua RAB và phân xử bằng ưu tiên bất đối xứng. Robot thắng giữ nguyên
 * làn + tốc độ; robot thua chuyển sang footbot_grid_detour.cpp.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::KeepAliveReservation() {
   /* Ô đang đứng luôn thuộc về mình: gia hạn mỗi tick */
   LF().TryReserveCell(m_unId, m_sCurCell, LF().Tick() + RES_KEEPALIVE);
}

/****************************************/
/****************************************/

void CFootBotGrid::StepMovement() {
   KeepAliveReservation();

   if(!m_bHaveGoal || m_bGoalReached) {
      StopWheels();
      return;
   }

   if(m_eTraffic == TRAFFIC_DETOURING) { HandleDetourPhase(); return; }
   if(m_eTraffic == TRAFFIC_YIELDING)  { HandleYieldPhase();  return; }

   const UInt32 unNow = LF().Tick();

   /* Đã ở ô đích: bám nốt hồng tâm rồi báo xong */
   if(m_sCurCell == m_sGoalCell) {
      CVector2 cCen(RowToWorldX(m_sGoalCell.Row), ColToWorldY(m_sGoalCell.Col));
      if((m_cEstPos - cCen).Length() < m_fGoalTol) {
         m_bGoalReached = true;
         StopWheels();
      }
      else {
         ApplySteering(cCen, true);
      }
      return;
   }

   if(m_vecPath.empty() || !(m_sPathGoal == m_sGoalCell)
      || m_unPathIdx >= m_vecPath.size()) {
      if(!PlanPath(m_sGoalCell, 0)) {
         StopWheels();
         ++m_unBlockedTicks;
         return;
      }
   }

   SGridCell sNext = m_vecPath[m_unPathIdx];
   CVector2  cNextCen(RowToWorldX(sNext.Row), ColToWorldY(sNext.Col));
   SGridCell sEstCell(WorldXToRow(m_cEstPos.GetX()),
                      WorldYToCol(m_cEstPos.GetY()));

   /* Chạm tâm ô kế tiếp: hoàn tất một bước lưới */
   if(sEstCell == sNext && (m_cEstPos - cNextCen).Length() < m_fWaypointTol) {
      LF().ReleaseCell(m_unId, m_sCurCell, RES_GRACE);
      m_sPrevCell      = m_sCurCell;
      m_sCurCell       = sNext;
      ++m_unPathIdx;
      m_unBlockedTicks = 0;
      ApplySteering(cNextCen, m_unPathIdx >= m_vecPath.size());
      return;
   }

   /* Quét xung đột: ĐỐI ĐẦU (hàng xóm ở sNext muốn vào ô của tôi),
    * GIAO CẮT (hàng xóm cũng nhắm sNext), CHIẾM CHỖ (thân còn ở sNext) */
   bool bOccupiedAhead = false;
   bool bLosing        = false;
   for(const SNeighbor& sN : m_vecNeighbors) {
      if(sN.Cur == sNext) {
         bOccupiedAhead = true;
         if(sN.Next.IsValid() && sN.Next == m_sCurCell && !IWinAgainst(sN))
            bLosing = true;
      }
      else if(sN.Next.IsValid() && sN.Next == sNext && !IWinAgainst(sN)) {
         bLosing = true;
      }
   }

   if(bLosing) {
      /* Thua quyền: nhả đặt chỗ ô tranh chấp nếu chưa lấn sang */
      if(sEstCell == m_sCurCell) LF().ReleaseCell(m_unId, sNext, 0);
      if(TryStartDetour(sNext)) return;
      m_eTraffic     = TRAFFIC_YIELDING;
      m_unYieldTimer = 2;
      StopWheels();
      return;
   }
   else if(bOccupiedAhead) {
      /* Thắng quyền nhưng ô trước còn người: đứng chờ, không đổi làn */
      StopWheels();
      ++m_unBlockedTicks;
   }
   else {
      if(LF().TryReserveCell(m_unId, sNext, unNow + RES_AHEAD)) {
         ApplySteering(cNextCen, false);
         return;
      }
      StopWheels();
      ++m_unBlockedTicks;
   }

   /* Lưới an toàn cho kẹt bệnh lý (dạt làn đã xử lý phần lớn xung đột):
    * 3s -> A* né ô bị đặt chỗ, 8s -> né luôn hàng xóm RAB, 16s -> reset */
   if(m_unBlockedTicks == 30)      PlanPath(m_sGoalCell, 1);
   else if(m_unBlockedTicks == 80) PlanPath(m_sGoalCell, 2);
   else if(m_unBlockedTicks > 160) m_unBlockedTicks = 0;
}

/****************************************/
/****************************************/

bool CFootBotGrid::IWinAgainst(const SNeighbor& s_n) const {
   /* Hòa cấp -> Id nhỏ thắng: luật tất định, hai bên tự kết luận
    * GIỐNG NHAU không cần bắt tay */
   UInt8 unMine = GetPriority();
   if(unMine != s_n.Prio) return unMine < s_n.Prio;
   return m_unId < s_n.Id;
}

}  /* namespace argos */
