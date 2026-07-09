/**
 * footbot_grid_traffic.cpp — ĐẶT CHỖ PHI TẬP TRUNG + DẠT LÀN CỤC BỘ
 * (Windowed/local traffic — KHÔNG BAO GIỜ gọi A* toàn cục để né một
 * robot đối diện; A* toàn cục chỉ là lưới an toàn cuối cùng khi kẹt
 * bệnh lý rất lâu, xem PlanPath escalation ở cuối StepMovement()).
 *
 * Nguyên tắc "đặt trước - đi sau": trước khi bước sang ô kế tiếp, robot
 * phải ghi được đặt chỗ ô đó lên bảng chiếm dụng chia sẻ
 * (m_sGridReservations[Tick][(Row,Col)] = RobotId, xem grid_loop_functions).
 *
 * QUY TẮC ƯU TIÊN BẤT ĐỐI XỨNG: robot thắng quyền giữ nguyên lộ trình
 * và V_Base=10cm/s, không bẻ lái/giảm tốc vì xung đột (IWinAgainst chỉ
 * làm thay đổi hành vi của robot THUA). Robot thua thực hiện THUẬT
 * TOÁN DẠT LÀN CỤC BỘ ĐÚNG 3 BƯỚC:
 *   [1] rẽ ngang sang ô trống bên cạnh (vuông góc hướng đi hiện tại)
 *   [2] tịnh tiến song song 1 ô, vượt qua robot đối diện
 *   [3] rẽ ngược trở lại làn cũ, tiếp tục lộ trình gốc
 * Chỉ khi CẢ HAI ô bên trái/phải đều nghẽn mới đứng im 1-2 tick
 * (STATE_YIELDING) chờ không gian trống rồi thử lại.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::KeepAliveReservation() {
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

   /* --- Đã đứng trong ô đích: chỉ còn bám nốt hồng tâm rồi báo xong --- */
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

   /* --- Chạm tâm ô kế tiếp: hoàn tất một bước lưới --- */
   if(sEstCell == sNext && (m_cEstPos - cNextCen).Length() < m_fWaypointTol) {
      LF().ReleaseCell(m_unId, m_sCurCell, RES_GRACE);
      m_sPrevCell      = m_sCurCell;
      m_sCurCell       = sNext;
      ++m_unPathIdx;
      m_unBlockedTicks = 0;
      ApplySteering(cNextCen, m_unPathIdx >= m_vecPath.size());
      return;
   }

   /* ------------------------------------------------------------------
    * QUÉT XUNG ĐỘT CỤC BỘ QUA RAB trước khi bước vào sNext:
    *   ĐỐI ĐẦU  : hàng xóm đứng ở sNext và muốn bước vào ô của TÔI
    *   GIAO CẮT : hàng xóm cũng định bước vào đúng sNext
    * ------------------------------------------------------------------ */
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
      /* Thua quyền: nếu lỡ giữ đặt chỗ ô tranh chấp thì nhả ngay */
      if(sEstCell == m_sCurCell) LF().ReleaseCell(m_unId, sNext, 0);
      if(TryStartDetour(sNext)) return;
      /* Cả 2 ô bên đều nghẽn -> đứng im 1-2 tick rồi thử lại (spec) */
      m_eTraffic     = TRAFFIC_YIELDING;
      m_unYieldTimer = 2;
      StopWheels();
      return;
   }
   else if(bOccupiedAhead) {
      /* Mình thắng quyền nhưng ô phía trước còn người -> dừng chờ,
       * KHÔNG bẻ lái / giảm tốc theo lộ trình gốc (đúng V_Base=10cm/s
       * khi được đi tiếp — chỉ đơn thuần chờ ô vật lý trống ra). */
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

   /* Lưới an toàn cuối cùng cho kẹt bệnh lý (KHÔNG phải đường đi
    * thường trực — dạt làn cục bộ ở trên đã xử lý > 99% xung đột):
    * 3s -> né ô đã bị đặt chỗ quanh mình, 8s -> né luôn hàng xóm RAB. */
   if(m_unBlockedTicks == 30)      PlanPath(m_sGoalCell, 1);
   else if(m_unBlockedTicks == 80) PlanPath(m_sGoalCell, 2);
   else if(m_unBlockedTicks > 160) m_unBlockedTicks = 0;
}

/****************************************/
/****************************************/

bool CFootBotGrid::IWinAgainst(const SNeighbor& s_n) const {
   /* Ưu tiên bất đối xứng; hòa cấp -> Id nhỏ thắng (luật phá hòa tất
    * định để cả 2 bên luôn kết luận GIỐNG NHAU không cần bắt tay). */
   UInt8 unMine = GetPriority();
   if(unMine != s_n.Prio) return unMine < s_n.Prio;
   return m_unId < s_n.Id;
}

/****************************************/
/****************************************/

bool CFootBotGrid::TryStartDetour(const SGridCell& s_blocked) {
   /* THUẬT TOÁN DẠT LÀN CỤC BỘ 3 BƯỚC — không gọi A* toàn cục.
    * Hướng đi hiện tại (dRow,dCol) là bước đơn vị curCell -> s_blocked.
    * Hai ứng viên vuông góc (trái/phải) trong hành lang liền kề:      */
   const SInt32 dRow = s_blocked.Row - m_sCurCell.Row;
   const SInt32 dCol = s_blocked.Col - m_sCurCell.Col;
   const std::pair<SInt32, SInt32> aPerp[2] = {
      {  dCol, -dRow },   /* vuông góc bên "trái"  */
      { -dCol,  dRow }    /* vuông góc bên "phải"  */
   };

   const UInt32 unNow = LF().Tick();

   for(const auto& sPerp : aPerp) {
      SGridCell sStep1(m_sCurCell.Row + sPerp.first,  m_sCurCell.Col + sPerp.second);
      SGridCell sStep2(sStep1.Row + dRow,              sStep1.Col + dCol);
      SGridCell sStep3(m_sCurCell.Row + 2 * dRow,      m_sCurCell.Col + 2 * dCol);

      auto Free = [&](const SGridCell& c) {
         if(!c.IsValid() || CellTypeOf(c) != CELL_FREE) return false;
         SInt32 nOwner = LF().CellReserver(c);
         return nOwner < 0 || nOwner == (SInt32)m_unId;
      };
      if(!Free(sStep1) || !Free(sStep2) || !Free(sStep3)) continue;

      /* Đặt chỗ TRƯỚC cả 3 bước (không chỉ bước 1): việc dạt làn đưa
       * robot đi song song sát robot ưu tiên cao hơn ở khoảng cách
       * đúng 1 ô (0.2 m tâm-tâm, chỉ dư 0.03 m so với đường kính thân
       * 0.17 m) — giữ trọn cửa sổ thời gian cho cả 3 bước giảm rủi ro
       * một robot thứ 3 chen vào đúng lúc hai bên đang sát nhau. */
      const UInt32 unHold = unNow + RES_AHEAD + 2 * RES_KEEPALIVE;
      if(!LF().TryReserveCell(m_unId, sStep1, unNow + RES_AHEAD)) continue;
      if(!LF().TryReserveCell(m_unId, sStep2, unHold)) {
         LF().ReleaseCell(m_unId, sStep1, 0);
         continue;
      }
      if(!LF().TryReserveCell(m_unId, sStep3, unHold)) {
         LF().ReleaseCell(m_unId, sStep1, 0);
         LF().ReleaseCell(m_unId, sStep2, 0);
         continue;
      }

      m_vecDetourPath = { sStep1, sStep2, sStep3 };
      m_unDetourIdx   = 0;
      m_eTraffic      = TRAFFIC_DETOURING;
      ++m_unDetourCount;
      return true;
   }
   return false;
}

/****************************************/
/****************************************/

void CFootBotGrid::HandleDetourPhase() {
   if(m_unDetourIdx >= m_vecDetourPath.size()) {
      /* Hoàn tất 3 bước: đã ở làn cũ, 1 ô sau ô từng bị chặn -> buộc
       * vẽ lại lộ trình gốc từ vị trí mới (đơn giản, an toàn). */
      m_eTraffic = TRAFFIC_NONE;
      m_vecPath.clear();
      m_unBlockedTicks = 0;
      return;
   }

   const SGridCell& sTarget = m_vecDetourPath[m_unDetourIdx];
   CVector2 cCen(RowToWorldX(sTarget.Row), ColToWorldY(sTarget.Col));
   SGridCell sEstCell(WorldXToRow(m_cEstPos.GetX()), WorldYToCol(m_cEstPos.GetY()));

   if(sEstCell == sTarget && (m_cEstPos - cCen).Length() < m_fWaypointTol) {
      LF().ReleaseCell(m_unId, m_sCurCell, RES_GRACE);
      m_sPrevCell = m_sCurCell;
      m_sCurCell  = sTarget;
      ++m_unDetourIdx;
      /* Đặt chỗ trước bước kế tiếp của detour (nếu còn) trước khi đi */
      if(m_unDetourIdx < m_vecDetourPath.size()) {
         const SGridCell& sNextStep = m_vecDetourPath[m_unDetourIdx];
         if(!LF().TryReserveCell(m_unId, sNextStep, LF().Tick() + RES_AHEAD)) {
            StopWheels();   /* bước kế bị chiếm tức thời -> chờ 1 tick */
            return;
         }
      }
      ApplySteering(cCen, m_unDetourIdx >= m_vecDetourPath.size());
      return;
   }
   ApplySteering(cCen, false);
}

/****************************************/
/****************************************/

void CFootBotGrid::HandleYieldPhase() {
   StopWheels();
   if(m_unYieldTimer > 0) --m_unYieldTimer;
   if(m_unYieldTimer == 0) {
      m_eTraffic = TRAFFIC_NONE;   /* StepMovement tick sau sẽ thử lại */
   }
}

/****************************************/
/****************************************/

SGridCell CFootBotGrid::PlannedNextCell() const {
   if(m_eTraffic == TRAFFIC_DETOURING && m_unDetourIdx < m_vecDetourPath.size())
      return m_vecDetourPath[m_unDetourIdx];
   if(m_eTraffic == TRAFFIC_YIELDING) return SGridCell();
   if(m_bHaveGoal && !m_bGoalReached && m_unPathIdx < m_vecPath.size())
      return m_vecPath[m_unPathIdx];
   return SGridCell();
}

/****************************************/
/****************************************/

void CFootBotGrid::ParseNeighbors() {
   m_vecNeighbors.clear();
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   for(size_t i = 0; i < tMsgs.size(); ++i) {
      const CCI_RangeAndBearingSensor::SPacket& sPkt = tMsgs[i];
      if(sPkt.Data.Size() < RAB_MSG_SIZE) continue;
      SNeighbor sN;
      sN.Id = sPkt.Data[RAB_IDX_ID];
      if(sN.Id == m_unId || sN.Id >= NUM_DOCKS) continue;
      sN.Prio  = sPkt.Data[RAB_IDX_PRIO];
      sN.State = sPkt.Data[RAB_IDX_STATE];
      sN.Flags = sPkt.Data[RAB_IDX_FLAGS];
      sN.Batt  = sPkt.Data[RAB_IDX_BATT];
      sN.Cur   = SGridCell(sPkt.Data[RAB_IDX_CUR_R], sPkt.Data[RAB_IDX_CUR_C]);
      if(sPkt.Data[RAB_IDX_NEXT_R] != RAB_NO_CELL)
         sN.Next = SGridCell(sPkt.Data[RAB_IDX_NEXT_R], sPkt.Data[RAB_IDX_NEXT_C]);
      sN.Range = sPkt.Range * 0.01;
      m_vecNeighbors.push_back(sN);
   }
}

/****************************************/
/****************************************/

void CFootBotGrid::BroadcastState() {
   SGridCell sNext = PlannedNextCell();
   m_pcRABAct->SetData(RAB_IDX_ID,    m_unId);
   m_pcRABAct->SetData(RAB_IDX_PRIO,  GetPriority());
   m_pcRABAct->SetData(RAB_IDX_STATE, static_cast<UInt8>(m_eState));
   m_pcRABAct->SetData(RAB_IDX_CUR_R, static_cast<UInt8>(m_sCurCell.Row));
   m_pcRABAct->SetData(RAB_IDX_CUR_C, static_cast<UInt8>(m_sCurCell.Col));
   m_pcRABAct->SetData(RAB_IDX_NEXT_R,
      sNext.IsValid() ? static_cast<UInt8>(sNext.Row) : RAB_NO_CELL);
   m_pcRABAct->SetData(RAB_IDX_NEXT_C,
      sNext.IsValid() ? static_cast<UInt8>(sNext.Col) : RAB_NO_CELL);
   UInt8 unFlags = 0;
   if(m_sTask.HasBox)              unFlags |= RAB_FLAG_CARRY;
   if(m_eTraffic != TRAFFIC_NONE)  unFlags |= RAB_FLAG_YIELD;
   m_pcRABAct->SetData(RAB_IDX_FLAGS, unFlags);
   m_pcRABAct->SetData(RAB_IDX_BATT,
      static_cast<UInt8>(Max<Real>(0.0, Min<Real>(1.0, m_fBattery)) * 100.0));
}

}  /* namespace argos */
