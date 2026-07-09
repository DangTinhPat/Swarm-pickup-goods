/**
 * footbot_grid_traffic.cpp — ĐẶT CHỖ PHI TẬP TRUNG + XỬ LÝ VA CHẠM
 * THEO QUYỀN ƯU TIÊN (swarm logic thuần cục bộ)
 *
 * Nguyên tắc "đặt trước - đi sau": robot KHÔNG BAO GIỜ cho bánh lăn về
 * một ô khi chưa ghi được đặt chỗ ô đó lên bảng chiếm dụng chia sẻ
 * (Grid[Row][Col] + cửa sổ tick hết hạn) do Loop Functions cầm hộ.
 *
 * Tranh chấp tại hành lang 2 ô được giải bằng trao đổi mức ưu tiên qua
 * RAB: (1) khẩn cấp sắp cạn pin, (2) đang chở hàng, (3) chạy không tải.
 * Robot yếu quyền tự nhường: đứng im tại ô của mình, hoặc chủ động DẠT
 * sang ô trống bên cạnh (làn thứ hai của hành lang) cho robot mạnh
 * quyền đi qua; hết xung đột thì tự vẽ lại đường đi tiếp.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::KeepAliveReservation() {
   /* Ô đang đứng luôn thuộc về mình: gia hạn mỗi tick để bảng chiếm
    * dụng phản ánh đúng hiện diện vật lý (kể cả khi đứng sạc rất lâu) */
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

   /* Đang trong một pha nhường đường? */
   if(m_eYield != YIELD_NONE) {
      HandleYieldPhase();
      return;
   }

   const UInt32 unNow = LF().Tick();

   /* --- Đã đứng trong ô đích: chỉ còn bám nốt hồng tâm rồi báo xong --- */
   if(m_sCurCell == m_sGoalCell) {
      CVector2 cCen(ColToWorldX(m_sGoalCell.Col), RowToWorldY(m_sGoalCell.Row));
      if((m_cEstPos - cCen).Length() < m_fGoalTol) {
         m_bGoalReached = true;
         StopWheels();
      }
      else {
         ApplySteering(cCen, true);
      }
      return;
   }

   /* --- Bảo đảm có lộ trình hợp lệ cho đích hiện tại --- */
   if(m_vecPath.empty() || !(m_sPathGoal == m_sGoalCell)
      || m_unPathIdx >= m_vecPath.size()) {
      if(!PlanPath(m_sGoalCell, 0)) {
         StopWheels();
         ++m_unBlockedTicks;
         return;
      }
   }

   SGridCell sNext = m_vecPath[m_unPathIdx];
   CVector2  cNextCen(ColToWorldX(sNext.Col), RowToWorldY(sNext.Row));
   SGridCell sEstCell(WorldYToRow(m_cEstPos.GetY()),
                      WorldXToCol(m_cEstPos.GetX()));

   /* --- Chạm tâm ô kế tiếp: hoàn tất một bước lưới --- */
   if(sEstCell == sNext && (m_cEstPos - cNextCen).Length() < m_fWaypointTol) {
      /* nhả ô cũ nhưng giữ "ân hạn" vài tick vì đuôi xe có thể còn
       * vắt qua vạch — robot khác chỉ đặt chỗ được sau ân hạn này */
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
    *   CHIẾM CHỖ: thân hàng xóm còn nằm trong sNext (đi cùng chiều...)
    * Kẻ thua cuộc (ưu tiên yếu, hoặc cùng ưu tiên nhưng Id lớn) phải
    * nhường; kẻ thắng chỉ việc đứng chờ ô trống rồi đi tiếp.
    * ------------------------------------------------------------------ */
   bool bOccupiedAhead = false;
   bool bLoseHeadOn    = false;
   bool bLoseCross     = false;
   const SNeighbor* psOpp = nullptr;
   for(const SNeighbor& sN : m_vecNeighbors) {
      if(sN.Cur == sNext) {
         bOccupiedAhead = true;
         if(sN.Next.IsValid() && sN.Next == m_sCurCell && !IWinAgainst(sN)) {
            bLoseHeadOn = true;
            psOpp = &sN;
         }
      }
      else if(sN.Next.IsValid() && sN.Next == sNext && !IWinAgainst(sN)) {
         bLoseCross = true;
         if(psOpp == nullptr) psOpp = &sN;
      }
   }

   if(bLoseHeadOn || bLoseCross) {
      /* Thua quyền: nếu lỡ giữ đặt chỗ ô tranh chấp (và thân mình chưa
       * lấn sang) thì NHẢ NGAY cho robot ưu tiên cao đi trước */
      if(sEstCell == m_sCurCell) {
         LF().ReleaseCell(m_unId, sNext, 0);
      }
      /* Đối đầu trực diện: cố gắng dạt sang làn bên cạnh của hành lang */
      if(bLoseHeadOn && m_unSideCooldown == 0) {
         if(TrySidestep(*psOpp, sNext)) return;
         m_unSideCooldown = 10;   /* né thất bại: đừng thử lại từng tick */
      }
      StopWheels();
      ++m_unBlockedTicks;
   }
   else if(bOccupiedAhead) {
      /* Mình thắng quyền nhưng ô phía trước còn người -> dừng chờ */
      StopWheels();
      ++m_unBlockedTicks;
   }
   else {
      /* --- ĐẶT CHỖ RỒI MỚI LĂN BÁNH --- */
      if(LF().TryReserveCell(m_unId, sNext, unNow + RES_AHEAD)) {
         ApplySteering(cNextCen, false);
         if(m_unSideCooldown > 0) --m_unSideCooldown;
         return;
      }
      /* Ô bị robot ngoài tầm RAB đặt trước -> chờ / leo thang */
      StopWheels();
      ++m_unBlockedTicks;
   }

   if(m_unSideCooldown > 0) --m_unSideCooldown;

   /* LEO THANG KHI BỊ CHẶN LÂU: 3 s -> vẽ đường né các ô đã bị đặt chỗ,
    * 8 s -> né luôn vị trí mọi hàng xóm (đổi hẳn sang hành lang khác),
    * 16 s -> xóa đếm để chu kỳ chờ/né lặp lại từ đầu. */
   if(m_unBlockedTicks == 30) {
      PlanPath(m_sGoalCell, 1);
   }
   else if(m_unBlockedTicks == 80) {
      PlanPath(m_sGoalCell, 2);
   }
   else if(m_unBlockedTicks > 160) {
      m_unBlockedTicks = 0;
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::IWinAgainst(const SNeighbor& s_n) const {
   /* So quyền ưu tiên trao đổi qua RAB; hòa thì Id nhỏ thắng —
    * luật phá hòa tất định để hai bên luôn kết luận GIỐNG NHAU
    * mà không cần thêm vòng bắt tay nào. */
   UInt8 unMine = GetPriority();
   if(unMine != s_n.Prio) return unMine < s_n.Prio;
   return m_unId < s_n.Id;
}

/****************************************/
/****************************************/

bool CFootBotGrid::TrySidestep(const SNeighbor& s_opp,
                               const SGridCell& s_blocked) {
   /* Dạt sang ô vuông góc với hướng đang đi (chính là làn thứ hai của
    * hành lang đôi), hoặc lùi về ô vừa rời nếu hai bên đều kẹt.       */
   SInt32 nDR = s_blocked.Row - m_sCurCell.Row;
   SInt32 nDC = s_blocked.Col - m_sCurCell.Col;
   const SGridCell asCand[3] = {
      SGridCell(m_sCurCell.Row + nDC, m_sCurCell.Col - nDR),  /* trái  */
      SGridCell(m_sCurCell.Row - nDC, m_sCurCell.Col + nDR),  /* phải  */
      m_sPrevCell                                             /* lùi   */
   };
   for(const SGridCell& sCand : asCand) {
      if(!sCand.IsValid() || sCand == m_sCurCell) continue;
      if(CellTypeOf(sCand) != CELL_FREE) continue;         /* không trốn vào trạm */
      if(sCand == s_opp.Cur) continue;
      if(s_opp.Next.IsValid() && sCand == s_opp.Next) continue;
      SInt32 nOwner = LF().CellReserver(sCand);
      if(nOwner >= 0 && nOwner != (SInt32)m_unId) continue;
      if(!LF().TryReserveCell(m_unId, sCand, LF().Tick() + RES_AHEAD + 40))
         continue;

      m_sSideCell     = sCand;
      m_sYieldBlocked = s_blocked;
      m_unYieldOppId  = s_opp.Id;
      m_eYield        = YIELD_SIDESTEP;
      m_unYieldTimer  = 90;
      ++m_unSidestepCount;
      return true;
   }
   return false;
}

/****************************************/
/****************************************/

void CFootBotGrid::HandleYieldPhase() {
   switch(m_eYield) {

      case YIELD_SIDESTEP: {
         /* Đang lăn sang ô nhường */
         CVector2 cCen(ColToWorldX(m_sSideCell.Col), RowToWorldY(m_sSideCell.Row));
         SGridCell sEstCell(WorldYToRow(m_cEstPos.GetY()),
                            WorldXToCol(m_cEstPos.GetX()));
         if(sEstCell == m_sSideCell
            && (m_cEstPos - cCen).Length() < m_fWaypointTol) {
            LF().ReleaseCell(m_unId, m_sCurCell, RES_GRACE);
            m_sPrevCell    = m_sCurCell;
            m_sCurCell     = m_sSideCell;
            m_eYield       = YIELD_WAIT_CLEAR;
            m_unYieldTimer = 60;
            StopWheels();
            return;
         }
         ApplySteering(cCen, true);
         if(m_unYieldTimer > 0) --m_unYieldTimer;
         if(m_unYieldTimer == 0) {          /* kẹt giữa chừng: bỏ pha né */
            m_eYield = YIELD_NONE;
            m_vecPath.clear();
         }
         return;
      }

      case YIELD_WAIT_CLEAR: {
         /* Đứng nép trong làn phụ, chờ robot ưu tiên cao đi hết qua */
         StopWheels();
         bool bCleared = true;
         for(const SNeighbor& sN : m_vecNeighbors) {
            if(sN.Id != m_unYieldOppId) continue;
            /* đối thủ vẫn còn đứng trong ô tranh chấp -> chưa xong */
            if(sN.Cur == m_sYieldBlocked) bCleared = false;
            break;
         }
         if(m_unYieldTimer > 0) --m_unYieldTimer;
         if(bCleared || m_unYieldTimer == 0) {
            m_eYield         = YIELD_NONE;
            m_vecPath.clear();       /* vẽ lại lộ trình từ làn phụ */
            m_unBlockedTicks = 0;
         }
         return;
      }

      default:
         m_eYield = YIELD_NONE;
         return;
   }
}

/****************************************/
/****************************************/

SGridCell CFootBotGrid::PlannedNextCell() const {
   if(m_eYield == YIELD_SIDESTEP)    return m_sSideCell;
   if(m_eYield == YIELD_WAIT_CLEAR)  return SGridCell();   /* đứng yên */
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
      sN.Id    = sPkt.Data[RAB_IDX_ID];
      if(sN.Id == m_unId || sN.Id >= NUM_DOCKS) continue;
      sN.Prio  = sPkt.Data[RAB_IDX_PRIO];
      sN.State = sPkt.Data[RAB_IDX_STATE];
      sN.Flags = sPkt.Data[RAB_IDX_FLAGS];
      sN.Batt  = sPkt.Data[RAB_IDX_BATT];
      sN.Cur   = SGridCell(sPkt.Data[RAB_IDX_CUR_R], sPkt.Data[RAB_IDX_CUR_C]);
      if(sPkt.Data[RAB_IDX_NEXT_R] != RAB_NO_CELL)
         sN.Next = SGridCell(sPkt.Data[RAB_IDX_NEXT_R], sPkt.Data[RAB_IDX_NEXT_C]);
      sN.Range = sPkt.Range * 0.01;   /* cm -> m */
      m_vecNeighbors.push_back(sN);
   }
}

/****************************************/
/****************************************/

void CFootBotGrid::BroadcastState() {
   /* Gói 16 byte/tick: định danh, MỨC ƯU TIÊN, ô hiện tại, ô sắp bước
    * vào — đủ để mọi hàng xóm trong 3 m tự phân xử tranh chấp mà không
    * cần bất kỳ trọng tài trung tâm nào. */
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
   if(m_sTask.HasBox)          unFlags |= RAB_FLAG_CARRY;
   if(m_eYield != YIELD_NONE)  unFlags |= RAB_FLAG_YIELD;
   m_pcRABAct->SetData(RAB_IDX_FLAGS, unFlags);
   m_pcRABAct->SetData(RAB_IDX_BATT,
      static_cast<UInt8>(Max<Real>(0.0, Min<Real>(1.0, m_fBattery)) * 100.0));
}

}  /* namespace argos */
