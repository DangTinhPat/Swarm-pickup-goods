/**
 * footbot_grid_detour.cpp — Dạt làn cục bộ ĐÚNG 3 BƯỚC cho robot thua
 * quyền (không bao giờ gọi A* toàn cục để né một robot):
 *   [1] rẽ vuông góc sang ô trống bên cạnh
 *   [2] tịnh tiến song song 1 ô, vượt qua robot đối diện
 *   [3] rẽ về làn cũ, vẽ lại lộ trình gốc từ vị trí mới
 * Cả hai phía đều nghẽn -> TRAFFIC_YIELDING: đứng im 1-2 tick, thử lại.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

namespace argos {

/****************************************/
/****************************************/

bool CFootBotGrid::TryStartDetour(const SGridCell& s_blocked) {
   const SInt32 dRow = s_blocked.Row - m_sCurCell.Row;
   const SInt32 dCol = s_blocked.Col - m_sCurCell.Col;
   const std::pair<SInt32, SInt32> aPerp[2] = {
      {  dCol, -dRow },
      { -dCol,  dRow }
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

      /* Đặt chỗ trước CẢ 3 bước: hai robot sẽ đi song song cách nhau
       * đúng 1 ô (dư 0.03m so với thân) — giữ trọn cửa sổ thời gian
       * để robot thứ ba không chen vào giữa lúc hai bên đang sát nhau */
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
      /* Xong 3 bước: đã về làn cũ, 1 ô sau ô từng bị chặn */
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
      if(m_unDetourIdx < m_vecDetourPath.size()) {
         const SGridCell& sNextStep = m_vecDetourPath[m_unDetourIdx];
         if(!LF().TryReserveCell(m_unId, sNextStep, LF().Tick() + RES_AHEAD)) {
            StopWheels();
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
      m_eTraffic = TRAFFIC_NONE;
   }
}

}  /* namespace argos */
