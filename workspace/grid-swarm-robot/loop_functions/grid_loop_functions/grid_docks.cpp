/**
 * grid_docks.cpp — Trạng thái dock sạc ẨN DANH hai biên Đông/Tây:
 * dock "trống" ⇔ không robot NÀO KHÁC đặt chỗ ô đó + không thân robot
 * nào khác đứng trong ô. Ai đến trước dùng trước, không phân biệt ID.
 */

#include "grid_loop_functions.h"

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

/****************************************/
/****************************************/

bool CGridLoopFunctions::DockFree(SInt32 n_dock, UInt8 un_id) const {
   if(n_dock < 0 || n_dock >= NUM_DOCKS) return false;
   const SGridCell sCell = DockCell(n_dock);
   SInt32 nOwner = CellReserver(sCell);
   if(nOwner >= 0 && nOwner != (SInt32)un_id) return false;
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(i == un_id || m_vecBots[i] == nullptr) continue;
      if(RobotCellOf(i) == sCell) return false;
   }
   return true;
}

/****************************************/
/****************************************/

SInt32 CGridLoopFunctions::NearestFreeDock(const SGridCell& s_from,
                                           UInt8 un_id) const {
   SInt32 nBest = -1, nBestDist = 0;
   for(SInt32 i = 0; i < NUM_DOCKS; ++i) {
      if(!DockFree(i, un_id)) continue;
      SInt32 nDist = s_from.ManhattanTo(DockCell(i));
      if(nBest < 0 || nDist < nBestDist) {
         nBest = i;
         nBestDist = nDist;
      }
   }
   return nBest;
}

}  /* namespace argos */
