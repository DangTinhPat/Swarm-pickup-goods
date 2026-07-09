/**
 * footbot_grid_pathfinding.cpp — A* 4 hướng trên lưới 30x30: phạt rẽ
 * 0.35 (chuộng đường thẳng theo làn), cấm tuyệt đối CELL_OBSTACLE, ô
 * dock/băng chuyền chỉ đi vào được khi là đích của chính robot.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <algorithm>

namespace argos {

/****************************************/
/****************************************/

bool CFootBotGrid::PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level) {
   /* Mức né động (lưới an toàn khi dạt làn cục bộ đã bó tay):
    *   0 - chỉ né vật cản tĩnh
    *   1 - né thêm ô bị robot khác đặt chỗ trong bán kính 6 ô
    *   2 - né thêm ô hiện tại + ô dự định của mọi hàng xóm RAB */

   constexpr SInt32 N = GRID_ROWS * GRID_COLS;
   auto Idx = [](const SGridCell& c) { return c.Row * GRID_COLS + c.Col; };

   std::array<bool, N> arrAvoid{};
   if(un_avoid_level >= 1) {
      for(SInt32 r = 0; r < GRID_ROWS; ++r)
         for(SInt32 c = 0; c < GRID_COLS; ++c) {
            SGridCell sCell(r, c);
            if(sCell.ManhattanTo(m_sCurCell) > 6) continue;
            SInt32 nOwner = LF().CellReserver(sCell);
            if(nOwner >= 0 && nOwner != (SInt32)m_unId)
               arrAvoid[Idx(sCell)] = true;
         }
   }
   if(un_avoid_level >= 2) {
      for(const SNeighbor& sN : m_vecNeighbors) {
         if(sN.Cur.IsValid())  arrAvoid[Idx(sN.Cur)]  = true;
         if(sN.Next.IsValid()) arrAvoid[Idx(sN.Next)] = true;
      }
   }
   arrAvoid[Idx(m_sCurCell)] = false;

   auto Passable = [&](const SGridCell& c) {
      if(!c.IsValid()) return false;
      if(CellTypeOf(c) == CELL_OBSTACLE) return false;
      if(arrAvoid[Idx(c)] && !(c == s_goal)) return false;
      EGridCellType e = CellTypeOf(c);
      return e == CELL_FREE || c == s_goal;
   };

   std::array<Real,   N> arrG;      arrG.fill(1.0e9);
   std::array<SInt32, N> arrParent; arrParent.fill(-1);
   std::array<SInt8,  N> arrDir;    arrDir.fill(-1);
   std::array<bool,   N> arrClosed{};
   const SInt32 DR[4] = { 0, 1, 0, -1 };
   const SInt32 DC[4] = { 1, 0, -1, 0 };

   SInt32 nStart = Idx(m_sCurCell), nGoal = Idx(s_goal);
   if(m_sCurCell == s_goal) {
      m_vecPath.clear(); m_unPathIdx = 0; m_sPathGoal = s_goal;
      return true;
   }
   arrG[nStart] = 0.0;
   std::vector<SInt32> vecOpen{ nStart };

   while(!vecOpen.empty()) {
      size_t unBest = 0;
      Real fBestF = 1.0e18;
      for(size_t i = 0; i < vecOpen.size(); ++i) {
         SInt32 n = vecOpen[i];
         SGridCell c(n / GRID_COLS, n % GRID_COLS);
         Real fF = arrG[n] + c.ManhattanTo(s_goal);
         if(fF < fBestF) { fBestF = fF; unBest = i; }
      }
      SInt32 nCur = vecOpen[unBest];
      vecOpen[unBest] = vecOpen.back();
      vecOpen.pop_back();
      if(nCur == nGoal) break;
      if(arrClosed[nCur]) continue;
      arrClosed[nCur] = true;

      SGridCell sCur(nCur / GRID_COLS, nCur % GRID_COLS);
      for(SInt32 d = 0; d < 4; ++d) {
         SGridCell sNb(sCur.Row + DR[d], sCur.Col + DC[d]);
         if(!Passable(sNb)) continue;
         SInt32 nNb = Idx(sNb);
         if(arrClosed[nNb]) continue;
         Real fTurn = (arrDir[nCur] >= 0 && arrDir[nCur] != d) ? 0.35 : 0.0;
         Real fNewG = arrG[nCur] + 1.0 + fTurn;
         if(fNewG < arrG[nNb]) {
            arrG[nNb]      = fNewG;
            arrParent[nNb] = nCur;
            arrDir[nNb]    = d;
            vecOpen.push_back(nNb);
         }
      }
   }

   if(arrParent[nGoal] < 0) return false;

   std::vector<SGridCell> vecRev;
   for(SInt32 n = nGoal; n != nStart; n = arrParent[n])
      vecRev.emplace_back(n / GRID_COLS, n % GRID_COLS);
   m_vecPath.assign(vecRev.rbegin(), vecRev.rend());
   m_unPathIdx = 0;
   m_sPathGoal = s_goal;
   return true;
}

}  /* namespace argos */
