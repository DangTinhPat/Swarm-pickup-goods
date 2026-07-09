/**
 * grid_reservations.cpp — Ma trận chiếm dụng động theo tick:
 * m_sGridReservations[Tick][(Row,Col)] = RobotID. Đặt chỗ theo nguyên
 * tắc all-or-nothing trên trọn cửa sổ [now, until] để không để lại
 * khoảng hở thời gian giữa hai robot.
 */

#include "grid_loop_functions.h"

namespace argos {

/****************************************/
/****************************************/

bool CGridLoopFunctions::TryReserveCell(UInt8 un_id,
                                        const SGridCell& s_cell,
                                        UInt32 un_until) {
   if(!s_cell.IsValid()) return false;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   const UInt32 unTo  = Max(un_until, unNow);
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);

   for(UInt32 t = unNow; t <= unTo; ++t) {
      auto itTick = m_sGridReservations.find(t);
      if(itTick == m_sGridReservations.end()) continue;
      auto itCell = itTick->second.find(sKey);
      if(itCell != itTick->second.end() && itCell->second != un_id) return false;
   }
   for(UInt32 t = unNow; t <= unTo; ++t)
      m_sGridReservations[t][sKey] = un_id;
   return true;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::ReleaseCell(UInt8 un_id,
                                     const SGridCell& s_cell,
                                     UInt32 un_grace) {
   if(!s_cell.IsValid()) return;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unKeepUntil = Tick() + un_grace;   /* đuôi xe còn vắt vạch */
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);

   for(auto& sTickEntry : m_sGridReservations) {
      if(sTickEntry.first <= unKeepUntil) continue;
      auto it = sTickEntry.second.find(sKey);
      if(it != sTickEntry.second.end() && it->second == un_id)
         sTickEntry.second.erase(it);
   }
}

/****************************************/
/****************************************/

SInt32 CGridLoopFunctions::CellReserver(const SGridCell& s_cell) const {
   if(!s_cell.IsValid()) return -1;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);
   for(UInt32 t = unNow; t <= unNow + 3; ++t) {
      auto itTick = m_sGridReservations.find(t);
      if(itTick == m_sGridReservations.end()) continue;
      auto itCell = itTick->second.find(sKey);
      if(itCell != itTick->second.end()) return itCell->second;
   }
   return -1;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::PruneOldReservations() {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   for(auto it = m_sGridReservations.begin(); it != m_sGridReservations.end(); ) {
      if(it->first + 2 < unNow) it = m_sGridReservations.erase(it);
      else ++it;
   }
}

}  /* namespace argos */
