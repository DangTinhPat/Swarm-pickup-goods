/**
 * grid_metrics.cpp — Log trạng thái định kỳ và bảng tổng kết cuối phiên.
 */

#include "grid_loop_functions.h"
#include <controllers/footbot_grid/footbot_grid.h>

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

/****************************************/
/****************************************/

void CGridLoopFunctions::LogStatus() {
   Real fBattSum = 0.0;
   UInt32 unBots = 0;
   for(size_t i = 0; i < m_vecCtrls.size(); ++i) {
      if(m_vecCtrls[i] == nullptr) continue;
      fBattSum += m_vecCtrls[i]->GetBatteryFrac();
      ++unBots;
   }
   UInt32 unActive = 0;
   for(const SDemand& s : m_vecDemands) if(s.Active) ++unActive;
   UInt32 unWaiting = 0;
   for(const SConveyor& s : m_vecConveyors) unWaiting += s.Queue.size();

   LOG << "[grid-swarm] t=" << Tick()
       << " | da giao=" << m_unDeliveredTotal
       << " (D:" << m_arrDeliveredPerColor[BOX_RED]
       << " L:"  << m_arrDeliveredPerColor[BOX_GREEN]
       << " B:"  << m_arrDeliveredPerColor[BOX_BLUE] << ")"
       << " | yeu cau mo=" << unActive
       << " | hop cho=" << unWaiting
       << " | pin tb=" << (unBots > 0 ? (int)(100.0 * fBattSum / unBots) : 0) << "%"
       << " | khan cap=" << m_unEmergencies
       << " | va cham than=" << m_unHardCollisions
       << " | gan cham=" << m_unNearMisses << std::endl;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::PostExperiment() {
   LOG << "==================== KET QUA GRID-SWARM ====================" << std::endl;
   LOG << "Delivered total: " << m_unDeliveredTotal
       << " (DO:" << m_arrDeliveredPerColor[BOX_RED]
       << " XANH-LA:" << m_arrDeliveredPerColor[BOX_GREEN]
       << " XANH-DUONG:" << m_arrDeliveredPerColor[BOX_BLUE] << ")" << std::endl;
   LOG << "Emergencies (pin<20%): " << m_unEmergencies << std::endl;
   LOG << "Va cham than robot (<0.17m, PHAI=0): " << m_unHardCollisions << std::endl;
   LOG << "Gan cham (<0.19m, tham khao): " << m_unNearMisses << std::endl;
   Real fDistTotal = 0.0;
   UInt32 unDetourTotal = 0;
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      fDistTotal += m_vecDistance[i];
      UInt32 unDetour = m_vecCtrls[i] ? m_vecCtrls[i]->GetDetourCount() : 0;
      unDetourTotal += unDetour;
      LOG << "  fb" << i
          << " quang duong=" << m_vecDistance[i] << " m"
          << " | chot QR=" << (m_vecCtrls[i] ? m_vecCtrls[i]->GetSnapCount() : 0)
          << " | cuu ho lac=" << (m_vecCtrls[i] ? m_vecCtrls[i]->GetRelocCount() : 0)
          << " | dat lan cuc bo=" << unDetour
          << std::endl;
   }
   LOG << "Tong quang duong doi xe: " << fDistTotal << " m" << std::endl;
   LOG << "Tong so lan dat lan cuc bo (3-buoc): " << unDetourTotal << std::endl;
   LOG << "============================================================" << std::endl;
}

}  /* namespace argos */
