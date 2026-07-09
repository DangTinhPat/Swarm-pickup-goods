/**
 * grid_task_board.cpp — Bảng việc thụ động: ghi nhận claim nguyên tử,
 * xác thực bốc/giao tại chỗ, và sinh hộp/yêu cầu màu ngẫu nhiên.
 * Không phân việc cho ai — robot tự chọn rồi xin ghi.
 */

#include "grid_loop_functions.h"

#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

/****************************************/
/****************************************/

bool CGridLoopFunctions::TryClaimTask(UInt8 un_id, SInt32 n_conv, SInt32 n_dem) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_conv < 0 || n_conv >= (SInt32)m_vecConveyors.size()) return false;
   if(n_dem  < 0 || n_dem  >= (SInt32)m_vecDemands.size())   return false;
   SConveyor& sConv = m_vecConveyors[n_conv];
   SDemand&   sDem  = m_vecDemands[n_dem];
   if(!sDem.Active || sDem.ClaimedBy >= 0) return false;
   /* Nhận hộp bất kỳ trong hàng đợi khớp màu (không bắt buộc FIFO) */
   for(size_t k = 0; k < sConv.Queue.size(); ++k) {
      if(sConv.ClaimedBy[k] >= 0 || sConv.Queue[k] != sDem.Color) continue;
      sConv.ClaimedBy[k] = un_id;
      sDem.ClaimedBy     = un_id;
      return true;
   }
   return false;
}

/****************************************/
/****************************************/

bool CGridLoopFunctions::PickUpBox(UInt8 un_id, SInt32 n_conv, UInt8& un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_conv < 0 || n_conv >= (SInt32)m_vecConveyors.size()) return false;
   SConveyor& sConv = m_vecConveyors[n_conv];
   if(!(RobotCellOf(un_id) == sConv.Cell)) return false;   /* phải đứng tại miệng băng */
   for(size_t k = 0; k < sConv.Queue.size(); ++k) {
      if(sConv.ClaimedBy[k] != (SInt32)un_id) continue;
      un_color = sConv.Queue[k];
      sConv.Queue.erase(sConv.Queue.begin() + k);
      sConv.ClaimedBy.erase(sConv.ClaimedBy.begin() + k);
      m_bFloorDirty = true;
      return true;
   }
   return false;
}

/****************************************/
/****************************************/

bool CGridLoopFunctions::DeliverBox(UInt8 un_id, SInt32 n_dem, UInt8 un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_dem < 0 || n_dem >= (SInt32)m_vecDemands.size()) return false;
   SDemand& sDem = m_vecDemands[n_dem];
   if(!sDem.Active || sDem.ClaimedBy != (SInt32)un_id) return false;
   if(sDem.Color != un_color)                          return false;
   /* Chấp nhận giao từ MỘT TRONG HAI mặt kệ (robot có thể đã dạt làn) */
   SGridCell sRobotCell = RobotCellOf(un_id);
   SGridCell sFaceA = StackFaceCell(sDem.Cell, false);
   SGridCell sFaceB = StackFaceCell(sDem.Cell, true);
   if(!(sRobotCell == sFaceA || sRobotCell == sFaceB)) return false;

   sDem.Active        = false;
   sDem.ClaimedBy     = -1;
   sDem.CooldownUntil = Tick() + m_pcRNG->Uniform(
      CRange<UInt32>(m_unCooldownMin, m_unCooldownMax + 1));
   ++sDem.Fulfilled;
   ++m_unDeliveredTotal;
   ++m_arrDeliveredPerColor[un_color % NUM_BOX_COLORS];
   m_bFloorDirty = true;
   LOG << "[grid-swarm] t=" << Tick() << " fb" << (int)un_id
       << " giao hop " << BoxColorName(un_color)
       << " vao ke (" << sDem.Cell.Row << "," << sDem.Cell.Col
       << ") | tong=" << m_unDeliveredTotal << std::endl;
   return true;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::AbandonTask(UInt8 un_id, bool b_keep_demand) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   for(SConveyor& sConv : m_vecConveyors)
      for(SInt32& nOwner : sConv.ClaimedBy)
         if(nOwner == (SInt32)un_id) nOwner = -1;
   if(!b_keep_demand)
      for(SDemand& sDem : m_vecDemands)
         if(sDem.ClaimedBy == (SInt32)un_id) sDem.ClaimedBy = -1;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::UpdateConveyorSpawns() {
   /* Còn chỗ (<3 hộp) và tới hẹn -> nhả thêm 1 hộp; nếu đang đầy, mốc
    * hẹn giữ nguyên (quá hạn) nên hễ trống là nạp ngay tick sau. */
   const UInt32 unTick = Tick();
   for(SConveyor& sConv : m_vecConveyors) {
      if(sConv.Queue.size() < SConveyor::QUEUE_CAP && unTick >= sConv.RespawnAt) {
         sConv.Queue.push_back(PickBoxColor());
         sConv.ClaimedBy.push_back(-1);
         ++sConv.Produced;
         sConv.RespawnAt = unTick + m_pcRNG->Uniform(
            CRange<UInt32>(m_unBoxRespawnMin, m_unBoxRespawnMax + 1));
         m_bFloorDirty = true;
      }
   }
}

/****************************************/
/****************************************/

void CGridLoopFunctions::SpawnDemandIfPossible() {
   UInt32 unActive = 0;
   for(const SDemand& s : m_vecDemands)
      if(s.Active) ++unActive;
   if(unActive >= m_unMaxActiveDemands) return;

   std::vector<size_t> vecFree;
   const UInt32 unNow = Tick();
   for(size_t i = 0; i < m_vecDemands.size(); ++i)
      if(!m_vecDemands[i].Active && m_vecDemands[i].CooldownUntil <= unNow)
         vecFree.push_back(i);
   if(vecFree.empty()) return;

   SDemand& sDem = m_vecDemands[
      vecFree[m_pcRNG->Uniform(CRange<UInt32>(0, vecFree.size()))]];
   sDem.Active    = true;
   sDem.ClaimedBy = -1;
   sDem.Color     = PickDemandColor();
   m_bFloorDirty  = true;
}

/****************************************/
/****************************************/

/* Hai bộ chọn màu thiên vị 60% về phía "đối tác đang chờ" để dòng hàng
 * không nghẽn vì lệch màu giữa cung và cầu. */

UInt8 CGridLoopFunctions::PickBoxColor() {
   if(m_pcRNG->Bernoulli(0.6)) {
      std::vector<UInt8> vecWanted;
      for(const SDemand& s : m_vecDemands)
         if(s.Active && s.ClaimedBy < 0) vecWanted.push_back(s.Color);
      if(!vecWanted.empty())
         return vecWanted[m_pcRNG->Uniform(CRange<UInt32>(0, vecWanted.size()))];
   }
   return m_pcRNG->Uniform(CRange<UInt32>(0, NUM_BOX_COLORS));
}

UInt8 CGridLoopFunctions::PickDemandColor() {
   if(m_pcRNG->Bernoulli(0.6)) {
      std::vector<UInt8> vecWaiting;
      for(const SConveyor& s : m_vecConveyors)
         for(size_t k = 0; k < s.Queue.size(); ++k)
            if(s.ClaimedBy[k] < 0) vecWaiting.push_back(s.Queue[k]);
      if(!vecWaiting.empty())
         return vecWaiting[m_pcRNG->Uniform(CRange<UInt32>(0, vecWaiting.size()))];
   }
   return m_pcRNG->Uniform(CRange<UInt32>(0, NUM_BOX_COLORS));
}

}  /* namespace argos */
