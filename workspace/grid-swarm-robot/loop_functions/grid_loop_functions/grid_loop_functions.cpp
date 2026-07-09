/**
 * grid_loop_functions.cpp — hạ tầng nhà kho: ma trận đặt chỗ theo
 * tick, băng chuyền, yêu cầu màu qua mặt kệ, sạc dock ẩn danh với mô
 * hình pin định lượng, thống kê.
 * (Vẽ sàn nằm riêng ở grid_floor_render.cpp)
 */

#include "grid_loop_functions.h"
#include <controllers/footbot_grid/footbot_grid.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

#include <algorithm>

namespace argos {

/****************************************/
/****************************************/

CGridLoopFunctions::CGridLoopFunctions() {}

/****************************************/
/****************************************/

void CGridLoopFunctions::Init(TConfigurationNode& t_tree) {
   if(NodeExists(t_tree, "grid")) {
      TConfigurationNode& tGrid = GetNode(t_tree, "grid");
      GetNodeAttributeOrDefault(tGrid, "max_active_demands", m_unMaxActiveDemands, m_unMaxActiveDemands);
      GetNodeAttributeOrDefault(tGrid, "demand_period",      m_unDemandPeriod,     m_unDemandPeriod);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_min",    m_unBoxRespawnMin,    m_unBoxRespawnMin);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_max",    m_unBoxRespawnMax,    m_unBoxRespawnMax);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_min", m_unCooldownMin,     m_unCooldownMin);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_max", m_unCooldownMax,     m_unCooldownMax);
      GetNodeAttributeOrDefault(tGrid, "discharging_factor", m_fDischargingFactor, m_fDischargingFactor);
      GetNodeAttributeOrDefault(tGrid, "charging_factor",    m_fChargingFactor,    m_fChargingFactor);
      GetNodeAttributeOrDefault(tGrid, "log_period",         m_unLogPeriod,        m_unLogPeriod);
   }

   m_pcRNG   = CRandom::CreateRNG("argos");
   m_pcFloor = &GetSpace().GetFloorEntity();

   m_vecBots.assign(NUM_DOCKS, nullptr);
   m_vecCtrls.assign(NUM_DOCKS, nullptr);
   CSpace::TMapPerType& tBots = GetSpace().GetEntitiesByType("foot-bot");
   for(auto& tEntry : tBots) {
      CFootBotEntity* pcBot = any_cast<CFootBotEntity*>(tEntry.second);
      const std::string& strId = pcBot->GetId();
      size_t unDigit = strId.find_first_of("0123456789");
      if(unDigit == std::string::npos) continue;
      size_t unIdx = std::stoul(strId.substr(unDigit));
      if(unIdx >= m_vecBots.size()) continue;
      m_vecBots[unIdx]  = pcBot;
      m_vecCtrls[unIdx] = dynamic_cast<CFootBotGrid*>(
         &pcBot->GetControllableEntity().GetController());
   }

   InitDynamicState();

   LOG << "[grid-swarm] San sang: luoi " << GRID_ROWS << "x" << GRID_COLS
       << " o " << CELL_SIZE << "m, " << NUM_DOCKS << " dock (5 tay+5 dong), "
       << NUM_CONVEYORS << " bang chuyen, " << NUM_STACK_CELLS
       << " o mat ke (3 hang vat can cung)." << std::endl;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::InitDynamicState() {
   const UInt32 unNow = Tick();

   m_vecConveyors.clear();
   for(SInt32 i = 0; i < NUM_CONVEYORS; ++i) {
      SConveyor sConv;
      sConv.Cell      = ConveyorCell(i);
      sConv.RespawnAt = unNow + m_pcRNG->Uniform(CRange<UInt32>(5, 40));
      m_vecConveyors.push_back(sConv);
   }

   m_vecDemands.clear();
   for(SInt32 i = 0; i < NUM_STACK_CELLS; ++i) {
      SDemand sDem;
      sDem.Cell = StackCell(i);
      m_vecDemands.push_back(sDem);
   }
   for(UInt32 i = 0; i < 8; ++i) SpawnDemandIfPossible();

   m_sGridReservations.clear();
   for(auto& a : m_arrConflictLatch) a.fill(false);

   m_vecLastPos.assign(m_vecBots.size(), CVector2());
   m_vecDistance.assign(m_vecBots.size(), 0.0);
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      const CVector3& cP =
         m_vecBots[i]->GetEmbodiedEntity().GetOriginAnchor().Position;
      m_vecLastPos[i].Set(cP.GetX(), cP.GetY());
   }

   m_unDeliveredTotal = 0;
   m_arrDeliveredPerColor.fill(0);
   m_unEmergencies    = 0;
   m_unHardCollisions = 0;
   m_unNearMisses     = 0;
   m_bFloorDirty      = true;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::Reset() {
   InitDynamicState();
   m_pcFloor->SetChanged();
}

/****************************************/
/****************************************/

UInt32 CGridLoopFunctions::Tick() const {
   return GetSpace().GetSimulationClock();
}

SGridCell CGridLoopFunctions::RobotCellOf(size_t un_idx) const {
   const CVector3& cP =
      m_vecBots[un_idx]->GetEmbodiedEntity().GetOriginAnchor().Position;
   return SGridCell(WorldXToRow(cP.GetX()), WorldYToCol(cP.GetY()));
}

/****************************************/
/* MA TRẬN CHIẾM DỤNG ĐỘNG Grid[Tick][(Row,Col)] = RobotID              */
/****************************************/

bool CGridLoopFunctions::TryReserveCell(UInt8 un_id,
                                        const SGridCell& s_cell,
                                        UInt32 un_until) {
   if(!s_cell.IsValid()) return false;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   const UInt32 unTo  = Max(un_until, unNow);
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);

   /* Kiểm tra TOÀN BỘ cửa sổ [now, until]: nếu bất kỳ tick nào trong
    * khoảng đã bị ROBOT KHÁC chiếm ô này -> từ chối, không ghi gì cả
    * (đặt chỗ phải tất cả-hoặc-không-gì để không để lại khoảng hở). */
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

void CGridLoopFunctions::ReleaseCell(UInt8 un_id,
                                     const SGridCell& s_cell,
                                     UInt32 un_grace) {
   if(!s_cell.IsValid()) return;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   const UInt32 unKeepUntil = unNow + un_grace;
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);

   /* Xóa đặt chỗ của MÌNH cho mọi tick TƯƠNG LAI vượt quá ân hạn (đuôi
    * robot có thể còn vắt qua vạch nên giữ lại vài tick đầu). */
   for(auto& sTickEntry : m_sGridReservations) {
      if(sTickEntry.first <= unKeepUntil) continue;
      auto it = sTickEntry.second.find(sKey);
      if(it != sTickEntry.second.end() && it->second == un_id)
         sTickEntry.second.erase(it);
   }
}

SInt32 CGridLoopFunctions::CellReserver(const SGridCell& s_cell) const {
   if(!s_cell.IsValid()) return -1;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   const std::pair<SInt32, SInt32> sKey(s_cell.Row, s_cell.Col);
   /* "Sắp bị chiếm" = có chủ trong tick hiện tại hoặc vài tick tới */
   for(UInt32 t = unNow; t <= unNow + 3; ++t) {
      auto itTick = m_sGridReservations.find(t);
      if(itTick == m_sGridReservations.end()) continue;
      auto itCell = itTick->second.find(sKey);
      if(itCell != itTick->second.end()) return itCell->second;
   }
   return -1;
}

void CGridLoopFunctions::PruneOldReservations() {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const UInt32 unNow = Tick();
   for(auto it = m_sGridReservations.begin(); it != m_sGridReservations.end(); ) {
      if(it->first + 2 < unNow) it = m_sGridReservations.erase(it);
      else ++it;
   }
}

/****************************************/
/* BẢNG VIỆC — chỉ ghi nhận, không phân phối                            */
/****************************************/

bool CGridLoopFunctions::TryClaimTask(UInt8 un_id, SInt32 n_conv, SInt32 n_dem) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_conv < 0 || n_conv >= (SInt32)m_vecConveyors.size()) return false;
   if(n_dem  < 0 || n_dem  >= (SInt32)m_vecDemands.size())   return false;
   SConveyor& sConv = m_vecConveyors[n_conv];
   SDemand&   sDem  = m_vecDemands[n_dem];
   if(!sDem.Active || sDem.ClaimedBy >= 0) return false;
   /* Tìm 1 hộp CHƯA AI NHẬN trong hàng đợi trưng bày đúng màu yêu cầu
    * (không bắt buộc đúng thứ tự FIFO — cả 3 ô đang bày đều "lấy được"). */
   for(size_t k = 0; k < sConv.Queue.size(); ++k) {
      if(sConv.ClaimedBy[k] >= 0 || sConv.Queue[k] != sDem.Color) continue;
      sConv.ClaimedBy[k] = un_id;
      sDem.ClaimedBy     = un_id;
      return true;
   }
   return false;
}

bool CGridLoopFunctions::PickUpBox(UInt8 un_id, SInt32 n_conv, UInt8& un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_conv < 0 || n_conv >= (SInt32)m_vecConveyors.size()) return false;
   SConveyor& sConv = m_vecConveyors[n_conv];
   if(!(RobotCellOf(un_id) == sConv.Cell)) return false;
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

bool CGridLoopFunctions::DeliverBox(UInt8 un_id, SInt32 n_dem, UInt8 un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_dem < 0 || n_dem >= (SInt32)m_vecDemands.size()) return false;
   SDemand& sDem = m_vecDemands[n_dem];
   if(!sDem.Active || sDem.ClaimedBy != (SInt32)un_id) return false;
   if(sDem.Color != un_color)                          return false;
   /* Ô ngăn xếp là vật cản: chấp nhận giao từ MỘT TRONG HAI mặt kệ
    * liền kề (trước/sau dải obstacle) — không đòi hỏi đúng mặt đã
    * nhắm lúc nhận việc, vì robot có thể phải dạt làn dọc đường. */
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
/* DOCK SẠC ẨN DANH Ở HAI BIÊN ĐÔNG/TÂY                                 */
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

/****************************************/
/* VÒNG HẬU-TICK: pin định lượng, sinh hàng, an toàn, thống kê          */
/****************************************/

void CGridLoopFunctions::PostStep() {
   const UInt32 unTick = Tick();

   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      const CVector3& cP3 =
         m_vecBots[i]->GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector2 cP(cP3.GetX(), cP3.GetY());
      const Real fStep = (cP - m_vecLastPos[i]).Length();
      m_vecDistance[i] += fStep;
      m_vecLastPos[i]   = cP;

      CBatteryEquippedEntity& cBatt =
         m_vecBots[i]->GetBatterySensorEquippedEntity();
      const bool bAtDock  = CellTypeOf(WorldXToRow(cP.GetX()), WorldYToCol(cP.GetY())) == CELL_DOCK;
      const bool bMoving  = fStep > MOVE_EPSILON;

      if(bAtDock && !bMoving) {
         /* "Vận tốc bánh xe bằng 0" trong ô dock -> nạp theo charging_factor */
         Real fDelta = m_fChargingFactor * BASE_CHARGE_RATE;
         cBatt.SetAvailableCharge(
            Min(cBatt.GetFullCharge(), cBatt.GetAvailableCharge() + fDelta));
      }
      else if(bMoving) {
         /* "Di chuyển liên tục" ngoài dock -> xả theo discharging_factor */
         Real fDelta = m_fDischargingFactor * BASE_DISCHARGE_RATE;
         cBatt.SetAvailableCharge(Max<Real>(0.0, cBatt.GetAvailableCharge() - fDelta));
      }
      /* Đứng yên NGOÀI dock (chờ giao thông/bốc-hạ hộp): không xả,
       * không sạc — trung tính, giữ nguyên mức pin hiện tại. */
   }

   /* --- Giám sát an toàn vật lý: thân robot (đường kính 0.17 m) --- */
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      for(size_t j = i + 1; j < m_vecBots.size(); ++j) {
         if(m_vecBots[j] == nullptr) continue;
         Real fD = (m_vecLastPos[i] - m_vecLastPos[j]).Length();
         if(fD < 0.17 && !m_arrConflictLatch[i][j]) {
            ++m_unHardCollisions;
            m_arrConflictLatch[i][j] = true;
            LOGERR << "[grid-swarm] t=" << unTick << " VA CHAM THAN fb" << i
                   << "-fb" << j << " d=" << fD << " m" << std::endl;
         }
         /* Ngưỡng "gần chạm" = 0.19 m (không phải 0.20 m): khoảng cách
          * dock liền kề theo thiết kế đúng bằng 0.20 m, sai số dấu phẩy
          * động khi tính Length() có thể lệch dưới 0.20 m dù 2 robot
          * đang ĐỨNG YÊN đúng vị trí — dùng 0.19 m để không báo khống
          * hàng xóm dock tĩnh, vẫn bắt được các lần áp sát thật khi
          * di chuyển/dạt làn. */
         else if(fD < 0.19) {
            ++m_unNearMisses;
         }
         else if(fD > 0.22) {
            m_arrConflictLatch[i][j] = false;
         }
      }
   }

   /* Băng chuyền trưng bày tối đa 3 hộp cùng lúc: còn chỗ + tới hẹn
    * thì sinh thêm 1 hộp màu ngẫu nhiên, hẹn lại mốc kế tiếp. Nếu đầy
    * (3/3), mốc hẹn cứ giữ nguyên (đã quá hạn) nên hễ có chỗ trống
    * (sau khi robot lấy hộp) là băng chuyền nạp ngay hộp mới ở lần
    * PostStep kế tiếp — không cần chờ thêm một chu kỳ ngẫu nhiên mới. */
   for(SConveyor& sConv : m_vecConveyors) {
      if(sConv.Queue.size() < SConveyor::QUEUE_CAP && unTick >= sConv.RespawnAt) {
         sConv.Queue.push_back(PickBoxColor());
         sConv.ClaimedBy.push_back(-1);
         ++sConv.Produced;
         sConv.RespawnAt = unTick + m_pcRNG->Uniform(
            CRange<UInt32>(m_unBoxRespawnMin, m_unBoxRespawnMax + 1));
         m_bFloorDirty   = true;
      }
   }

   if(unTick % m_unDemandPeriod == 0) SpawnDemandIfPossible();

   if(m_bFloorDirty) {
      m_pcFloor->SetChanged();
      m_bFloorDirty = false;
   }

   PruneOldReservations();

   if(unTick % m_unLogPeriod == 0) LogStatus();
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
   LOG << "Gan cham (<0.20m, tham khao): " << m_unNearMisses << std::endl;
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

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CGridLoopFunctions, "grid_loop_functions")

}  /* namespace argos */
