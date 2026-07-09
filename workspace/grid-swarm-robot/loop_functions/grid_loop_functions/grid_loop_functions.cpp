/**
 * grid_loop_functions.cpp — hạ tầng nhà kho: ma trận đặt chỗ, băng
 * chuyền, yêu cầu màu, sạc dock ẩn danh, thống kê.
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

CGridLoopFunctions::CGridLoopFunctions() {
   for(auto& s : m_arrReservations) s = SCellReservation();
}

/****************************************/
/****************************************/

void CGridLoopFunctions::Init(TConfigurationNode& t_tree) {
   /* Tham số kịch bản */
   if(NodeExists(t_tree, "grid")) {
      TConfigurationNode& tGrid = GetNode(t_tree, "grid");
      GetNodeAttributeOrDefault(tGrid, "max_active_demands", m_unMaxActiveDemands, m_unMaxActiveDemands);
      GetNodeAttributeOrDefault(tGrid, "demand_period",      m_unDemandPeriod,     m_unDemandPeriod);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_min",    m_unBoxRespawnMin,    m_unBoxRespawnMin);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_max",    m_unBoxRespawnMax,    m_unBoxRespawnMax);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_min", m_unCooldownMin,     m_unCooldownMin);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_max", m_unCooldownMax,     m_unCooldownMax);
      GetNodeAttributeOrDefault(tGrid, "charge_rate",        m_fChargeRate,        m_fChargeRate);
      GetNodeAttributeOrDefault(tGrid, "time_factor",        m_fTimeFactor,        m_fTimeFactor);
      GetNodeAttributeOrDefault(tGrid, "pos_factor",         m_fPosFactor,         m_fPosFactor);
      GetNodeAttributeOrDefault(tGrid, "log_period",         m_unLogPeriod,        m_unLogPeriod);
   }

   m_pcRNG   = CRandom::CreateRNG("argos");
   m_pcFloor = &GetSpace().GetFloorEntity();

   /* Gom 10 foot-bot theo Id số ("fb3" -> chỉ số 3) */
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
       << ", " << NUM_DOCKS << " dock (5 trai + 5 phai), "
       << NUM_CONVEYORS << " bang chuyen, "
       << NUM_STACK_CELLS << " o ngan xep." << std::endl;
}

/****************************************/
/****************************************/

void CGridLoopFunctions::InitDynamicState() {
   const UInt32 unNow = Tick();

   /* Băng chuyền: hộp đầu tiên xuất hiện lệch pha nhau chút ít */
   m_vecConveyors.clear();
   for(SInt32 i = 0; i < NUM_CONVEYORS; ++i) {
      SConveyor sConv;
      sConv.Cell      = ConveyorCell(i);
      sConv.RespawnAt = unNow + m_pcRNG->Uniform(CRange<UInt32>(5, 40));
      m_vecConveyors.push_back(sConv);
   }

   /* 36 chỗ yêu cầu (mỗi ô ngăn xếp một chỗ), mở sẵn vài yêu cầu đầu */
   m_vecDemands.clear();
   for(SInt32 i = 0; i < NUM_STACK_CELLS; ++i) {
      SDemand sDem;
      sDem.Cell = StackCell(i);
      m_vecDemands.push_back(sDem);
   }
   for(UInt32 i = 0; i < 5; ++i) SpawnDemandIfPossible();

   for(auto& s : m_arrReservations) s = SCellReservation();
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
   m_unEmergencies = 0;
   m_unConflicts   = 0;
   m_bFloorDirty   = true;
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
   return SGridCell(WorldYToRow(cP.GetY()), WorldXToCol(cP.GetX()));
}

/****************************************/
/* MA TRẬN CHIẾM DỤNG Grid[Row][Col] + cửa sổ tick                      */
/****************************************/

bool CGridLoopFunctions::TryReserveCell(UInt8 un_id,
                                        const SGridCell& s_cell,
                                        UInt32 un_until) {
   if(!s_cell.IsValid()) return false;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   SCellReservation& sRes = m_arrReservations[s_cell.Row * GRID_COLS + s_cell.Col];
   const UInt32 unNow = Tick();
   /* Ô đang thuộc về robot khác và cửa sổ đặt chỗ chưa hết hạn -> từ chối */
   if(sRes.Owner >= 0 && sRes.Owner != (SInt32)un_id && sRes.Expiry > unNow)
      return false;
   sRes.Owner  = un_id;
   sRes.Expiry = Max(un_until, unNow + 1);
   return true;
}

void CGridLoopFunctions::ReleaseCell(UInt8 un_id,
                                     const SGridCell& s_cell,
                                     UInt32 un_grace) {
   if(!s_cell.IsValid()) return;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   SCellReservation& sRes = m_arrReservations[s_cell.Row * GRID_COLS + s_cell.Col];
   if(sRes.Owner != (SInt32)un_id) return;
   if(un_grace == 0) {
      sRes.Owner  = -1;
      sRes.Expiry = 0;
   }
   else {
      /* giữ thêm "ân hạn" vì đuôi robot có thể còn vắt qua vạch ô */
      sRes.Expiry = Min(sRes.Expiry, Tick() + un_grace);
   }
}

SInt32 CGridLoopFunctions::CellReserver(const SGridCell& s_cell) const {
   if(!s_cell.IsValid()) return -1;
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   const SCellReservation& sRes =
      m_arrReservations[s_cell.Row * GRID_COLS + s_cell.Col];
   return (sRes.Owner >= 0 && sRes.Expiry > Tick()) ? sRes.Owner : -1;
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
   if(!sConv.HasBox || sConv.ClaimedBy >= 0) return false;
   if(!sDem.Active  || sDem.ClaimedBy  >= 0) return false;
   if(sConv.Color != sDem.Color)             return false;
   sConv.ClaimedBy = un_id;
   sDem.ClaimedBy  = un_id;
   return true;
}

bool CGridLoopFunctions::PickUpBox(UInt8 un_id, SInt32 n_conv, UInt8& un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_conv < 0 || n_conv >= (SInt32)m_vecConveyors.size()) return false;
   SConveyor& sConv = m_vecConveyors[n_conv];
   if(!sConv.HasBox || sConv.ClaimedBy != (SInt32)un_id) return false;
   /* xác thực robot thật sự đứng ở miệng băng chuyền */
   if(!(RobotCellOf(un_id) == sConv.Cell)) return false;
   un_color        = sConv.Color;
   sConv.HasBox    = false;
   sConv.ClaimedBy = -1;
   sConv.RespawnAt = Tick() + m_pcRNG->Uniform(
      CRange<UInt32>(m_unBoxRespawnMin, m_unBoxRespawnMax + 1));
   m_bFloorDirty   = true;
   return true;
}

bool CGridLoopFunctions::DeliverBox(UInt8 un_id, SInt32 n_dem, UInt8 un_color) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   if(n_dem < 0 || n_dem >= (SInt32)m_vecDemands.size()) return false;
   SDemand& sDem = m_vecDemands[n_dem];
   if(!sDem.Active || sDem.ClaimedBy != (SInt32)un_id) return false;
   if(sDem.Color != un_color)                          return false;
   if(!(RobotCellOf(un_id) == sDem.Cell))              return false;
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
       << " vao o (" << sDem.Cell.Row << "," << sDem.Cell.Col
       << ") | tong=" << m_unDeliveredTotal << std::endl;
   return true;
}

void CGridLoopFunctions::AbandonTask(UInt8 un_id, bool b_keep_demand) {
   std::lock_guard<std::mutex> cLock(m_muxBoard);
   for(SConveyor& sConv : m_vecConveyors)
      if(sConv.ClaimedBy == (SInt32)un_id) sConv.ClaimedBy = -1;
   if(!b_keep_demand)
      for(SDemand& sDem : m_vecDemands)
         if(sDem.ClaimedBy == (SInt32)un_id) sDem.ClaimedBy = -1;
}

/****************************************/
/* DOCK SẠC ẨN DANH Ở HAI BIÊN                                          */
/****************************************/

bool CGridLoopFunctions::DockFree(SInt32 n_dock, UInt8 un_id) const {
   /* Một dock "trống" khi: (1) không robot NÀO KHÁC đang đặt chỗ ô đó
    * trên ma trận chiếm dụng, và (2) không có thân robot nào khác đang
    * đứng trong ô. KHÔNG có khái niệm "dock của robot X" — bất kỳ ai
    * thỏa 2 điều kiện trên đều cắm sạc được (ai đến trước dùng trước). */
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
   /* Duyệt cả 10 dock ở CẢ HAI biên trái/phải, chọn dock trống gần
    * nhất theo Manhattan — robot ở giữa kho sẽ tự nhiên tràn về biên
    * gần hơn, đúng hành vi "ai gần bên nào về bên đó". */
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
/* VÒNG HẬU-TICK: sạc, sinh hàng, thống kê                              */
/****************************************/

void CGridLoopFunctions::PostStep() {
   const UInt32 unTick = Tick();

   /* --- Robot: quãng đường + SẠC PIN vị trí (dock ẩn danh) --- */
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
      if(CellTypeOf(WorldYToRow(cP.GetY()), WorldXToCol(cP.GetX())) == CELL_DOCK) {
         /* Đứng trong ô dock = đang cắm sạc, bất kể là robot nào */
         cBatt.SetAvailableCharge(
            Min(cBatt.GetFullCharge(),
                cBatt.GetAvailableCharge() + m_fChargeRate * cBatt.GetFullCharge()));
      }
      else {
         /* Xả tuyến tính time_motion: phí thời gian + phí quãng đường */
         cBatt.SetAvailableCharge(
            Max<Real>(0.0,
                      cBatt.GetAvailableCharge()
                      - (m_fTimeFactor + m_fPosFactor * fStep)
                        * cBatt.GetFullCharge()));
      }
   }

   /* --- Giám sát an toàn: hai thân robot áp sát < 25 cm --- */
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      for(size_t j = i + 1; j < m_vecBots.size(); ++j) {
         if(m_vecBots[j] == nullptr) continue;
         Real fD = (m_vecLastPos[i] - m_vecLastPos[j]).Length();
         if(fD < 0.25 && !m_arrConflictLatch[i][j]) {
            ++m_unConflicts;
            m_arrConflictLatch[i][j] = true;
            LOGERR << "[grid-swarm] t=" << unTick << " AP SAT fb" << i
                   << "-fb" << j << " d=" << fD << " m" << std::endl;
         }
         else if(fD > 0.40) {
            m_arrConflictLatch[i][j] = false;
         }
      }
   }

   /* --- Băng chuyền nhả hộp mới --- */
   for(SConveyor& sConv : m_vecConveyors) {
      if(!sConv.HasBox && unTick >= sConv.RespawnAt) {
         sConv.HasBox    = true;
         sConv.ClaimedBy = -1;
         sConv.Color     = PickBoxColor();
         ++sConv.Produced;
         m_bFloorDirty   = true;
      }
   }

   /* --- Mở yêu cầu màu mới trên ngăn xếp --- */
   if(unTick % m_unDemandPeriod == 0) SpawnDemandIfPossible();

   if(m_bFloorDirty) {
      m_pcFloor->SetChanged();
      m_bFloorDirty = false;
   }

   if(unTick % m_unLogPeriod == 0) LogStatus();
}

/****************************************/
/****************************************/

void CGridLoopFunctions::SpawnDemandIfPossible() {
   UInt32 unActive = 0;
   for(const SDemand& s : m_vecDemands)
      if(s.Active) ++unActive;
   if(unActive >= m_unMaxActiveDemands) return;

   /* gom các ô ngăn xếp đang rảnh và đã hết thời gian nghỉ */
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
   /* 60% ưu tiên màu mà một ô ngăn xếp đang chờ (chưa ai nhận) để dòng
    * hàng không bị nghẽn vì lệch màu; còn lại chọn đều ngẫu nhiên. */
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
   /* Đối xứng với PickBoxColor: ưu tiên màu các hộp đang nằm chờ */
   if(m_pcRNG->Bernoulli(0.6)) {
      std::vector<UInt8> vecWaiting;
      for(const SConveyor& s : m_vecConveyors)
         if(s.HasBox && s.ClaimedBy < 0) vecWaiting.push_back(s.Color);
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
   for(const SConveyor& s : m_vecConveyors) if(s.HasBox) ++unWaiting;

   LOG << "[grid-swarm] t=" << Tick()
       << " | da giao=" << m_unDeliveredTotal
       << " (D:" << m_arrDeliveredPerColor[BOX_RED]
       << " L:"  << m_arrDeliveredPerColor[BOX_GREEN]
       << " B:"  << m_arrDeliveredPerColor[BOX_BLUE] << ")"
       << " | yeu cau mo=" << unActive
       << " | hop cho=" << unWaiting
       << " | pin tb=" << (unBots > 0 ? (int)(100.0 * fBattSum / unBots) : 0) << "%"
       << " | khan cap=" << m_unEmergencies
       << " | ap sat=" << m_unConflicts << std::endl;
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
   LOG << "Conflicts (<25cm): " << m_unConflicts << std::endl;
   Real fDistTotal = 0.0;
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      fDistTotal += m_vecDistance[i];
      LOG << "  fb" << i
          << " quang duong=" << m_vecDistance[i] << " m"
          << " | chot QR=" << (m_vecCtrls[i] ? m_vecCtrls[i]->GetSnapCount() : 0)
          << " | cuu ho lac=" << (m_vecCtrls[i] ? m_vecCtrls[i]->GetRelocCount() : 0)
          << " | dat lan nhuong=" << (m_vecCtrls[i] ? m_vecCtrls[i]->GetSidestepCount() : 0)
          << std::endl;
   }
   LOG << "Tong quang duong doi xe: " << fDistTotal << " m" << std::endl;
   LOG << "============================================================" << std::endl;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CGridLoopFunctions, "grid_loop_functions")

}  /* namespace argos */
