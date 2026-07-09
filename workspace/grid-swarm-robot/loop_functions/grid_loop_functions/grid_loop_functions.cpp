/**
 * grid_loop_functions.cpp — Vòng đời loop functions: Init/Reset và
 * PostStep điều phối các mô-đun (năng lượng, an toàn, sinh hàng, dọn
 * rác đặt chỗ, log định kỳ).
 */

#include "grid_loop_functions.h"
#include <controllers/footbot_grid/footbot_grid.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

/****************************************/
/****************************************/

void CGridLoopFunctions::Init(TConfigurationNode& t_tree) {
   if(NodeExists(t_tree, "grid")) {
      TConfigurationNode& tGrid = GetNode(t_tree, "grid");
      GetNodeAttributeOrDefault(tGrid, "max_active_demands",  m_unMaxActiveDemands, m_unMaxActiveDemands);
      GetNodeAttributeOrDefault(tGrid, "demand_period",       m_unDemandPeriod,     m_unDemandPeriod);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_min",     m_unBoxRespawnMin,    m_unBoxRespawnMin);
      GetNodeAttributeOrDefault(tGrid, "box_respawn_max",     m_unBoxRespawnMax,    m_unBoxRespawnMax);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_min", m_unCooldownMin,      m_unCooldownMin);
      GetNodeAttributeOrDefault(tGrid, "demand_cooldown_max", m_unCooldownMax,      m_unCooldownMax);
      GetNodeAttributeOrDefault(tGrid, "discharging_factor",  m_fDischargingFactor, m_fDischargingFactor);
      GetNodeAttributeOrDefault(tGrid, "charging_factor",     m_fChargingFactor,    m_fChargingFactor);
      GetNodeAttributeOrDefault(tGrid, "log_period",          m_unLogPeriod,        m_unLogPeriod);
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
/****************************************/

void CGridLoopFunctions::PostStep() {
   const UInt32 unTick = Tick();

   UpdateEnergyAndOdometry();
   MonitorProximity();
   UpdateConveyorSpawns();
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

REGISTER_LOOP_FUNCTIONS(CGridLoopFunctions, "grid_loop_functions")

}  /* namespace argos */
