/*
 * Core lifecycle (Init/Reset/Destroy) + the PreStep per-tick orchestrator.
 * Behavior-specific logic lives in the sibling warehouse_*.cpp files
 * listed in warehouse_loop_functions.h; this file wires them together.
 */
#include "warehouse_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <controllers/footbot_warehouse/footbot_warehouse.h>

/****************************************/
/****************************************/

CWarehouseLoopFunctions::CWarehouseLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_fZoneHalf(0.4),
   m_unSpawnPeriod(15),
   m_unQueueCap(6),
   m_fPickupRadius(0.35),
   m_fChargeRate(0.004),
   m_unChargeWarmup(50),
   m_fDrainTime(0.00002),
   m_fDrainMove(0.008),
   m_unDelivered(0),
   m_unCollisionTicks(0),
   m_fMinPairDistance(1000.0),
   m_fMinWallClearance(1000.0),
   m_fMinChargeSeen(1.0),
   m_unDeadTicks(0),
   m_fStigmergyDecay(0.998),
   m_fStigmergyGain(1.0) {
   for(UInt32 i = 0; i < NUM_ADDRS; ++i) m_unDeliveredPerAddr[i] = 0;
   m_fSideActivity[0] = 0.0;
   m_fSideActivity[1] = 0.0;
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tWh = GetNode(t_node, "warehouse");
      m_pcFloor = &GetSpace().GetFloorEntity();
      m_pcRNG = CRandom::CreateRNG("argos");

      GetNodeAttribute(tWh, "belt0", m_cBeltPickup[0]);
      GetNodeAttribute(tWh, "belt1", m_cBeltPickup[1]);
      GetNodeAttribute(tWh, "belt2", m_cBeltPickup[2]);
      GetNodeAttribute(tWh, "addr_a", m_cAddrPos[0]);
      GetNodeAttribute(tWh, "addr_b", m_cAddrPos[1]);
      GetNodeAttribute(tWh, "addr_c", m_cAddrPos[2]);
      GetNodeAttribute(tWh, "addr_d", m_cAddrPos[3]);
      GetNodeAttribute(tWh, "addr_e", m_cAddrPos[4]);
      UInt32 unRows, unCols;
      Real fSpacingX, fSpacingY;
      GetNodeAttribute(tWh, "dock_center", m_cDockCenter);
      GetNodeAttribute(tWh, "dock_rows", unRows);
      GetNodeAttribute(tWh, "dock_cols", unCols);
      GetNodeAttribute(tWh, "dock_spacing_x", fSpacingX);
      GetNodeAttribute(tWh, "dock_spacing_y", fSpacingY);
      for(UInt32 r = 0; r < unRows; ++r) {
         for(UInt32 cc = 0; cc < unCols; ++cc) {
            m_cDockSlots.push_back(m_cDockCenter + CVector2(
               (Real)((SInt32)r - ((SInt32)unRows - 1) / 2.0) * fSpacingX,
               (Real)((SInt32)cc - ((SInt32)unCols - 1) / 2.0) * fSpacingY));
         }
      }
      m_unSlotStatus.resize(m_cDockSlots.size(), 0);
      GetNodeAttributeOrDefault(tWh, "zone_half", m_fZoneHalf, m_fZoneHalf);
      GetNodeAttribute(tWh, "spawn_period", m_unSpawnPeriod);
      GetNodeAttribute(tWh, "queue_cap", m_unQueueCap);
      GetNodeAttribute(tWh, "pickup_radius", m_fPickupRadius);
      GetNodeAttributeOrDefault(tWh, "charge_rate", m_fChargeRate, m_fChargeRate);
      GetNodeAttributeOrDefault(tWh, "charge_warmup", m_unChargeWarmup, m_unChargeWarmup);
      GetNodeAttributeOrDefault(tWh, "drain_time", m_fDrainTime, m_fDrainTime);
      GetNodeAttributeOrDefault(tWh, "drain_move", m_fDrainMove, m_fDrainMove);
      GetNodeAttributeOrDefault(tWh, "stigmergy_decay", m_fStigmergyDecay, m_fStigmergyDecay);
      GetNodeAttributeOrDefault(tWh, "stigmergy_gain", m_fStigmergyGain, m_fStigmergyGain);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing warehouse loop functions!", ex);
   }

   /* Stagger the fleet's state of charge (0.55..0.95) so the robots do
    * not all need their first recharge in the same minute */
   CSpace::TMapPerType& cFootBots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CBatteryEquippedEntity& cBattery = cFootBot.GetBatterySensorEquippedEntity();
      cBattery.SetAvailableCharge(
         cBattery.GetFullCharge() *
         m_pcRNG->Uniform(CRange<Real>(0.55, 0.95)));
   }
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::Reset() {
   for(UInt32 b = 0; b < NUM_BELTS; ++b) m_cQueues[b].clear();
   m_unDelivered = 0;
   for(UInt32 i = 0; i < NUM_ADDRS; ++i) m_unDeliveredPerAddr[i] = 0;
   m_unCollisionTicks = 0;
   m_fMinPairDistance = 1000.0;
   m_fMinWallClearance = 1000.0;
   m_fMinChargeSeen = 1.0;
   m_unDeadTicks = 0;
   std::fill(m_unSlotStatus.begin(), m_unSlotStatus.end(), (UInt8)0);
   m_mapWarmup.clear();
   m_fSideActivity[0] = 0.0;
   m_fSideActivity[1] = 0.0;
   m_mapWasParked.clear();
   m_mapLastPos.clear();
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::Destroy() {
   /* How many robots ended the run parked on a dock slot? */
   UInt32 unDocked = 0;
   CSpace::TMapPerType& cFootBots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CVector2 cPos(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                    cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
         if((cPos - m_cDockSlots[s]).Length() < 0.12) {
            ++unDocked;
            break;
         }
      }
   }
   LOG << "[warehouse] Robots docked at end: " << unDocked << std::endl;
   LOG << "[warehouse] Delivered total: " << m_unDelivered << " (";
   for(UInt32 i = 0; i < NUM_ADDRS; ++i) {
      LOG << (char)('A' + i) << "=" << m_unDeliveredPerAddr[i]
          << (i + 1 < NUM_ADDRS ? " " : "");
   }
   LOG << ") | collision pair-ticks: " << m_unCollisionTicks
       << " | closest pass: " << m_fMinPairDistance << " m"
       << " | closest wall: " << m_fMinWallClearance << " m" << std::endl;
   LOG << "[warehouse] Energy: lowest charge seen " << (m_fMinChargeSeen * 100.0)
       << "% | dead robot-ticks: " << m_unDeadTicks << std::endl;
   LOG.Flush();
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::PreStep() {
   UpdateStigmergyDecay();
   SpawnParcel();

   /* Ground truth for the belt-gossip system (see footbot_comms.cpp for
    * why this is only "sensed", not broadcast globally to everyone) */
   UInt8 unQueueLens[NUM_BELTS];
   bool bBlocked[NUM_BELTS];
   ComputeBeltGroundTruth(unQueueLens, bBlocked);

   std::vector<CVector2> cPositions;
   std::vector<CFootBotWarehouse*> cControllers;
   UpdateRobots(unQueueLens, bBlocked, cPositions, cControllers);

   HandoverAtBelts(cPositions, cControllers);
   UpdateCollisionMetrics(cPositions);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWarehouseLoopFunctions, "warehouse_loop_functions")
