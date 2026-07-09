/*
 * Parcel/belt lifecycle: conveyor spawn, the belt ground truth handed to
 * controllers for local sensing (see footbot_comms.cpp — this is deliberately
 * NOT broadcast globally), and bin handover to the nearest eligible robot.
 */
#include "warehouse_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_warehouse/footbot_warehouse.h>

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::SpawnParcel() {
   /* Conveyor output: a parcel with a random address lands in a random
    * belt's bin, continuously */
   if(GetSpace().GetSimulationClock() % m_unSpawnPeriod == 0) {
      UInt32 unBelt = m_pcRNG->Uniform(CRange<UInt32>(0, NUM_BELTS));
      if(m_cQueues[unBelt].size() < m_unQueueCap) {
         m_cQueues[unBelt].push_back(
            (UInt8)m_pcRNG->Uniform(CRange<UInt32>(0, NUM_ADDRS)));
      }
   }
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::ComputeBeltGroundTruth(UInt8* pun_queue_lens, bool* pb_blocked) {
   /* Queue depth, and whether a bricked/stuck robot is squatting on the
    * pickup point, blocking it. A pre-pass is needed because "blocked"
    * depends on EVERY robot's position, not just the one currently being
    * updated in the main per-robot loop (see warehouse_robot_update.cpp). */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      pun_queue_lens[b] = (UInt8)m_cQueues[b].size();
      pb_blocked[b] = false;
   }
   CSpace::TMapPerType& cFootBots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotWarehouse& cController =
         dynamic_cast<CFootBotWarehouse&>(cFootBot.GetControllableEntity().GetController());
      if(!cController.IsDead()) continue;
      CVector2 cPos(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                    cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      for(UInt32 b = 0; b < NUM_BELTS; ++b) {
         if((cPos - m_cBeltPickup[b]).Length() < m_fPickupRadius) {
            pb_blocked[b] = true;
         }
      }
   }
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::HandoverAtBelts(const std::vector<CVector2>& c_positions,
                                               const std::vector<CFootBotWarehouse*>& c_controllers) {
   /* Nearest empty-handed robot within reach gets the front parcel; one
    * handover per belt per tick — like a worker handing over one box at
    * a time */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(m_cQueues[b].empty()) continue;
      SInt32 nBest = -1;
      Real fBestDist = m_fPickupRadius;
      for(size_t i = 0; i < c_positions.size(); ++i) {
         if(c_controllers[i]->IsCarrying() || !c_controllers[i]->WantsWork()) continue;
         Real fDist = (c_positions[i] - m_cBeltPickup[b]).Length();
         if(fDist < fBestDist) {
            fBestDist = fDist;
            nBest = i;
         }
      }
      if(nBest >= 0) {
         c_controllers[nBest]->AssignItem(m_cQueues[b].front());
         m_cQueues[b].pop_front();
      }
   }
}
