#include "warehouse_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <controllers/footbot_warehouse/footbot_warehouse.h>

/****************************************/
/****************************************/

CColor CWarehouseLoopFunctions::AddressColor(UInt32 un_addr) {
   switch(un_addr) {
      case 0:  return CColor::RED;      /* A */
      case 1:  return CColor::GREEN;    /* B */
      case 2:  return CColor::BLUE;     /* C */
      case 3:  return CColor::YELLOW;   /* D */
      default: return CColor::MAGENTA;  /* E */
   }
}

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
   m_unDelivered(0),
   m_unCollisionTicks(0),
   m_fMinPairDistance(1000.0),
   m_fMinWallClearance(1000.0),
   m_fMinChargeSeen(1.0),
   m_unDeadTicks(0) {
   for(UInt32 i = 0; i < NUM_ADDRS; ++i) m_unDeliveredPerAddr[i] = 0;
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

CColor CWarehouseLoopFunctions::GetFloorColor(const CVector2& c_pos) {
   /* Charging bays: darker square per slot — GREEN while it is
    * actively charging a robot; a light pad tile around each slot */
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(Abs(c_pos.GetX() - m_cDockSlots[s].GetX()) < 0.14 &&
         Abs(c_pos.GetY() - m_cDockSlots[s].GetY()) < 0.14) {
         if(m_unSlotStatus[s] == 2) return CColor(60, 200, 90);    /* charging */
         if(m_unSlotStatus[s] == 1) return CColor(235, 180, 60);    /* warming up */
         return CColor::GRAY50;
      }
   }
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(Abs(c_pos.GetX() - m_cDockSlots[s].GetX()) < 0.45 &&
         Abs(c_pos.GetY() - m_cDockSlots[s].GetY()) < 0.45) {
         return CColor::GRAY80;
      }
   }
   /* Address zones: light shade of the address color */
   for(UInt32 a = 0; a < NUM_ADDRS; ++a) {
      if(Abs(c_pos.GetX() - m_cAddrPos[a].GetX()) < m_fZoneHalf &&
         Abs(c_pos.GetY() - m_cAddrPos[a].GetY()) < m_fZoneHalf) {
         CColor cFull = AddressColor(a);
         return CColor((cFull.GetRed() + 2 * 255) / 3,
                       (cFull.GetGreen() + 2 * 255) / 3,
                       (cFull.GetBlue() + 2 * 255) / 3);
      }
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::PreStep() {
   /* Conveyor output: a parcel with a random address lands in a random
    * belt's bin, continuously */
   if(GetSpace().GetSimulationClock() % m_unSpawnPeriod == 0) {
      UInt32 unBelt = m_pcRNG->Uniform(CRange<UInt32>(0, NUM_BELTS));
      if(m_cQueues[unBelt].size() < m_unQueueCap) {
         m_cQueues[unBelt].push_back(
            (UInt8)m_pcRNG->Uniform(CRange<UInt32>(0, NUM_ADDRS)));
      }
   }

   /* Gather robots; feed them the WMS queue info; handle deliveries */
   UInt8 unQueueLens[NUM_BELTS];
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      unQueueLens[b] = (UInt8)m_cQueues[b].size();
   }

   std::vector<CVector2> cPositions;
   std::vector<CFootBotWarehouse*> cControllers;
   std::vector<UInt8> cStatusNow(m_cDockSlots.size(), 0);
   CSpace::TMapPerType& cFootBots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotWarehouse& cController =
         dynamic_cast<CFootBotWarehouse&>(cFootBot.GetControllableEntity().GetController());
      CVector2 cPos(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                    cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      cPositions.push_back(cPos);
      cControllers.push_back(&cController);

      cController.SetBeltQueues(unQueueLens);

      /* Charging bays: after a 5 s docking handshake (warm-up) on the
       * bay, power flows. Pad: orange while warming up, green while
       * charging. Leaving the bay resets the handshake. */
      CBatteryEquippedEntity& cBattery = cFootBot.GetBatterySensorEquippedEntity();
      bool bOnBay = false;
      for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
         if((cPos - m_cDockSlots[s]).Length() < 0.18) {
            bOnBay = true;
            UInt32& unWarm = m_mapWarmup[cFootBot.GetId()];
            ++unWarm;
            if(cBattery.GetAvailableCharge() < cBattery.GetFullCharge()) {
               if(unWarm >= m_unChargeWarmup) {
                  cBattery.SetAvailableCharge(
                     Min(cBattery.GetFullCharge(),
                         cBattery.GetAvailableCharge() +
                         m_fChargeRate * cBattery.GetFullCharge()));
                  cStatusNow[s] = 2;
               }
               else {
                  cStatusNow[s] = 1;
               }
            }
            break;
         }
      }
      if(!bOnBay) {
         m_mapWarmup.erase(cFootBot.GetId());
      }
      /* Energy metrics */
      Real fFrac = cBattery.GetAvailableCharge() / cBattery.GetFullCharge();
      if(fFrac < m_fMinChargeSeen) m_fMinChargeSeen = fFrac;
      if(fFrac <= 0.005) ++m_unDeadTicks;

      /* Delivery: loaded robot inside its parcel's address zone */
      if(cController.IsCarrying()) {
         UInt32 unAddr = cController.GetCarriedAddress();
         if(Abs(cPos.GetX() - m_cAddrPos[unAddr].GetX()) < m_fZoneHalf &&
            Abs(cPos.GetY() - m_cAddrPos[unAddr].GetY()) < m_fZoneHalf) {
            cController.Deliver();
            ++m_unDelivered;
            ++m_unDeliveredPerAddr[unAddr];
            LOG << "[warehouse] " << cFootBot.GetId()
                << " delivered to " << (char)('A' + unAddr)
                << ". Total: " << m_unDelivered << std::endl;
         }
      }
   }

   /* Handover at the bins: nearest empty-handed robot within reach gets
    * the front parcel; one handover per belt per tick */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(m_cQueues[b].empty()) continue;
      SInt32 nBest = -1;
      Real fBestDist = m_fPickupRadius;
      for(size_t i = 0; i < cPositions.size(); ++i) {
         if(cControllers[i]->IsCarrying() || !cControllers[i]->WantsWork()) continue;
         Real fDist = (cPositions[i] - m_cBeltPickup[b]).Length();
         if(fDist < fBestDist) {
            fBestDist = fDist;
            nBest = i;
         }
      }
      if(nBest >= 0) {
         cControllers[nBest]->AssignItem(m_cQueues[b].front());
         m_cQueues[b].pop_front();
      }
   }

   /* Repaint the floor when any bay changes status */
   if(cStatusNow != m_unSlotStatus) {
      m_unSlotStatus = cStatusNow;
      m_pcFloor->SetChanged();
   }

   /* Collision metrics (same as ball-collector): body radius 0.085 m */
   for(size_t i = 0; i < cPositions.size(); ++i) {
      Real fWall = 3.95 - Max(Abs(cPositions[i].GetX()),
                              Abs(cPositions[i].GetY()));
      if(fWall < m_fMinWallClearance) {
         m_fMinWallClearance = fWall;
      }
      for(size_t j = i + 1; j < cPositions.size(); ++j) {
         Real fDist = (cPositions[i] - cPositions[j]).Length();
         if(fDist < 0.181) {
            ++m_unCollisionTicks;
         }
         if(fDist < m_fMinPairDistance) {
            m_fMinPairDistance = fDist;
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWarehouseLoopFunctions, "warehouse_loop_functions")
