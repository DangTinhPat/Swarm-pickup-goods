/*
 * Per-tick, per-robot pass: stigmergy trail decay, belt ground-truth
 * injection, docking/charging (+ the stigmergy activity bump and the
 * warm-up death-floor fix — see the inline comment), energy metrics, and
 * delivery detection. Bundled into one fleet iteration rather than four
 * separate ones since every step here operates on the SAME one robot at
 * a time with no cross-robot ordering dependency.
 */
#include "warehouse_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <controllers/footbot_warehouse/footbot_warehouse.h>

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::UpdateStigmergyDecay() {
   /* A little fainter every tick, regardless of whether anything
    * happens this tick — this is what makes it a genuine fading TRACE
    * (ant pheromone), not a permanent record. */
   Real fOldActivity0 = m_fSideActivity[0], fOldActivity1 = m_fSideActivity[1];
   m_fSideActivity[0] *= m_fStigmergyDecay;
   m_fSideActivity[1] *= m_fStigmergyDecay;
   if((SInt32)(fOldActivity0 * 20) != (SInt32)(m_fSideActivity[0] * 20) ||
      (SInt32)(fOldActivity1 * 20) != (SInt32)(m_fSideActivity[1] * 20)) {
      m_pcFloor->SetChanged();
   }
}

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::UpdateRobots(const UInt8* pun_queue_lens, const bool* pb_blocked,
                                            std::vector<CVector2>& c_positions,
                                            std::vector<CFootBotWarehouse*>& c_controllers) {
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
      c_positions.push_back(cPos);
      c_controllers.push_back(&cController);

      cController.SetBeltGroundTruth(pun_queue_lens, pb_blocked);

      CBatteryEquippedEntity& cBattery = cFootBot.GetBatterySensorEquippedEntity();

      std::map<std::string, CVector2>::iterator itLast =
         m_mapLastPos.find(cFootBot.GetId());
      if(itLast != m_mapLastPos.end()) {
         Real fStep = (cPos - itLast->second).Length();

         Real fDrain = m_fDrainTime + m_fDrainMove * fStep;
         cBattery.SetAvailableCharge(
            Max<Real>(0.0, cBattery.GetAvailableCharge() - fDrain));
         itLast->second = cPos;
      }
      else {
         m_mapLastPos[cFootBot.GetId()] = cPos;
      }

      /* Charging bays: after a warm-up handshake on the bay, full power
       * flows (pad orange while warming up, green while charging). During
       * warm-up the charge is clamped to a 1% protective floor so a robot
       * that arrived critically low can't dip under the death line while
       * the handshake completes. This only ever pushes charge UP to 1%. */
      bool bOnBay = false;
      for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
         if((cPos - m_cDockSlots[s]).Length() < 0.18) {
            bOnBay = true;
            /* Stigmergy: bump the side's trail only on the just-arrived
             * transition, else activity would climb without bound. */
            bool& bWasParked = m_mapWasParked[cFootBot.GetId()];
            if(!bWasParked) {
               UInt32 unSide = SlotIsLeftSide(s) ? 0 : 1;
               m_fSideActivity[unSide] += m_fStigmergyGain;
               m_pcFloor->SetChanged();
            }
            bWasParked = true;
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
                  Real fProtectiveFloor = 0.01 * cBattery.GetFullCharge();
                  if(cBattery.GetAvailableCharge() < fProtectiveFloor) {
                     cBattery.SetAvailableCharge(fProtectiveFloor);
                  }
                  cStatusNow[s] = 1;
               }
            }
            break;
         }
      }
      if(!bOnBay) {
         m_mapWarmup.erase(cFootBot.GetId());
         m_mapWasParked[cFootBot.GetId()] = false;
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

   /* Repaint the floor when any bay changes status. */
   if(cStatusNow != m_unSlotStatus) {
      m_unSlotStatus = cStatusNow;
      m_pcFloor->SetChanged();
   }
}
