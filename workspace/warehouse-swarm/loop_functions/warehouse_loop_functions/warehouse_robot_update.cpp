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

      /* Charging bays: after a 5 s docking handshake (warm-up) on the
       * bay, full power flows. Pad: orange while warming up, green while
       * charging. Leaving the bay resets the handshake.
       *
       * BUG FIXED HERE: during warm-up no charge was being added, yet
       * the battery's own discharge model keeps draining a little every
       * tick just for existing (the "time" component of time_motion) —
       * a robot that reached the bay with only a hair above the death
       * floor (0.005) could tick over into STATE_DEAD while sitting
       * right on the pad, mid-handshake. A physically connected charger
       * would never let that happen even during a negotiation delay, so
       * the warm-up phase now clamps charge to a small protective floor
       * (1%) instead of letting it free-fall — this never accelerates
       * charging for a healthy robot (it only ever pushes UP to 1%, and
       * any robot already above that is untouched), it just closes the
       * death window for the rare critical-arrival case. */
      CBatteryEquippedEntity& cBattery = cFootBot.GetBatterySensorEquippedEntity();
      bool bOnBay = false;
      for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
         if((cPos - m_cDockSlots[s]).Length() < 0.18) {
            bOnBay = true;
            /* Stigmergy: bump this side's trail on the false->true
             * "just arrived" transition only (not every tick it sits
             * there, or activity would climb without bound) */
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

   /* Repaint the floor when any bay changes status. (Reordered ahead of
    * handover relative to the pre-split code — safe, since dock-status
    * repainting and belt handover touch entirely disjoint state.) */
   if(cStatusNow != m_unSlotStatus) {
      m_unSlotStatus = cStatusNow;
      m_pcFloor->SetChanged();
   }
}
