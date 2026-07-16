/*
 * RAB packets + the belt-status gossip system.
 *
 * RAB packet layout (14 bytes):
 *   [0]     UInt8  state (EState)
 *   [1-2]   UInt16 sender id (hash)
 *   [3]     UInt8  belt the sender is heading to (255 = none)
 *   [4]     UInt8  sender's distance to that belt, in 4 cm units, capped
 *   [5]     UInt8  address of the carried parcel (255 = none)
 *   [6]     UInt8  dock slot the sender claims (255 = none)
 *   [7]     UInt8  1 if the sender is mid deep-rescue (wall/pillar/cluster
 *                  extraction), else 0 — neighbors give it full priority
 *   [8-13]  belt gossip, 2 bytes per belt (see SBeltBelief):
 *             byte0 = Queue (0..queue_cap)
 *             byte1 = bit7 Blocked | bits0-6 Age (0..127, saturated)
 *
 * Belt info is GOSSIP, not a global feed: a robot knows a belt firsthand
 * only when close enough to sense it (SenseBelts); everyone else relays
 * what a neighbor said, aging every tick — classic epidemic propagation,
 * so news crosses the facility in real (simulated) time. A dead robot
 * squatting on a pickup point rides the same channel as the belt's own
 * Blocked bit (SetBeltGroundTruth) rather than a separate distress message.
 */
#include "footbot_warehouse.h"
#include <argos3/core/utility/datatypes/byte_array.h>
#include <algorithm>

/****************************************/
/****************************************/

static const UInt8 NO_BYTE = 255;

/****************************************/
/****************************************/

UInt8 CFootBotWarehouse::PriorityOf(UInt8 un_state) {
   switch(un_state) {
      case STATE_CHARGE:  return 3;  /* low battery: let it through */
      case STATE_DELIVER: return 2;  /* loaded: right of way */
      case STATE_TO_BELT: return 1;  /* fetching */
      default:            return 0;  /* idle / dead */
   }
}

/****************************************/
/****************************************/

void CFootBotWarehouse::SetBeltGroundTruth(const UInt8* pun_queues, const bool* pb_blocked) {
   for(UInt32 i = 0; i < NUM_BELTS; ++i) {
      m_unTrueQueue[i] = pun_queues[i];
      m_bTrueBlocked[i] = pb_blocked[i];
   }
}

/****************************************/
/****************************************/

UInt32 CFootBotWarehouse::BeltAge(UInt32 un_belt) const {
   if(m_tBeltBelief[un_belt].OriginTick == 0) return 0xFFFFFFFF;
   return m_unTickCount - m_tBeltBelief[un_belt].OriginTick;
}

/****************************************/
/****************************************/

void CFootBotWarehouse::SenseBelts() {
   /* Direct local sensing: within range, confirm the belt's true queue and
    * blocked flag and stamp the belief with this tick. Age is only ever
    * derived from (now - OriginTick), never incremented separately. */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      if(fDist < m_fBeltSenseRange) {
         m_tBeltBelief[b].Queue = m_unTrueQueue[b];
         m_tBeltBelief[b].Blocked = m_bTrueBlocked[b];
         m_tBeltBelief[b].OriginTick = m_unTickCount;
      }
   }
}

/****************************************/
/****************************************/

bool CFootBotWarehouse::NeedsBeltPatrol(CVector2& c_target) const {
   /* Scout only when BLIND — every belt's info has aged out. If even one
    * belt is fresh (sensed, or heard via gossip) the robot can bid on that
    * and has no reason to trek off, which is exactly what kept surplus
    * robots shuffling to far belts forever. Bootstrap still works: at
    * startup every belt is unknown, so idle robots do go look. */
   SInt32 nBest = -1;
   Real fBestDist = 1.0e9;
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(BeltAge(b) < BELT_INFO_MAX_AGE) return false;
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      if(fDist < fBestDist) {
         fBestDist = fDist;
         nBest = b;
      }
   }
   c_target = m_cBelt[nBest];
   return true;
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ProcessMessages() {
   m_tNeighbors.clear();
   std::fill(m_bSlotTaken.begin(), m_bSlotTaken.end(), false);
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABSens->GetReadings();
   for(size_t i = 0; i < tPackets.size(); ++i) {
      CByteArray cData = tPackets[i].Data;
      UInt8 unState, unBelt, unDist, unAddr, unDock, unRescuing;
      UInt16 unSenderId;
      cData >> unState >> unSenderId >> unBelt >> unDist >> unAddr >> unDock >> unRescuing;
      /* Remember the neighbor for priority-aware separation */
      m_tNeighbors.push_back({tPackets[i].Range,
                              tPackets[i].HorizontalBearing,
                              unState, unSenderId, unRescuing != 0});
      /* Belt gossip (epidemic propagation): reconstruct the neighbor's
       * absolute OriginTick and adopt it only if newer than mine. Using an
       * absolute tick (not a relayed "age" counter) keeps it monotonic and
       * immune to mutual reinforcement between two in-range robots. The -1
       * compensates RAB's fixed one-tick delay; without it the error would
       * compound per hop and let stale data look perpetually fresh. */
      for(UInt32 b = 0; b < NUM_BELTS; ++b) {
         UInt8 unQueue, unFreshness;
         cData >> unQueue >> unFreshness;
         UInt32 unTheirAge = 127 - (unFreshness & 0x7F);
         bool bTheirBlocked = (unFreshness & 0x80) != 0;
         UInt32 unTheirOrigin = (unTheirAge + 1 >= m_unTickCount) ? 0 : (m_unTickCount - unTheirAge - 1);
         if(unTheirOrigin > m_tBeltBelief[b].OriginTick) {
            m_tBeltBelief[b].Queue = unQueue;
            m_tBeltBelief[b].Blocked = bTheirBlocked;
            m_tBeltBelief[b].OriginTick = unTheirOrigin;
         }
      }
      /* Dock claims resolved by POSSESSION: whoever is closer to the slot
       * keeps it (ties break on id), so a docked robot is never evicted by
       * a distant claimer — only a still-approaching one can lose the race. */
      if(unDock < m_bSlotTaken.size()) {
         m_bSlotTaken[unDock] = true;
         bool bImTrulyParked = (SInt8)unDock == m_nDockSlot &&
            (m_cDockSlots[unDock] - m_cPos).Length() < DOCKED_DIST;
         if((SInt8)unDock == m_nDockSlot && !bImTrulyParked) {
            CVector2 cTheirPos = m_cPos +
               CVector2(tPackets[i].Range / 100.0,
                        m_cYaw + tPackets[i].HorizontalBearing);
            Real fTheirDist = (m_cDockSlots[unDock] - cTheirPos).Length();
            Real fMyDist = (m_cDockSlots[unDock] - m_cPos).Length();
            if(fTheirDist < fMyDist - 0.02 ||
               (Abs(fTheirDist - fMyDist) <= 0.02 && unSenderId < m_unRobotId)) {
               m_nDockSlot = -1;
            }
         }
      }
      /* Count only colleagues closer to the belt than me (ahead of me in
       * the implicit queue): this makes the swarm self-sort by distance. */
      if(unState == STATE_TO_BELT && unBelt < NUM_BELTS) {
         Real fTheirDist = unDist * 0.04;
         Real fMyDist = (m_cBelt[unBelt] - m_cPos).Length();
         if(fTheirDist < fMyDist) {
            ++m_unInbound[unBelt];
         }
      }
   }
}

/****************************************/
/****************************************/

void CFootBotWarehouse::Broadcast() {
   CByteArray cData;
   cData << (UInt8)m_eState;
   cData << m_unRobotId;
   /* A frozen (operator-stopped) robot must not keep claiming its belt:
    * neighbors count inbound claimers when bidding, and a claim from a
    * robot that is not actually moving would deter them from real work. */
   if(m_eState == STATE_TO_BELT && m_nBeltChoice >= 0 && m_eOverride == OP_AUTO) {
      cData << (UInt8)m_nBeltChoice;
      Real fDist = (m_cBelt[m_nBeltChoice] - m_cPos).Length();
      cData << (UInt8)Min<Real>(fDist / 0.04, 255.0);
   }
   else {
      cData << NO_BYTE << NO_BYTE;
   }
   cData << (UInt8)(m_nCarryAddr >= 0 ? m_nCarryAddr : NO_BYTE);
   cData << (UInt8)(m_nDockSlot >= 0 ? m_nDockSlot : NO_BYTE);
   cData << (UInt8)(m_bRescuing ? 1 : 0);
   /* Belt gossip: relay my belief so neighbors can learn it second-hand.
    * Age is encoded INVERTED (as freshness, high=fresh): ARGoS hands back
    * an all-zero packet for a robot that has not broadcast yet, and if
    * age=0 meant "freshest" that phantom would poison every belief to
    * "empty" at startup. As freshness, a zero byte decodes to maximally
    * stale, which can never beat genuine knowledge. */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      cData << m_tBeltBelief[b].Queue;
      UInt8 unWireAge = (UInt8)Min<UInt32>(BeltAge(b), 127);
      UInt8 unFreshness = 127 - unWireAge;
      if(m_tBeltBelief[b].Blocked) unFreshness |= 0x80;
      cData << unFreshness;
   }
   m_pcRABAct->SetData(cData);
}
