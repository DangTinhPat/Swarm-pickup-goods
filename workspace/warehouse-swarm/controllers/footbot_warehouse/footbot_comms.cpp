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
 *           GOSSIP, NOT A GLOBAL FEED: a robot only has fresh (Age=0)
 *           belt info when it was physically close enough to sense it
 *           itself (see SenseBelts); everyone else's knowledge comes
 *           purely from repeating what a neighbor said, aging every
 *           tick — classic epidemic/gossip propagation, so information
 *           about a belt takes real (simulated) time to reach a robot
 *           on the far side of the facility, exactly like real limited-
 *           range swarm communication.
 *
 * A dead/stuck robot squatting on a belt's pickup point rides the same
 * channel: the loop functions fold that fact into the belt's own
 * Blocked bit (see SetBeltGroundTruth) rather than a separate distress
 * message — to the fleet, "this belt is unusable" is the same fact
 * whether the cause is a full queue or a corpse on the pickup point.
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
   /* Direct local sensing: within range, a real proximity/camera sensor
    * would read the belt's true queue depth and whether its pickup
    * point is obstructed — stamp the belief as confirmed THIS tick.
    * Out of range, nothing to do here: age is always DERIVED from
    * (now - OriginTick), never separately incremented, which is exactly
    * what makes this immune to the mutual-refresh gossip bug (see the
    * header comment on SBeltBelief). ProcessMessages() may still adopt
    * a neighbor's more recent OriginTick. */
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
   SInt32 nBest = -1;
   Real fBestDist = 1.0e9;
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(BeltAge(b) < BELT_INFO_MAX_AGE) continue;
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      if(fDist < fBestDist) {
         fBestDist = fDist;
         nBest = b;
      }
   }
   if(nBest < 0) return false;
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
      /* Belt gossip: epidemic propagation — reconstruct the neighbor's
       * OriginTick from their reported age and adopt it only if it is
       * MORE RECENT than what I already have. Adopting a reconstructed
       * absolute tick (not a relayed counter) is what makes this immune
       * to mutual reinforcement: OriginTick only ever moves forward, so
       * BeltAge() always reflects true elapsed time no matter how many
       * hops or how often the same information gets re-heard.
       *
       * The "+1" matters: RAB has exactly one tick of propagation delay
       * in ARGoS (a packet sent at tick T is read at T+1), so the
       * sender's age was computed one tick before I receive it. Without
       * adding that back, reconstruction UNDERSHOOTS by 1 tick at every
       * single hop — harmless for a single hop, but the error is
       * systematic and ADDS UP across a relay chain: information that
       * bounces through N robots looks N ticks fresher than it really
       * is, letting old data masquerade as recent indefinitely in a
       * densely-connected fleet (found by tracing a belief that stayed
       * at "age 1" for 80+ ticks straight while the real queue kept
       * changing underneath it). Accounting for the delay explicitly
       * keeps the reconstructed origin exact regardless of hop count. */
      for(UInt32 b = 0; b < NUM_BELTS; ++b) {
         UInt8 unQueue, unFreshness;
         cData >> unQueue >> unFreshness;
         /* Invert back (see Broadcast() for why the wire format is
          * freshness, not literal age) */
         UInt32 unTheirAge = 127 - (unFreshness & 0x7F);
         bool bTheirBlocked = (unFreshness & 0x80) != 0;
         UInt32 unTheirOrigin = (unTheirAge + 1 >= m_unTickCount) ? 0 : (m_unTickCount - unTheirAge - 1);
         if(unTheirOrigin > m_tBeltBelief[b].OriginTick) {
            m_tBeltBelief[b].Queue = unQueue;
            m_tBeltBelief[b].Blocked = bTheirBlocked;
            m_tBeltBelief[b].OriginTick = unTheirOrigin;
         }
      }
      /* Dock bookkeeping: mark claimed slots. Conflicts over MY slot are
       * resolved by POSSESSION: whoever is closer to the slot keeps it
       * (computable from range+bearing), so a parked/charging robot can
       * never be evicted by a distant claimer; ties break on id. */
      if(unDock < m_bSlotTaken.size()) {
         m_bSlotTaken[unDock] = true;
         /* Once truly parked (physically on the bay), ownership is
          * absolute — no stray/late claim can evict a docked robot. Only
          * a still-approaching robot can lose the race to a closer one. */
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
      /* Count only colleagues CLOSER to that belt than me: they are
       * ahead of me in the implicit queue, the ones behind me are not
       * my problem. This makes the swarm sort itself by distance. */
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
   if(m_eState == STATE_TO_BELT && m_nBeltChoice >= 0) {
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
   /* Belt gossip: repeat my current belief for each belt so neighbors
    * who haven't sensed it directly can still learn it, second-hand */
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      cData << m_tBeltBelief[b].Queue;
      /* Encode age INVERTED (freshness, high=fresh) rather than
       * literally. Why: at tick 1 (and potentially any time a neighbor
       * hasn't broadcast recently), ARGoS's RAB sensor can hand back a
       * default all-zero reading for a robot that has never called
       * SetData yet — indistinguishable, byte-for-byte, from a real
       * packet. If age=0 meant "just sensed, perfectly fresh" on the
       * wire, that phantom packet would look like the most trustworthy
       * possible sighting and instantly poison every real robot's
       * belief to "every belt is empty" before the simulation even
       * starts moving. Inverting means a phantom all-zero byte decodes
       * to freshness=0 -> age=127 (maximally stale), which can never
       * beat genuine knowledge; only a robot that truly just sensed
       * something (Age=0) ever sends the wire value 127. */
      UInt8 unWireAge = (UInt8)Min<UInt32>(BeltAge(b), 127);
      UInt8 unFreshness = 127 - unWireAge;
      if(m_tBeltBelief[b].Blocked) unFreshness |= 0x80;
      cData << unFreshness;
   }
   m_pcRABAct->SetData(cData);
}
