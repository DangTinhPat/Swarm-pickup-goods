/*
 * Market-based, decentralized task allocation. Each robot locally scores
 * every belt from its OWN belt belief (see footbot_comms.cpp — never a
 * facility-wide oracle) and who else is already heading there, and
 * commits to the best one:
 *      U(b) = W_QUEUE*queue(b) - W_INBOUND*inbound(b) - W_DISTANCE*distance(b)
 * No dispatcher assigns jobs: load balancing emerges from every robot
 * bidding with the same local rule. This file also owns the small job-
 * lifecycle setters (AssignItem/Deliver) the loop functions call when a
 * handover or delivery physically happens.
 */
#include "footbot_warehouse.h"

/****************************************/
/****************************************/

/* Utility weights for market-based belt selection */
static const Real W_QUEUE    = 1.5;  /* pull of waiting parcels */
static const Real W_INBOUND  = 1.0;  /* push of colleagues already coming */
static const Real W_DISTANCE = 0.35; /* travel cost, per meter */
static const Real HYSTERESIS = 0.8;  /* keep current belt unless clearly worse */
/* A parked robot leaves its slot only for a clearly worthwhile job; a
 * robot already on the floor takes any non-negative one. This stops the
 * whole dock from surging out at the first parcel and dribbling back. */
static const Real LEAVE_UTILITY = 0.3;
/* Keep-working floor: an already-committed robot holds its job (and may
 * still receive its parcel) down to this charge; only NEW jobs require
 * min_work_charge. The gap is hysteresis against take-job/return churn. */
static const Real KEEP_WORKING_CHARGE = 0.25;

/****************************************/
/****************************************/

void CFootBotWarehouse::AssignItem(UInt8 un_addr) {
   m_nCarryAddr = un_addr;
   m_eState = STATE_DELIVER;
   m_nBeltChoice = -1;
}

void CFootBotWarehouse::Deliver() {
   m_nCarryAddr = -1;
   m_eState = STATE_IDLE;
   m_nBeltChoice = -1;   /* re-bid next tick */
}

/****************************************/
/****************************************/

bool CFootBotWarehouse::WantsWork() const {
   return m_eState != STATE_CHARGE && m_eState != STATE_DEAD &&
          m_fCharge >= KEEP_WORKING_CHARGE;
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ChooseBelt() {
   /* Energy gates with hysteresis:
    * - a NEW job requires min_work_charge (e.g. 40%);
    * - an existing commitment survives down to KEEP_WORKING_CHARGE, so
    *   a robot that dipped slightly en route neither camps at the bin
    *   nor ping-pongs between charger and belt. */
   if(m_nBeltChoice < 0 && m_fCharge < m_fMinWorkCharge) {
      return;
   }
   if(m_nBeltChoice >= 0 && m_fCharge < KEEP_WORKING_CHARGE) {
      m_nBeltChoice = -1;
      return;
   }
   SInt8 nBest = -1;
   Real fBestU = -1.0e9;
   Real fCurrentU = -1.0e9;
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      /* Can't bid on what I don't (reliably) know: skip belts whose
       * belief is stale (never sensed, or too long since anyone told
       * me) or known blocked (e.g. a bricked robot sits on the pickup
       * point) — this is the gossip system's entire point: bidding uses
       * only locally-grounded knowledge, not a facility-wide oracle. */
      if(BeltAge(b) >= BELT_INFO_MAX_AGE) continue;
      if(m_tBeltBelief[b].Blocked) continue;
      if(m_tBeltBelief[b].Queue == 0) continue;   /* nothing to fetch there */
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      /* Energy-aware bidding: the emptier the battery, the more
       * expensive distance feels — low robots prefer near jobs and
       * leave the far ones to fresher colleagues */
      Real fU = W_QUEUE * m_tBeltBelief[b].Queue
              - W_INBOUND * m_unInbound[b]
              - W_DISTANCE * fDist * (2.0 - m_fCharge)
              /* small noise breaks the symmetry when identical robots
               * would otherwise all make the same choice on the same tick */
              + m_pcRNG->Uniform(CRange<Real>(0.0, 0.3));
      if((SInt8)b == m_nBeltChoice) fCurrentU = fU;
      if(fU > fBestU) {
         fBestU = fU;
         nBest = b;
      }
   }
   /* Work already covered by closer colleagues? Wait at the dock
    * instead of crowding the bins. Parked robots additionally demand a
    * clearly worthwhile job before giving up their slot. */
   bool bParked = m_nDockSlot >= 0 &&
                  (m_cDockSlots[m_nDockSlot] - m_cPos).Length() < 0.3;
   Real fCommitBar = bParked ? LEAVE_UTILITY : 0.0;
   if(nBest >= 0 && fBestU < fCommitBar) {
      nBest = -1;
   }
   /* Hysteresis: stick with the current belt unless clearly beaten (only
    * if that belief is still fresh enough to trust) */
   if(m_nBeltChoice >= 0 && BeltAge(m_nBeltChoice) < BELT_INFO_MAX_AGE &&
      m_tBeltBelief[m_nBeltChoice].Queue > 0 &&
      fCurrentU >= 0.0 && fCurrentU + HYSTERESIS >= fBestU) {
      return;
   }
   m_nBeltChoice = nBest;
}
