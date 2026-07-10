/*
 * Market-based, decentralized task allocation. Each robot scores every
 * belt from its own belt belief (footbot_comms.cpp — never a facility-wide
 * oracle) and commits to the best one:
 *     U(b) = W_QUEUE*queue(b) - W_INBOUND*inbound(b) - W_DISTANCE*distance(b)
 * Load balancing emerges from every robot bidding with the same local rule.
 *
 * Anti-dithering by design. The pathology being prevented is a robot that
 * leaves the dock for a flickering rumor, turns back, leaves again — moving
 * constantly without delivering. Two rules stop it:
 *   1. A committed robot NEVER returns to the dock on a marginal utility dip.
 *      It only drops its job on a hard fact: empty battery, the belt is
 *      blocked, or it got close enough to see the bin is actually empty.
 *      It MAY still switch to a clearly better belt (load balancing keeps
 *      working) — switching between belts is productive, not dithering.
 *   2. Every decision is held for DECISION_LOCK ticks before it can change,
 *      damping both dock churn and belt-to-belt flip-flopping.
 */
#include "footbot_warehouse.h"

static const Real W_QUEUE    = 1.5;
static const Real W_INBOUND  = 1.0;
static const Real W_DISTANCE = 0.35;
/* A parked robot demands a clearly worthwhile job before leaving its bay;
 * a robot already on the floor takes any non-negative one. */
static const Real LEAVE_UTILITY = 0.3;
/* Switch belts mid-trip only if the new one beats the current by this much. */
static const Real HYSTERESIS = 0.8;
/* A committed robot keeps its job down to this charge; only NEW jobs need
 * the higher min_work_charge. The gap is hysteresis against charge churn. */
static const Real KEEP_WORKING_CHARGE = 0.25;
/* Minimum ticks a fresh decision is held before it can change again. */
static const UInt32 DECISION_LOCK = 30;

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
   m_nBeltChoice = -1;
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
   if(m_nBeltChoice < 0 && m_fCharge < m_fMinWorkCharge) return;
   if(m_nBeltChoice >= 0 && m_fCharge < KEEP_WORKING_CHARGE) {
      m_nBeltChoice = -1;
      m_unDecisionLockUntil = m_unTickCount + DECISION_LOCK;
      return;
   }

   /* Hard abandon (→ back to dock), overriding the lock: the belt is blocked,
    * or I am now close enough to confirm the bin is empty for myself. */
   if(m_nBeltChoice >= 0) {
      bool bBlocked = m_tBeltBelief[m_nBeltChoice].Blocked;
      bool bArrivedEmpty =
         (m_cBelt[m_nBeltChoice] - m_cPos).Length() < m_fBeltSenseRange &&
         m_tBeltBelief[m_nBeltChoice].Queue == 0;
      if(bBlocked || bArrivedEmpty) {
         m_nBeltChoice = -1;
         m_unDecisionLockUntil = m_unTickCount + DECISION_LOCK;
         return;
      }
   }

   /* Hold the current decision (idle OR committed belt) until it settles. */
   if(m_unTickCount < m_unDecisionLockUntil) return;

   SInt8 nBest = -1;
   Real fBestU = -1.0e9;
   Real fCurrentU = -1.0e9;
   for(UInt32 b = 0; b < NUM_BELTS; ++b) {
      if(BeltAge(b) >= BELT_INFO_MAX_AGE) continue;
      if(m_tBeltBelief[b].Blocked) continue;
      if(m_tBeltBelief[b].Queue == 0) continue;
      Real fDist = (m_cBelt[b] - m_cPos).Length();
      /* Energy-aware: the emptier the battery, the more distance costs. */
      Real fU = W_QUEUE * m_tBeltBelief[b].Queue
              - W_INBOUND * m_unInbound[b]
              - W_DISTANCE * fDist * (2.0 - m_fCharge)
              + m_pcRNG->Uniform(CRange<Real>(0.0, 0.3));
      if((SInt8)b == m_nBeltChoice) fCurrentU = fU;
      if(fU > fBestU) {
         fBestU = fU;
         nBest = b;
      }
   }

   /* Committed: switch to a clearly better belt (load balancing), but never
    * fall back to the dock here — that is the dithering we are preventing. */
   if(m_nBeltChoice >= 0) {
      if(nBest >= 0 && nBest != m_nBeltChoice && fBestU > fCurrentU + HYSTERESIS) {
         m_nBeltChoice = nBest;
         m_unDecisionLockUntil = m_unTickCount + DECISION_LOCK;
      }
      return;
   }

   /* Idle: commit only to a worthwhile job (parked robots demand more). */
   bool bParked = m_nDockSlot >= 0 &&
                  (m_cDockSlots[m_nDockSlot] - m_cPos).Length() < 0.3;
   if(nBest >= 0 && fBestU < (bParked ? LEAVE_UTILITY : 0.0)) nBest = -1;
   if(nBest != m_nBeltChoice) m_unDecisionLockUntil = m_unTickCount + DECISION_LOCK;
   m_nBeltChoice = nBest;
}
