/*
 * Dock-slot claiming (nearest free slot, per the last round of RAB
 * broadcasts processed in footbot_comms.cpp) plus the stigmergic bias
 * away from a dock side the robot currently senses as busy — see
 * GroundDarkness's declaration in footbot_warehouse.h for why this can
 * only ever act locally, never influence a decision from across the
 * warehouse.
 */
#include "footbot_warehouse.h"

/****************************************/
/****************************************/

CVector2 CFootBotWarehouse::ComputeDockTarget(bool& b_exempt) {
   if(m_nDockSlot < 0) {
      ChooseDockSlot();
   }
   if(m_nDockSlot >= 0) {
      b_exempt = (m_cDockSlots[m_nDockSlot] - m_cPos).Length() < DOCKED_DIST;
      return m_cDockSlots[m_nDockSlot];
   }
   /* No free slot heard of (transient): hold near the nearest bay row
    * until claims sort themselves out */
   Real fBest = 1.0e9;
   CVector2 cNearest;
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      Real fD = (m_cDockSlots[s] - m_cPos).Length();
      if(fD < fBest) { fBest = fD; cNearest = m_cDockSlots[s]; }
   }
   b_exempt = false;
   return cNearest;
}

/****************************************/
/****************************************/

Real CFootBotWarehouse::GroundDarkness() const {
   /* Ground sensor: 1.0 = white floor, 0.0 = black. The loop functions
    * tint each dock side darker the busier it has recently been, so
    * darkness is a purely local, physically-sensed "this side is busy". */
   const CCI_FootBotMotorGroundSensor::TReadings& tReadings = m_pcGround->GetReadings();
   if(tReadings.empty()) return 0.0;
   Real fSum = 0.0;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      fSum += tReadings[i].Value;
   }
   return 1.0 - (fSum / tReadings.size());
}

/****************************************/
/****************************************/

void CFootBotWarehouse::ChooseDockSlot() {
   /* Nearest unclaimed slot, with a soft stigmergic penalty on slots on a
    * side I currently sense as busy. The penalty is a meters-equivalent
    * score bump, not a hard exclusion — it only tips genuinely close calls. */
   Real fDarkness = GroundDarkness();
   bool bImOnLeftSide = m_cPos.GetY() > 0.0;
   SInt8 nBest = -1;
   Real fBestScore = 1.0e9;
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(m_bSlotTaken[s]) continue;
      Real fScore = (m_cDockSlots[s] - m_cPos).Length();
      if(fDarkness > 0.3 && SlotIsLeftSide(s) == bImOnLeftSide) {
         fScore += fDarkness * 1.5;
      }
      if(fScore < fBestScore) {
         fBestScore = fScore;
         nBest = s;
      }
   }
   m_nDockSlot = nBest;
}
