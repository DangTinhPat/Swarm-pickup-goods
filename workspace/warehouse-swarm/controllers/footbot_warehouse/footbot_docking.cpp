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
   /* Real ground sensor readings: 1.0 = white/floor, 0.0 = black. The
    * loop functions paint the dock-area zone darker the more activity
    * that side has recently seen (stigmergy), so darkness here is a
    * purely local, physically-sensed proxy for "this side is busy" —
    * a robot only ever perceives it once its own footprint is over
    * the tinted patch, never from across the warehouse. */
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
   /* Nearest slot nobody claims (per the last round of broadcasts),
    * with a stigmergic nudge away from a side I currently sense as
    * busy (see the header comment on GroundDarkness for why this can
    * only ever fire once already near a side, never from across the
    * warehouse — that's the whole nature of stigmergy). The penalty is
    * a soft, meters-equivalent score bump, not a hard exclusion: a much
    * closer slot on the "busy" side still wins if the calmer side is
    * far away, this only tips genuinely close calls. */
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
