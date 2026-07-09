/**
 * footbot_grid_steering.cpp — P-controller vi sai bám tâm ô:
 *   |err| > 55°: xoay tại chỗ ±3 cm/s (rẽ 90° đúng hồng tâm, chống trượt)
 *   ngược lại : v = V_Base·cos(err), ω = Kp·err
 *   ô đích cuối: phanh tỷ lệ theo khoảng cách, dừng trong goal_tol.
 */

#include "footbot_grid.h"

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::ApplySteering(const CVector2& c_target, bool b_final) {
   CVector2 cDelta = c_target - m_cEstPos;
   Real     fDist  = cDelta.Length();
   CRadians cErr   = ATan2(cDelta.GetY(), cDelta.GetX()) - m_cEstYaw;
   cErr.SignedNormalize();

   if(Abs(cErr) > m_cHardTurn) {
      Real fS = (cErr.GetValue() > 0.0) ? m_fPivotSpeed : -m_fPivotSpeed;
      m_pcWheels->SetLinearVelocity(-fS, fS);
      return;
   }

   Real fV = m_fCruiseSpeed * Max<Real>(0.15, Cos(cErr));
   if(b_final) fV = Min(fV, 300.0 * fDist);
   fV = Max<Real>(fV, 1.0);

   Real fOmega = m_fKpHeading * cErr.GetValue();
   Real fDiff  = fOmega * (m_fAxisM * 0.5) * 100.0;   /* rad/s -> cm/s   */
   Real fVL    = fV - fDiff;
   Real fVR    = fV + fDiff;
   Real fOver  = Max(Max(fVL, fVR) - m_fCruiseSpeed, 0.0);
   fVL -= fOver; fVR -= fOver;
   fVL = Max(Min(fVL, m_fCruiseSpeed), -m_fCruiseSpeed);
   fVR = Max(Min(fVR, m_fCruiseSpeed), -m_fCruiseSpeed);
   m_pcWheels->SetLinearVelocity(fVL, fVR);
}

/****************************************/
/****************************************/

void CFootBotGrid::StopWheels() {
   m_pcWheels->SetLinearVelocity(0.0, 0.0);
}

}  /* namespace argos */
