/**
 * footbot_grid_localization.cpp — Định vị: dead-reckoning encoder giữa
 * hai mốc, ghi đè tuyệt đối (x, y, θ) mỗi lần "đọc được QR sàn" — sai
 * số trượt bánh tích lũy về 0 tại mỗi hồng tâm ô.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <cmath>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::UpdateLocalization() {

   /* Dead-reckoning tích phân trung điểm:
    *   ds = (dL+dR)/2 ; dθ = (dR-dL)/L ; tiến theo hướng θ+dθ/2 */
   if(m_bPoseInit) {
      const CCI_DifferentialSteeringSensor::SReading& sOdo =
         m_pcWheelsSens->GetReading();
      Real fDL   = sOdo.CoveredDistanceLeftWheel  * 0.01;   /* cm -> m */
      Real fDR   = sOdo.CoveredDistanceRightWheel * 0.01;
      Real fAxis = sOdo.WheelAxisLength           * 0.01;
      if(fAxis > 1e-4) m_fAxisM = fAxis;

      Real fDs  = 0.5 * (fDL + fDR);
      Real fDth = (fDR - fDL) / m_fAxisM;
      Real fMid = m_cEstYaw.GetValue() + 0.5 * fDth;
      m_cEstPos += CVector2(fDs * std::cos(fMid), fDs * std::sin(fMid));
      m_cEstYaw = CRadians(m_cEstYaw.GetValue() + fDth);
      m_cEstYaw.SignedNormalize();
      m_fOdoSinceFix += std::fabs(fDs);
   }

   /* Đĩa QR là vùng DUY NHẤT trên sàn có độ xám < 0.08 (xem
    * grid_floor_render.cpp). Chốt khi: thấy đĩa + ước lượng nằm trong
    * bán kính tin cậy quanh tâm ô + chưa chốt trong chính ô này. */
   const CCI_FootBotMotorGroundSensor::TReadings& tGround =
      m_pcGround->GetReadings();
   Real fDarkest = 1.0;
   for(size_t i = 0; i < tGround.size(); ++i)
      fDarkest = Min(fDarkest, tGround[i].Value);

   auto SnapFromMarker = [this]() {
      const CCI_PositioningSensor::SReading& sPos = m_pcPosSens->GetReading();
      m_cEstPos.Set(sPos.Position.GetX(), sPos.Position.GetY());
      CRadians cZ, cY, cX;
      sPos.Orientation.ToEulerAngles(cZ, cY, cX);
      m_cEstYaw      = cZ;
      m_fOdoSinceFix = 0.0;
   };

   if(!m_bPoseInit) {
      /* Tick đầu: robot spawn ngay trên dock (có QR) -> chốt luôn */
      SnapFromMarker();
      m_bPoseInit = true;
      m_sCurCell  = SGridCell(WorldXToRow(m_cEstPos.GetX()),
                              WorldYToCol(m_cEstPos.GetY()));
      m_sPrevCell = m_sCurCell;
      m_sLastSnapCell = m_sCurCell;
      return;
   }

   SGridCell sNearest(WorldXToRow(m_cEstPos.GetX()),
                      WorldYToCol(m_cEstPos.GetY()));
   CVector2 cCenter(RowToWorldX(sNearest.Row), ColToWorldY(sNearest.Col));
   Real fDistCenter = (m_cEstPos - cCenter).Length();

   if(fDarkest < 0.08 && fDistCenter <= m_fQrSnapRadius && sNearest != m_sLastSnapCell) {
      SnapFromMarker();
      m_sLastSnapCell = sNearest;
      ++m_unSnapCount;
   }
   else if(m_fOdoSinceFix > 0.6) {
      /* Trôi 3 ô chưa gặp QR = lạc làn -> tái định vị cứu hộ (đếm để
       * lộ lỗi tinh chỉnh nếu số này khác 0 trong báo cáo cuối). */
      SnapFromMarker();
      ++m_unRelocCount;
   }
}

}  /* namespace argos */
