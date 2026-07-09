/**
 * footbot_grid_nav.cpp — ĐỊNH VỊ + TÌM ĐƯỜNG + BÁM TÂM Ô
 *
 * 1. UpdateLocalization : dead-reckoning encoder, chốt lại bằng "QR sàn"
 *    (đĩa đen r=0.02 m ở hồng tâm ô) khi robot lọt vào đúng bán kính
 *    0.02 m quanh tâm — khớp chính xác yêu cầu đề bài.
 * 2. PlanPath            : A* 4 hướng trên lưới 30x30, CẤM TUYỆT ĐỐI ô
 *    CELL_OBSTACLE (thân ngăn xếp — vật cản vật lý thật ngoài .argos).
 * 3. ApplySteering        : bộ điều khiển tỷ lệ; V_Base = 10 cm/s chạy
 *    thẳng, hạ còn 3 cm/s khi bẻ lái > 55° (rẽ 90° tại tâm ô) để chống
 *    trượt bánh trong ô chỉ rộng 0.2 m.
 */

#include "footbot_grid.h"
#include <loop_functions/grid_loop_functions/grid_loop_functions.h>

#include <argos3/core/utility/logging/argos_log.h>

#include <algorithm>
#include <cmath>

namespace argos {

/****************************************/
/****************************************/

void CFootBotGrid::UpdateLocalization() {

   /* ------------------------------------------------------------------
    * A. DEAD-RECKONING (odometry bánh xe — nguồn duy nhất giữa 2 lần
    *    chốt QR; tích phân trung điểm cho sai số nhỏ hơn Euler thuần):
    *      ds  = (dL + dR) / 2          dth = (dR - dL) / L
    *      x  += ds * cos(theta + dth/2)   y += ds * sin(theta + dth/2)
    * ------------------------------------------------------------------ */
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

   /* ------------------------------------------------------------------
    * B. CHỐT VỊ TRÍ BẰNG "MÃ QR SÀN" — xóa hoàn toàn odometry drift.
    * Đĩa đen bán kính 0.02 m ở hồng tâm mọi ô (grid_floor_render.cpp).
    * Cảm biến sàn đọc độ xám < 0.08 chỉ có thể là đĩa đen. Khi (1) thấy
    * đĩa đen, (2) ước lượng vị trí đang trong đúng bán kính 0.02 m
    * quanh tâm ô gần nhất, (3) chưa chốt trong ô này -> "giải mã QR":
    * lấy tư thế tuyệt đối từ positioning, GHI ĐÈ ước lượng, đưa sai số
    * tích lũy VỀ 0. ------------------------------------------------- */
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
      /* Tick đầu: robot đứng ngay trên dock (có QR) -> chốt luôn */
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
      /* Lưới dày 0.2 m mà trôi 0.6 m (3 ô) chưa gặp QR nào = đã lạc
       * khỏi làn -> tái định vị cứu hộ, đếm sự kiện để lộ lỗi tinh
       * chỉnh nếu số này khác 0 trong báo cáo cuối. */
      SnapFromMarker();
      ++m_unRelocCount;
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level) {
   /* A* 4 hướng trên lưới 30x30. CELL_OBSTACLE (thân ngăn xếp) CẤM
    * TUYỆT ĐỐI dù có phải ô đích hay không — đích hợp lệ chỉ có thể là
    * CELL_FREE / CELL_DOCK / CELL_CONVEYOR (SetGoal() không bao giờ
    * trỏ vào CELL_OBSTACLE, nhưng vẫn chặn cứng ở đây để phòng thủ).
    *
    * Mức né động (leo thang khi bị chặn lâu — chỉ dùng khi DẠT LÀN
    * CỤC BỘ 3 BƯỚC đã thất bại cả hai phía, xem footbot_grid_traffic.cpp):
    *   0 - chỉ né vật cản tĩnh (tin vào đặt chỗ + dạt làn)
    *   1 - né thêm ô đang bị robot khác ĐẶT CHỖ trong bán kính 6 ô
    *   2 - né thêm ô hiện tại + ô dự định của mọi hàng xóm RAB       */

   constexpr SInt32 N = GRID_ROWS * GRID_COLS;
   auto Idx = [](const SGridCell& c) { return c.Row * GRID_COLS + c.Col; };

   std::array<bool, N> arrAvoid{};
   if(un_avoid_level >= 1) {
      for(SInt32 r = 0; r < GRID_ROWS; ++r)
         for(SInt32 c = 0; c < GRID_COLS; ++c) {
            SGridCell sCell(r, c);
            if(sCell.ManhattanTo(m_sCurCell) > 6) continue;
            SInt32 nOwner = LF().CellReserver(sCell);
            if(nOwner >= 0 && nOwner != (SInt32)m_unId)
               arrAvoid[Idx(sCell)] = true;
         }
   }
   if(un_avoid_level >= 2) {
      for(const SNeighbor& sN : m_vecNeighbors) {
         if(sN.Cur.IsValid())  arrAvoid[Idx(sN.Cur)]  = true;
         if(sN.Next.IsValid()) arrAvoid[Idx(sN.Next)] = true;
      }
   }
   arrAvoid[Idx(m_sCurCell)] = false;

   auto Passable = [&](const SGridCell& c) {
      if(!c.IsValid()) return false;
      if(CellTypeOf(c) == CELL_OBSTACLE) return false;   /* cấm tuyệt đối */
      if(arrAvoid[Idx(c)] && !(c == s_goal)) return false;
      EGridCellType e = CellTypeOf(c);
      return e == CELL_FREE || c == s_goal;
   };

   std::array<Real,   N> arrG;      arrG.fill(1.0e9);
   std::array<SInt32, N> arrParent; arrParent.fill(-1);
   std::array<SInt8,  N> arrDir;    arrDir.fill(-1);
   std::array<bool,   N> arrClosed{};
   const SInt32 DR[4] = { 0, 1, 0, -1 };
   const SInt32 DC[4] = { 1, 0, -1, 0 };

   SInt32 nStart = Idx(m_sCurCell), nGoal = Idx(s_goal);
   if(m_sCurCell == s_goal) {
      m_vecPath.clear(); m_unPathIdx = 0; m_sPathGoal = s_goal;
      return true;
   }
   arrG[nStart] = 0.0;
   std::vector<SInt32> vecOpen{ nStart };

   while(!vecOpen.empty()) {
      size_t unBest = 0;
      Real fBestF = 1.0e18;
      for(size_t i = 0; i < vecOpen.size(); ++i) {
         SInt32 n = vecOpen[i];
         SGridCell c(n / GRID_COLS, n % GRID_COLS);
         Real fF = arrG[n] + c.ManhattanTo(s_goal);
         if(fF < fBestF) { fBestF = fF; unBest = i; }
      }
      SInt32 nCur = vecOpen[unBest];
      vecOpen[unBest] = vecOpen.back();
      vecOpen.pop_back();
      if(nCur == nGoal) break;
      if(arrClosed[nCur]) continue;
      arrClosed[nCur] = true;

      SGridCell sCur(nCur / GRID_COLS, nCur % GRID_COLS);
      for(SInt32 d = 0; d < 4; ++d) {
         SGridCell sNb(sCur.Row + DR[d], sCur.Col + DC[d]);
         if(!Passable(sNb)) continue;
         SInt32 nNb = Idx(sNb);
         if(arrClosed[nNb]) continue;
         Real fTurn = (arrDir[nCur] >= 0 && arrDir[nCur] != d) ? 0.35 : 0.0;
         Real fNewG = arrG[nCur] + 1.0 + fTurn;
         if(fNewG < arrG[nNb]) {
            arrG[nNb]      = fNewG;
            arrParent[nNb] = nCur;
            arrDir[nNb]    = d;
            vecOpen.push_back(nNb);
         }
      }
   }

   if(arrParent[nGoal] < 0) return false;

   std::vector<SGridCell> vecRev;
   for(SInt32 n = nGoal; n != nStart; n = arrParent[n])
      vecRev.emplace_back(n / GRID_COLS, n % GRID_COLS);
   m_vecPath.assign(vecRev.rbegin(), vecRev.rend());
   m_unPathIdx = 0;
   m_sPathGoal = s_goal;
   return true;
}

/****************************************/
/****************************************/

void CFootBotGrid::ApplySteering(const CVector2& c_target, bool b_final) {
   /* ------------------------------------------------------------------
    * BỘ ĐIỀU KHIỂN BẺ LÁI TỶ LỆ (P) BÁM TÂM Ô — hệ vi sai 2 bánh
    *
    *   e = tâm_ô_kế_tiếp - vị_trí_ước_lượng ; psi = atan2(e.y, e.x)
    *   err = chuẩn_hóa(psi - theta) ∈ (-pi, pi]
    *
    * 1) |err| > 55° (rẽ 90° tại tâm ô, đổi làn/hành lang/dạt làn):
    *    XOAY TẠI CHỖ với V_Base HẠ CÒN 3 cm/s (m_fPivotSpeed) — bắt
    *    buộc theo đề bài để chống trượt bánh vật lý trong ô 0.2 m.
    *
    * 2) Ngược lại: chạy tới với luật tỷ lệ, V_Base = 10 cm/s
    *    (m_fCruiseSpeed):
    *      v = V_Base * cos(err) ; omega = Kp * err
    *      v_trái = v - omega*L/2 ; v_phải = v + omega*L/2
    *
    * 3) b_final (waypoint là ô đích cuối): phanh tỷ lệ theo khoảng
    *    cách để dừng êm đúng hồng tâm (goal_tol = 0.02 m).
    * ------------------------------------------------------------------ */
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
   Real fDiff  = fOmega * (m_fAxisM * 0.5) * 100.0;
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
