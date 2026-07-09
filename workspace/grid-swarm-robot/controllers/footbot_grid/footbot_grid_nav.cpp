/**
 * footbot_grid_nav.cpp — ĐỊNH VỊ + TÌM ĐƯỜNG + BÁM TÂM Ô
 *
 * 1. UpdateLocalization : dead-reckoning từ encoder 2 bánh, và "đọc mã
 *    QR sàn" (đĩa đen ở hồng tâm ô, bắt bằng cảm biến sàn) để XÓA SẠCH
 *    sai số trôi tích lũy về 0 mỗi lần ngang qua tâm ô.
 * 2. PlanPath           : A* 4 hướng trên ma trận 20x20, phạt cua rẽ để
 *    robot ưu tiên chạy thẳng theo làn.
 * 3. ApplySteering      : bộ điều khiển TỶ LỆ cho hệ vi sai, kéo robot
 *    về đúng hồng tâm ô kế tiếp.
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
    * A. DEAD-RECKONING (odometry bánh xe — NGUỒN DUY NHẤT khi đang chạy
    *    giữa hai vạch QR, và là nơi sai số trượt bánh tích lũy dần):
    *
    *      dL, dR : quãng đường 2 bánh lăn được trong tick [m]
    *      ds  = (dL + dR) / 2          — tịnh tiến của tâm robot
    *      dth = (dR - dL) / L          — góc xoay (L = khoảng cách 2 bánh)
    *
    *    Tích phân trung điểm (midpoint) cho sai số nhỏ hơn Euler thuần:
    *      x   += ds * cos(theta + dth/2)
    *      y   += ds * sin(theta + dth/2)
    *      theta = chuẩn hóa(theta + dth)
    * ------------------------------------------------------------------ */
   if(m_bPoseInit) {
      const CCI_DifferentialSteeringSensor::SReading& sOdo =
         m_pcWheelsSens->GetReading();
      /* ARGoS trả các quãng đường bằng cm -> đổi sang m */
      Real fDL   = sOdo.CoveredDistanceLeftWheel  * 0.01;
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
    * B. CHỐT VỊ TRÍ BẰNG "MÃ QR SÀN" (xóa hoàn toàn odometry drift):
    *
    * Mỗi hồng tâm ô có một đĩa ĐEN bán kính 10 cm do Loop Functions vẽ
    * lên sàn. Bốn cảm biến sàn của foot-bot đọc độ xám: giá trị < 0.08
    * chỉ có thể là đĩa đen (mọi màu khác trên sàn được chọn sáng hơn
    * hẳn). Khi (1) thấy đĩa đen, (2) ước lượng đang ở gần tâm ô nào đó
    * (< 0.35 m — cổng an toàn chống nhiễu), và (3) chưa chốt trong ô
    * này, robot "giải mã QR": lấy tư thế tuyệt đối (x, y, góc) từ cảm
    * biến positioning — mô phỏng việc camera gầm đọc ra tư thế in trong
    * mã — và GHI ĐÈ lên ước lượng, đưa sai số tích lũy VỀ 0. Nhờ vậy
    * robot luôn chạy thẳng tắp theo làn lưới vuông vắn.
    * ------------------------------------------------------------------ */
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
      /* Tick đầu tiên: robot đứng ngay trên dock (có QR) -> chốt luôn */
      SnapFromMarker();
      m_bPoseInit = true;
      m_sCurCell  = SGridCell(WorldYToRow(m_cEstPos.GetY()),
                              WorldXToCol(m_cEstPos.GetX()));
      m_sPrevCell = m_sCurCell;
      m_sLastSnapCell = m_sCurCell;
      return;
   }

   SGridCell sNearest(WorldYToRow(m_cEstPos.GetY()),
                      WorldXToCol(m_cEstPos.GetX()));
   CVector2 cCenter(ColToWorldX(sNearest.Col), RowToWorldY(sNearest.Row));
   Real fDistCenter = (m_cEstPos - cCenter).Length();

   if(fDarkest < 0.08 && fDistCenter < 0.35 && sNearest != m_sLastSnapCell) {
      SnapFromMarker();
      m_sLastSnapCell = sNearest;
      ++m_unSnapCount;
   }
   else if(m_fOdoSinceFix > 3.0) {
      /* Lưới dày 1 m mà chạy 3 m chưa gặp QR nào = đã lạc khỏi làn.
       * Robot thật sẽ chạy thủ tục quét tìm mã gần nhất; trong mô
       * phỏng ta tái định vị thẳng và đếm sự kiện để lộ ra khi tham
       * số nhiễu/điều khiển chỉnh sai. */
      SnapFromMarker();
      ++m_unRelocCount;
   }
}

/****************************************/
/****************************************/

bool CFootBotGrid::PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level) {
   /* A* 4 hướng trên ma trận 20x20.
    *
    * Quy tắc đi lại: chỉ ô CELL_FREE là công cộng; các ô dịch vụ
    * (dock / băng chuyền / ngăn xếp) là "ngõ cụt riêng" — CHỈ robot có
    * đích đúng ô đó mới được bước vào, nên luồng giao thông không bao
    * giờ xuyên qua trạm của robot khác.
    *
    * Mức né động (leo thang khi bị chặn lâu):
    *   0 - chỉ tránh vật cản tĩnh (tin vào đặt chỗ + nhường đường)
    *   1 - né thêm các ô đang bị robot khác ĐẶT CHỖ trong bán kính 8 ô
    *   2 - né thêm ô hiện tại + ô dự định của mọi hàng xóm RAB
    * -> nhờ vậy robot kẹt ở hành lang này sẽ tự vòng sang hành lang kia. */

   constexpr SInt32 N = GRID_ROWS * GRID_COLS;
   auto Idx  = [](const SGridCell& c) { return c.Row * GRID_COLS + c.Col; };

   std::array<bool, N> arrAvoid{};
   if(un_avoid_level >= 1) {
      for(SInt32 r = 0; r < GRID_ROWS; ++r)
         for(SInt32 c = 0; c < GRID_COLS; ++c) {
            SGridCell sCell(r, c);
            if(sCell.ManhattanTo(m_sCurCell) > 8) continue;
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
   arrAvoid[Idx(m_sCurCell)] = false;   /* không bao giờ tự khóa mình */

   auto Passable = [&](const SGridCell& c) {
      if(!c.IsValid()) return false;
      if(arrAvoid[Idx(c)] && !(c == s_goal)) return false;
      EGridCellType e = CellTypeOf(c);
      return e == CELL_FREE || c == s_goal;
   };

   /* Bảng trạng thái A* cỡ nhỏ — 400 ô nên không cần heap phức tạp */
   std::array<Real,   N> arrG;      arrG.fill(1.0e9);
   std::array<SInt32, N> arrParent; arrParent.fill(-1);
   std::array<SInt8,  N> arrDir;    arrDir.fill(-1);
   std::array<bool,   N> arrClosed{};
   /* 4 hướng: Đông, Bắc, Tây, Nam (dRow, dCol) */
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
      /* lấy nút có f = g + h nhỏ nhất */
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
         /* phạt 0.35 cho mỗi lần đổi hướng -> chuộng đường thẳng theo làn */
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

   if(arrParent[nGoal] < 0) return false;   /* không có đường */

   /* Lần ngược cha -> con, bỏ ô xuất phát */
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
    * BỘ ĐIỀU KHIỂN BẺ LÁI TỶ LỆ (P) BÁM TÂM Ô — cho hệ vi sai 2 bánh
    *
    *   sai số vị trí :  e   = tâm_ô_kế_tiếp - vị_trí_ước_lượng
    *   góc mong muốn :  psi = atan2(e.y, e.x)
    *   sai số hướng  :  err = chuẩn_hóa(psi - theta)  ∈ (-pi, pi]
    *
    * 1) |err| lớn hơn ngưỡng cua gắt (65°): XOAY TẠI CHỖ — hai bánh
    *    quay ngược chiều nhau (+/- turn_speed). Robot chỉ rẽ vuông góc
    *    ngay trên hồng tâm ô, giữ đội hình lưới vuông vắn.
    *
    * 2) Ngược lại: chạy tới với luật tỷ lệ
    *      v      = v_max * cos(err)          (lệch hướng -> tự giảm ga)
    *      omega  = Kp * err                  (bẻ lái tỷ lệ với sai số)
    *      v_trái  = v - omega * L/2
    *      v_phải  = v + omega * L/2
    *    Thành phần omega kéo mũi robot về thẳng hàng với tâm ô, nên mọi
    *    sai lệch ngang do trượt bánh đều bị "hút" trở lại trục làn.
    *
    * 3) b_final (ô đích cuối): thêm phanh tỷ lệ theo khoảng cách
    *    v <= 300 * dist [cm/s] để dừng êm đúng hồng tâm, không vượt lố.
    * ------------------------------------------------------------------ */
   CVector2 cDelta = c_target - m_cEstPos;
   Real     fDist  = cDelta.Length();
   CRadians cErr   = ATan2(cDelta.GetY(), cDelta.GetX()) - m_cEstYaw;
   cErr.SignedNormalize();

   if(Abs(cErr) > m_cHardTurn) {
      Real fS = (cErr.GetValue() > 0.0) ? m_fTurnSpeed : -m_fTurnSpeed;
      m_pcWheels->SetLinearVelocity(-fS, fS);
      return;
   }

   Real fV = m_fMaxSpeed * Max<Real>(0.15, Cos(cErr));
   if(b_final) fV = Min(fV, 300.0 * fDist);
   fV = Max<Real>(fV, 3.0);

   /* omega [rad/s] -> chênh lệch vận tốc bánh [cm/s] */
   Real fOmega = m_fKpHeading * cErr.GetValue();
   Real fDiff  = fOmega * (m_fAxisM * 0.5) * 100.0;
   Real fVL    = fV - fDiff;
   Real fVR    = fV + fDiff;
   /* kẹp trong khả năng động cơ, giữ nguyên hiệu số lái nếu có thể */
   Real fOver = Max(Max(fVL, fVR) - m_fMaxSpeed, 0.0);
   fVL -= fOver; fVR -= fOver;
   fVL = Max(Min(fVL, m_fMaxSpeed), -m_fMaxSpeed);
   fVR = Max(Min(fVR, m_fMaxSpeed), -m_fMaxSpeed);
   m_pcWheels->SetLinearVelocity(fVL, fVR);
}

/****************************************/
/****************************************/

void CFootBotGrid::StopWheels() {
   m_pcWheels->SetLinearVelocity(0.0, 0.0);
}

}  /* namespace argos */
