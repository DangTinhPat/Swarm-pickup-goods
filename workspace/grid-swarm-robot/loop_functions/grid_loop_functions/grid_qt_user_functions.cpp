#include "grid_qt_user_functions.h"
#include "grid_loop_functions.h"
#include <controllers/footbot_grid/footbot_grid.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/math/ray3.h>

#include <sstream>
#include <iomanip>

namespace argos {

/****************************************/
/****************************************/

CGridQTUserFunctions::CGridQTUserFunctions() {
   RegisterUserFunction<CGridQTUserFunctions, CFootBotEntity>(
      &CGridQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

CGridLoopFunctions& CGridQTUserFunctions::LF() {
   if(m_pcLF == nullptr) {
      m_pcLF = &dynamic_cast<CGridLoopFunctions&>(
         CSimulator::GetInstance().GetLoopFunctions());
   }
   return *m_pcLF;
}

/****************************************/
/****************************************/

void CGridQTUserFunctions::DrawInWorld() {
   /* --- 0. CHE ĐĨA QR ĐEN — CHỈ VỀ MẶT HIỂN THỊ ------------------------
    * Đĩa đen r=0.085 m trong GetFloorColor() vẫn giữ NGUYÊN (bắt buộc
    * để cảm biến sàn "thấy" được — xem grid_floor_render.cpp); cảm
    * biến gọi thẳng CFloorEntity::GetColorAtPoint() -> GetFloorColor(),
    * hoàn toàn tách biệt với những gì vẽ ở đây trong DrawInWorld() (một
    * lớp render 3D riêng, không phải nguồn dữ liệu cảm biến) — nên phủ
    * một lớp mờ CÙNG MÀU NỀN lên trên (z thấp hơn lưới) để mắt người
    * không thấy chấm đen nữa mà định vị vẫn hoạt động y hệt.
    * Dùng vài mảng phẳng lớn (nền trắng + 2 dải dock + từng ô băng
    * chuyền) thay vì phủ từng ô một (~2400 ô) để rẻ hơn nhiều cho GPU,
    * cộng thêm các đốm nhỏ phủ đúng những ô mặt kệ đang có yêu cầu màu
    * (số lượng nhỏ, đổi động theo demand). --------------------------- */
   DrawBox(CVector3(0.0, 0.0, 0.003), CQuaternion(),
           CVector3(2 * HALF_SPAN + 0.05, 2 * HALF_SPAN + 0.05, 0.001),
           CColor::WHITE);

   const Real fDockMidX = RowToWorldX(DOCK_ROW_MIN + 2);   /* tâm 5 ô dock */
   const Real fDockSpan = 5 * CELL_SIZE + 0.02;
   DrawBox(CVector3(fDockMidX, ColToWorldY(0), 0.004), CQuaternion(),
           CVector3(fDockSpan, CELL_SIZE + 0.02, 0.001), CColor(170, 218, 232));
   DrawBox(CVector3(fDockMidX, ColToWorldY(GRID_COLS - 1), 0.004), CQuaternion(),
           CVector3(fDockSpan, CELL_SIZE + 0.02, 0.001), CColor(170, 218, 232));

   for(SInt32 i = 0; i < NUM_CONVEYORS; ++i) {
      SGridCell sC = ConveyorCell(i);
      DrawBox(CVector3(RowToWorldX(sC.Row), ColToWorldY(sC.Col), 0.004),
              CQuaternion(), CVector3(CELL_SIZE + 0.02, CELL_SIZE + 0.02, 0.001),
              CColor(205, 205, 210));
   }

   for(const CGridLoopFunctions::SDemand& sDem : LF().GetDemands()) {
      if(!sDem.Active) continue;
      for(bool bFar : { false, true }) {
         SGridCell sFace = StackFaceCell(sDem.Cell, bFar);
         DrawBox(CVector3(RowToWorldX(sFace.Row), ColToWorldY(sFace.Col), 0.0045),
                 CQuaternion(), CVector3(CELL_SIZE + 0.02, CELL_SIZE + 0.02, 0.001),
                 CColor(232, 226, 208));
      }
   }

   /* --- 1. Kẻ lưới 50x50 (ô 0.2 m) nổi trên lớp che — tương đương hook
    * "PostDraw()" mà đề bài mô tả: ARGoS3 gọi API này là DrawInWorld()
    * trong CQTOpenGLUserFunctions (không tồn tại PostDraw() trong
    * ARGoS3 thật); DrawRay() là hàm vẽ đoạn thẳng (không có DrawLine()
    * trong API thật) --- */
   const CColor cGridCol(130, 130, 130);
   for(SInt32 i = 0; i <= GRID_ROWS; ++i) {
      Real fX = -HALF_SPAN + i * CELL_SIZE;
      DrawRay(CRay3(CVector3(fX, -HALF_SPAN, 0.01), CVector3(fX, HALF_SPAN, 0.01)),
              cGridCol, 0.5);
   }
   for(SInt32 i = 0; i <= GRID_COLS; ++i) {
      Real fY = -HALF_SPAN + i * CELL_SIZE;
      DrawRay(CRay3(CVector3(-HALF_SPAN, fY, 0.01), CVector3(HALF_SPAN, fY, 0.01)),
              cGridCol, 0.5);
   }

   /* --- 2. Hàng đợi tối đa 3 hộp trên mỗi băng chuyền, xếp chồng theo
    * chiều cao cho gọn trong ô 0.2 m (không đủ chỗ dàn hàng ngang) --- */
   for(const CGridLoopFunctions::SConveyor& sConv : LF().GetConveyors()) {
      for(size_t k = 0; k < sConv.Queue.size(); ++k) {
         DrawBox(CVector3(RowToWorldX(sConv.Cell.Row),
                          ColToWorldY(sConv.Cell.Col),
                          0.06 + k * 0.09),
                 CQuaternion(),
                 CVector3(0.10, 0.10, 0.08),
                 BoxCColor(sConv.Queue[k]));
      }
   }

   /* --- 3. "Hologram" màu đang yêu cầu lơ lửng trên ô mặt kệ --- */
   for(const CGridLoopFunctions::SDemand& sDem : LF().GetDemands()) {
      if(!sDem.Active) continue;
      SGridCell sFace = StackFaceCell(sDem.Cell, false);
      DrawBox(CVector3(RowToWorldX(sFace.Row),
                       ColToWorldY(sFace.Col), 0.22),
              CQuaternion(),
              CVector3(0.08, 0.08, 0.08),
              BoxCColor(sDem.Color));
   }

   /* --- 4. Bảng tổng kết ở mép Tây nhà kho --- */
   std::ostringstream ossTotal;
   ossTotal << "DA GIAO: " << LF().GetDeliveredTotal();
   DrawText(CVector3(-HALF_SPAN + 0.3, -HALF_SPAN + 0.3, 0.4), ossTotal.str());
}

/****************************************/
/****************************************/

void CGridQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotGrid* pcCtrl = dynamic_cast<CFootBotGrid*>(
      &c_entity.GetControllableEntity().GetController());
   if(pcCtrl == nullptr) return;

   if(pcCtrl->IsCarrying()) {
      DrawBox(CVector3(0.0, 0.0, 0.16),
              CQuaternion(),
              CVector3(0.07, 0.07, 0.07),
              BoxCColor(pcCtrl->GetCarriedColor()));
   }

   std::ostringstream oss;
   oss << "fb" << (int)pcCtrl->GetRobotId()
       << " " << (int)(pcCtrl->GetBatteryFrac() * 100.0) << "%"
       << " " << pcCtrl->GetStateName();
   if(pcCtrl->IsInTraffic()) oss << " *";
   Real fBatt = pcCtrl->GetBatteryFrac();
   CColor cCol = (fBatt < 0.20) ? CColor::RED
               : (fBatt < 0.70) ? CColor(200, 120, 0)
                                : CColor(0, 120, 0);
   DrawText(CVector3(-0.08, 0.0, 0.22), oss.str(), cCol);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CGridQTUserFunctions, "grid_qt_user_functions")

}  /* namespace argos */
