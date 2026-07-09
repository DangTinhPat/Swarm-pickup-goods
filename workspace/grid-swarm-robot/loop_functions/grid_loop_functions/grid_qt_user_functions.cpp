/**
 * grid_qt_user_functions.cpp — Lớp vẽ debug 3D. DrawInWorld() là API
 * tương đương "PostDraw" của ARGoS3 (không tồn tại PostDraw/DrawLine
 * trong API thật; DrawRay là hàm vẽ đoạn thẳng).
 *
 * Lớp phủ che đĩa QR CHỈ mang tính hiển thị: cảm biến sàn đọc thẳng
 * GetFloorColor() nên định vị không bị ảnh hưởng bởi bất cứ thứ gì vẽ
 * ở đây.
 */

#include "grid_qt_user_functions.h"
#include "grid_loop_functions.h"
#include <controllers/footbot_grid/footbot_grid.h>

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/math/ray3.h>

#include <sstream>

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
   /* Lớp phủ che đĩa QR: vài mảng phẳng lớn (rẻ cho GPU hơn ~900 đĩa) */
   DrawBox(CVector3(0.0, 0.0, 0.003), CQuaternion(),
           CVector3(2 * HALF_SPAN + 0.7, 2 * HALF_SPAN + 0.7, 0.001),
           CColor::WHITE);

   const Real fDockMidX = RowToWorldX(DOCK_ROW_MIN + 2);
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

   /* Lưới kẻ 30x30 nổi trên lớp phủ */
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

   /* Hàng đợi hộp trên băng chuyền: xếp chồng theo chiều cao */
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

   /* Hologram màu đang yêu cầu, lơ lửng trên ô mặt kệ */
   for(const CGridLoopFunctions::SDemand& sDem : LF().GetDemands()) {
      if(!sDem.Active) continue;
      SGridCell sFace = StackFaceCell(sDem.Cell, false);
      DrawBox(CVector3(RowToWorldX(sFace.Row),
                       ColToWorldY(sFace.Col), 0.22),
              CQuaternion(),
              CVector3(0.08, 0.08, 0.08),
              BoxCColor(sDem.Color));
   }

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
