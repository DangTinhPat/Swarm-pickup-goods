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
   /* --- 1. Kẻ lưới 20x20 nổi 1 cm trên sàn cho nhìn rõ trong 3D --- */
   const CColor cGridCol(110, 110, 110);
   for(SInt32 i = 0; i <= GRID_COLS; ++i) {
      Real fX = -HALF_W + i * CELL_SIZE;
      DrawRay(CRay3(CVector3(fX, -HALF_H, 0.01), CVector3(fX, HALF_H, 0.01)),
              cGridCol, 1.0);
   }
   for(SInt32 i = 0; i <= GRID_ROWS; ++i) {
      Real fY = -HALF_H + i * CELL_SIZE;
      DrawRay(CRay3(CVector3(-HALF_W, fY, 0.01), CVector3(HALF_W, fY, 0.01)),
              cGridCol, 1.0);
   }

   /* --- 2. Hộp hàng vật lý đang nằm chờ trên miệng băng chuyền --- */
   for(const CGridLoopFunctions::SConveyor& sConv : LF().GetConveyors()) {
      if(!sConv.HasBox) continue;
      DrawBox(CVector3(ColToWorldX(sConv.Cell.Col),
                       RowToWorldY(sConv.Cell.Row), 0.10),
              CQuaternion(),
              CVector3(0.22, 0.22, 0.20),
              BoxCColor(sConv.Color));
   }

   /* --- 3. "Hologram" màu đang yêu cầu lơ lửng trên ô ngăn xếp --- */
   for(const CGridLoopFunctions::SDemand& sDem : LF().GetDemands()) {
      if(!sDem.Active) continue;
      DrawBox(CVector3(ColToWorldX(sDem.Cell.Col),
                       RowToWorldY(sDem.Cell.Row), 0.35),
              CQuaternion(),
              CVector3(0.16, 0.16, 0.16),
              BoxCColor(sDem.Color));
      DrawText(CVector3(ColToWorldX(sDem.Cell.Col) - 0.15,
                        RowToWorldY(sDem.Cell.Row) - 0.30, 0.05),
               std::to_string(sDem.Fulfilled));
   }

   /* --- 4. Bảng tổng kết ở mép dưới nhà kho --- */
   std::ostringstream ossTotal;
   ossTotal << "DA GIAO: " << LF().GetDeliveredTotal();
   DrawText(CVector3(-1.2, -HALF_H + 0.15, 0.4), ossTotal.str());
}

/****************************************/
/****************************************/

void CGridQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotGrid* pcCtrl = dynamic_cast<CFootBotGrid*>(
      &c_entity.GetControllableEntity().GetController());
   if(pcCtrl == nullptr) return;

   /* Hộp hàng trên lưng khi đang chở */
   if(pcCtrl->IsCarrying()) {
      DrawBox(CVector3(0.0, 0.0, 0.32),
              CQuaternion(),
              CVector3(0.14, 0.14, 0.14),
              BoxCColor(pcCtrl->GetCarriedColor()));
   }

   /* Nhãn nổi: id | %pin | trạng thái (+ dấu * khi đang nhường đường) */
   std::ostringstream oss;
   oss << "fb" << (int)pcCtrl->GetRobotId()
       << " " << (int)(pcCtrl->GetBatteryFrac() * 100.0) << "%"
       << " " << pcCtrl->GetStateName();
   if(pcCtrl->IsYielding()) oss << " *";
   Real fBatt = pcCtrl->GetBatteryFrac();
   CColor cCol = (fBatt < 0.20) ? CColor::RED
               : (fBatt < 0.70) ? CColor(200, 120, 0)
                                : CColor(0, 120, 0);
   DrawText(CVector3(-0.15, 0.0, 0.46), oss.str(), cCol);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CGridQTUserFunctions, "grid_qt_user_functions")

}  /* namespace argos */
