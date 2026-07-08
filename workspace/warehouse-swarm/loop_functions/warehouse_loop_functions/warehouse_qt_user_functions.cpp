#include "warehouse_qt_user_functions.h"
#include "warehouse_loop_functions.h"
#include <controllers/footbot_warehouse/footbot_warehouse.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

/****************************************/
/****************************************/

static const Real PARCEL_RADIUS = 0.06;

CWarehouseQTUserFunctions::CWarehouseQTUserFunctions() :
   m_cLF(dynamic_cast<CWarehouseLoopFunctions&>(
            CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CWarehouseQTUserFunctions, CFootBotEntity>(
      &CWarehouseQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotWarehouse& cController =
      dynamic_cast<CFootBotWarehouse&>(c_entity.GetControllableEntity().GetController());
   if(cController.IsCarrying()) {
      DrawBox(
         CVector3(0.0, 0.0, 0.32),
         CQuaternion(),
         CVector3(0.12, 0.12, 0.1),
         CWarehouseLoopFunctions::AddressColor(cController.GetCarriedAddress()));
   }

   /* Battery HUD: a bar floating over the robot, kept world-aligned by
    * counter-rotating the robot's own yaw so it never spins with it */
   CBatteryEquippedEntity& cBattery = c_entity.GetBatterySensorEquippedEntity();
   Real fFrac = cBattery.GetAvailableCharge() / cBattery.GetFullCharge();
   CRadians cYaw, cPitch, cRoll;
   c_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation
      .ToEulerAngles(cYaw, cPitch, cRoll);
   CQuaternion cCounter(-cYaw, CVector3::Z);
   /* frame/background */
   DrawBox(CVector3(0.0, 0.0, 0.46), cCounter,
           CVector3(0.22, 0.05, 0.015), CColor::GRAY20);
   /* fill, anchored to the bar's left edge (in world frame) */
   CColor cFillColor;
   if(cController.IsDead())          cFillColor = CColor::BLACK;
   else if(cController.IsCharging()) cFillColor = CColor::BLUE;
   else if(fFrac > 0.5)              cFillColor = CColor::GREEN;
   else if(fFrac > 0.25)             cFillColor = CColor::ORANGE;
   else                              cFillColor = CColor::RED;
   CVector2 cOffset(-0.5 * (1.0 - fFrac) * 0.20, 0.0);
   cOffset.Rotate(-cYaw);
   DrawBox(CVector3(cOffset.GetX(), cOffset.GetY(), 0.475), cCounter,
           CVector3(Max<Real>(0.20 * fFrac, 0.005), 0.04, 0.015),
           cFillColor);
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::DrawInWorld() {
   /* Bin contents: one small box per queued parcel, in a line in front
    * of each belt, colored by destination */
   for(UInt32 b = 0; b < CWarehouseLoopFunctions::NUM_BELTS; ++b) {
      const std::deque<UInt8>& cQueue = m_cLF.GetBeltQueue(b);
      const CVector2& cPickup = m_cLF.GetBeltPickup(b);
      for(size_t i = 0; i < cQueue.size(); ++i) {
         DrawBox(
            CVector3(cPickup.GetX() + 0.35,
                     cPickup.GetY() - 0.30 + i * 0.13,
                     0.05),
            CQuaternion(),
            CVector3(0.1, 0.1, 0.1),
            CWarehouseLoopFunctions::AddressColor(cQueue[i]));
      }
   }
   /* Per-zone counter of successfully delivered parcels */
   const UInt32* punPerAddr = m_cLF.GetDeliveredPerAddr();
   for(UInt32 a = 0; a < CWarehouseLoopFunctions::NUM_ADDRS; ++a) {
      const CVector2& cPos = m_cLF.GetAddrPos(a);
      DrawText(CVector3(cPos.GetX(), cPos.GetY(), 0.25),
               std::to_string(punPerAddr[a]),
               CColor::BLACK);
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWarehouseQTUserFunctions, "warehouse_qt_user_functions")
