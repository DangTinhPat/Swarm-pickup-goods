#include "warehouse_qt_user_functions.h"
#include "warehouse_loop_functions.h"
#include <controllers/footbot_warehouse/footbot_warehouse.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

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
      DrawText(CVector3(cPickup.GetX() + 0.1, cPickup.GetY() + 0.45, 0.3),
               std::string("Belt ") + std::to_string(b) +
               std::string(": ") + std::to_string(cQueue.size()),
               CColor::BLACK);
   }
   /* Address zone letters */
   for(UInt32 a = 0; a < CWarehouseLoopFunctions::NUM_ADDRS; ++a) {
      const CVector2& cPos = m_cLF.GetAddrPos(a);
      DrawText(CVector3(cPos.GetX(), cPos.GetY(), 0.25),
               std::string(1, (char)('A' + a)),
               CColor::BLACK);
   }
   /* Delivery counter above the depot */
   DrawText(CVector3(m_cLF.GetDockCenter().GetX(), m_cLF.GetDockCenter().GetY(), 0.5),
            std::string("Delivered: ") + std::to_string(m_cLF.GetDelivered()),
            CColor::BLACK);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWarehouseQTUserFunctions, "warehouse_qt_user_functions")
