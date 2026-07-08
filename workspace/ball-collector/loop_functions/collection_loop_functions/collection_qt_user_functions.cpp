#include "collection_qt_user_functions.h"
#include "collection_loop_functions.h"
#include <controllers/footbot_collector/footbot_collector.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

/****************************************/
/****************************************/

static const Real BALL_RADIUS = 0.06;

CCollectionQTUserFunctions::CCollectionQTUserFunctions() :
   m_cLF(dynamic_cast<CCollectionLoopFunctions&>(
            CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CCollectionQTUserFunctions, CFootBotEntity>(
      &CCollectionQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CCollectionQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   CFootBotCollector& cController =
      dynamic_cast<CFootBotCollector&>(c_entity.GetControllableEntity().GetController());
   if(cController.IsCarrying()) {
      /* The carried ball rides on top of the robot */
      DrawCylinder(
         CVector3(0.0, 0.0, 0.3),
         CQuaternion(),
         BALL_RADIUS,
         2.0 * BALL_RADIUS,
         CColor::ORANGE);
   }
}

/****************************************/
/****************************************/

void CCollectionQTUserFunctions::DrawInWorld() {
   /* Free balls on the ground */
   const std::vector<CVector2>& cBalls = m_cLF.GetBalls();
   for(size_t i = 0; i < cBalls.size(); ++i) {
      DrawCylinder(
         CVector3(cBalls[i].GetX(), cBalls[i].GetY(), 0.0),
         CQuaternion(),
         BALL_RADIUS,
         2.0 * BALL_RADIUS,
         CColor::ORANGE);
   }
   /* Score floating above the nest */
   DrawText(
      CVector3(m_cLF.GetNestPos().GetX(), m_cLF.GetNestPos().GetY(), 0.4),
      std::string("Balls: ") + std::to_string(m_cLF.GetScore()),
      CColor::BLACK);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CCollectionQTUserFunctions, "collection_qt_user_functions")
