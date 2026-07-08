#ifndef WAREHOUSE_QT_USER_FUNCTIONS_H
#define WAREHOUSE_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CWarehouseLoopFunctions;

class CWarehouseQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CWarehouseQTUserFunctions();
   virtual ~CWarehouseQTUserFunctions() {}

   /* Draws the carried parcel (colored by destination) on a robot */
   void Draw(CFootBotEntity& c_entity);
   /* Draws bin contents, zone letters and the delivery counter */
   virtual void DrawInWorld();

private:

   CWarehouseLoopFunctions& m_cLF;
};

#endif
