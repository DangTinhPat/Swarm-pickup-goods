#ifndef COLLECTION_QT_USER_FUNCTIONS_H
#define COLLECTION_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CCollectionLoopFunctions;

class CCollectionQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CCollectionQTUserFunctions();
   virtual ~CCollectionQTUserFunctions() {}

   /* Draws the carried ball on top of a robot */
   void Draw(CFootBotEntity& c_entity);
   /* Draws the free balls on the ground and the score at the nest */
   virtual void DrawInWorld();

private:

   CCollectionLoopFunctions& m_cLF;
};

#endif
