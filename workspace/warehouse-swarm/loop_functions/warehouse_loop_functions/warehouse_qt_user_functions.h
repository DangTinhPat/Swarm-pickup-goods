/*
 * QT visualization hooks: in-world drawing (parcels, battery bars, bin
 * queues, zone counters) + the OPERATOR CONSOLE — a HUD panel with
 * clickable buttons, keyboard shortcuts and a per-robot status board.
 *
 * The console is a human command layer like a real AMR fleet manager's
 * (e-stop / resume / recall / per-robot maintenance), NOT a central
 * planner: it only flips each controller's EOverride; the robots stay
 * fully autonomous under OP_AUTO. Headless runs never load this file,
 * so benchmarks are untouched.
 *
 * Interaction map:
 *   Buttons (click)     E-STOP / RESUME / RECALL (fleet) · STOP / CHARGE /
 *                       AUTO (selected robot) · status-board toggle
 *   Keys                X e-stop all · R resume all · H recall all ·
 *                       T status board · 1/2/3 stop/charge/auto (selected)
 *   Shift+Click robot   select it for per-robot commands
 */

#ifndef WAREHOUSE_QT_USER_FUNCTIONS_H
#define WAREHOUSE_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_warehouse/footbot_warehouse.h>

#include <QRect>
#include <string>
#include <utility>
#include <vector>

class QPainter;
class QKeyEvent;
class QMouseEvent;

using namespace argos;

class CWarehouseLoopFunctions;

class CWarehouseQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CWarehouseQTUserFunctions();
   virtual ~CWarehouseQTUserFunctions() {}

   /* ---- In-world drawing ---- */
   /* Draws the carried parcel (colored by destination) on a robot */
   void Draw(CFootBotEntity& c_entity);
   /* Draws bin contents, zone letters and the delivery counter */
   virtual void DrawInWorld();

   /* ---- Operator console ---- */
   virtual void DrawOverlay(QPainter& c_painter);
   virtual void KeyPressed(QKeyEvent* pc_event);
   virtual void MouseKeyReleased(QMouseEvent* pc_event);
   virtual void EntitySelected(CEntity& c_entity);
   virtual void EntityDeselected(CEntity& c_entity);

private:

   enum EAction {
      ACT_FLEET_ESTOP = 0,
      ACT_FLEET_RESUME,
      ACT_FLEET_RECALL,
      ACT_TOGGLE_STATUS,
      ACT_SEL_STOP,
      ACT_SEL_CHARGE,
      ACT_SEL_AUTO
   };

   /* A clickable HUD button: rebuilt every DrawOverlay, hit-tested in
    * MouseKeyReleased (a plain click reaches release without dragging) */
   struct SButton {
      QRect   Rect;
      EAction Action;
   };

   /* All robots as (id, controller), in numeric id order */
   std::vector<std::pair<std::string, CFootBotWarehouse*> > Fleet() const;
   void FleetOverride(CFootBotWarehouse::EOverride e_op);
   void RunAction(EAction e_action);
   /* Draws a button and registers its hit-box (skipped when disabled) */
   void AddButton(QPainter& c_painter, const QRect& c_rect,
                  const QString& c_label, const QColor& c_border,
                  EAction e_action, bool b_enabled);

   CWarehouseLoopFunctions& m_cLF;

   /* Console state */
   CFootBotWarehouse* m_pcSelected;      /* robot picked via shift+click */
   std::string        m_strSelectedId;
   bool               m_bShowStatus;     /* per-robot status board on/off */
   std::vector<SButton> m_vecButtons;
};

#endif
