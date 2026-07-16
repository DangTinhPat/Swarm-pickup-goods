#include "warehouse_qt_user_functions.h"
#include "warehouse_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

#include <QPainter>
#include <QKeyEvent>
#include <QMouseEvent>
#include <algorithm>

/****************************************/
/****************************************/

static const Real PARCEL_RADIUS = 0.06;

/* Console palette */
static const QColor COL_PANEL_BG(12, 16, 24, 215);
static const QColor COL_TEXT(235, 238, 245);
static const QColor COL_TEXT_DIM(150, 158, 170);
static const QColor COL_ESTOP(220, 60, 50);
static const QColor COL_RESUME(70, 190, 90);
static const QColor COL_RECALL(70, 140, 240);
static const QColor COL_NEUTRAL(120, 128, 140);

static QString VN(const char* pch_utf8) { return QString::fromUtf8(pch_utf8); }

static QString ModeName(CFootBotWarehouse::EOverride e_op) {
   switch(e_op) {
      case CFootBotWarehouse::OP_STOPPED: return VN("DỪNG");
      case CFootBotWarehouse::OP_RECALL:  return VN("VỀ SẠC");
      default:                            return VN("TỰ ĐỘNG");
   }
}

static QColor ModeColor(CFootBotWarehouse::EOverride e_op) {
   switch(e_op) {
      case CFootBotWarehouse::OP_STOPPED: return COL_ESTOP;
      case CFootBotWarehouse::OP_RECALL:  return COL_RECALL;
      default:                            return COL_RESUME;
   }
}

static QString StateName(const CFootBotWarehouse& c_ctrl) {
   if(c_ctrl.IsDead()) return VN("CHẾT");
   switch(c_ctrl.GetState()) {
      case CFootBotWarehouse::STATE_TO_BELT: return VN("LẤY HÀNG");
      case CFootBotWarehouse::STATE_DELIVER: return VN("GIAO HÀNG");
      case CFootBotWarehouse::STATE_CHARGE:  return VN("SẠC");
      default:                               return VN("RẢNH");
   }
}

/****************************************/
/****************************************/

CWarehouseQTUserFunctions::CWarehouseQTUserFunctions() :
   m_cLF(dynamic_cast<CWarehouseLoopFunctions&>(
            CSimulator::GetInstance().GetLoopFunctions())),
   m_pcSelected(NULL),
   m_bShowStatus(false) {
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

std::vector<std::pair<std::string, CFootBotWarehouse*> >
CWarehouseQTUserFunctions::Fleet() const {
   std::vector<std::pair<std::string, CFootBotWarehouse*> > vecFleet;
   CSpace::TMapPerType& cFootBots =
      CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFB = *any_cast<CFootBotEntity*>(it->second);
      vecFleet.push_back(std::make_pair(
         cFB.GetId(),
         &dynamic_cast<CFootBotWarehouse&>(
            cFB.GetControllableEntity().GetController())));
   }
   /* Numeric id order: shorter ids first puts fb2 before fb10 */
   std::sort(vecFleet.begin(), vecFleet.end(),
             [](const std::pair<std::string, CFootBotWarehouse*>& a,
                const std::pair<std::string, CFootBotWarehouse*>& b) {
                if(a.first.size() != b.first.size())
                   return a.first.size() < b.first.size();
                return a.first < b.first;
             });
   return vecFleet;
}

void CWarehouseQTUserFunctions::FleetOverride(CFootBotWarehouse::EOverride e_op) {
   std::vector<std::pair<std::string, CFootBotWarehouse*> > vecFleet = Fleet();
   for(size_t i = 0; i < vecFleet.size(); ++i) {
      vecFleet[i].second->SetOverride(e_op);
   }
}

void CWarehouseQTUserFunctions::RunAction(EAction e_action) {
   switch(e_action) {
      case ACT_FLEET_ESTOP:   FleetOverride(CFootBotWarehouse::OP_STOPPED); break;
      case ACT_FLEET_RESUME:  FleetOverride(CFootBotWarehouse::OP_AUTO);    break;
      case ACT_FLEET_RECALL:  FleetOverride(CFootBotWarehouse::OP_RECALL);  break;
      case ACT_TOGGLE_STATUS: m_bShowStatus = !m_bShowStatus;               break;
      case ACT_SEL_STOP:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotWarehouse::OP_STOPPED);
         break;
      case ACT_SEL_CHARGE:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotWarehouse::OP_RECALL);
         break;
      case ACT_SEL_AUTO:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotWarehouse::OP_AUTO);
         break;
   }
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
   /* W/A/S/D/Q/E + arrows belong to the camera — everything here avoids
    * them, and unhandled keys fall through to the default handler */
   switch(pc_event->key()) {
      case Qt::Key_X: RunAction(ACT_FLEET_ESTOP);   break;
      case Qt::Key_R: RunAction(ACT_FLEET_RESUME);  break;
      case Qt::Key_H: RunAction(ACT_FLEET_RECALL);  break;
      case Qt::Key_T: RunAction(ACT_TOGGLE_STATUS); break;
      case Qt::Key_1: RunAction(ACT_SEL_STOP);      break;
      case Qt::Key_2: RunAction(ACT_SEL_CHARGE);    break;
      case Qt::Key_3: RunAction(ACT_SEL_AUTO);      break;
      default:
         CQTOpenGLUserFunctions::KeyPressed(pc_event);
         return;
   }
   GetQTOpenGLWidget().update();
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::MouseKeyReleased(QMouseEvent* pc_event) {
   /* A plain click (press + release without drag) ends up here; hit-test
    * the HUD buttons laid out by the last DrawOverlay pass */
   if(pc_event->button() == Qt::LeftButton) {
      for(size_t i = 0; i < m_vecButtons.size(); ++i) {
         if(m_vecButtons[i].Rect.contains(pc_event->pos())) {
            RunAction(m_vecButtons[i].Action);
            GetQTOpenGLWidget().update();
            return;
         }
      }
   }
   CQTOpenGLUserFunctions::MouseKeyReleased(pc_event);
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::EntitySelected(CEntity& c_entity) {
   CFootBotEntity* pcFB = dynamic_cast<CFootBotEntity*>(&c_entity);
   if(pcFB != NULL) {
      m_pcSelected = &dynamic_cast<CFootBotWarehouse&>(
         pcFB->GetControllableEntity().GetController());
      m_strSelectedId = pcFB->GetId();
   }
   else {
      m_pcSelected = NULL;
      m_strSelectedId.clear();
   }
   GetQTOpenGLWidget().update();
}

void CWarehouseQTUserFunctions::EntityDeselected(CEntity& c_entity) {
   m_pcSelected = NULL;
   m_strSelectedId.clear();
   GetQTOpenGLWidget().update();
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::AddButton(QPainter& c_painter, const QRect& c_rect,
                                          const QString& c_label, const QColor& c_border,
                                          EAction e_action, bool b_enabled) {
   QColor cBorder = b_enabled ? c_border : QColor(80, 84, 92);
   QColor cFill   = b_enabled ? QColor(c_border.red(), c_border.green(),
                                       c_border.blue(), 45)
                              : QColor(60, 64, 72, 40);
   c_painter.setPen(QPen(cBorder, 1.5));
   c_painter.setBrush(cFill);
   c_painter.drawRoundedRect(c_rect, 4, 4);
   c_painter.setPen(b_enabled ? COL_TEXT : COL_TEXT_DIM);
   c_painter.drawText(c_rect, Qt::AlignCenter, c_label);
   if(b_enabled) {
      SButton sBtn;
      sBtn.Rect = c_rect;
      sBtn.Action = e_action;
      m_vecButtons.push_back(sBtn);
   }
}

/****************************************/
/****************************************/

void CWarehouseQTUserFunctions::DrawOverlay(QPainter& c_painter) {
   m_vecButtons.clear();
   c_painter.save();
   c_painter.setRenderHint(QPainter::Antialiasing);
   c_painter.setRenderHint(QPainter::TextAntialiasing);

   QFont cTitle;  cTitle.setPointSize(10); cTitle.setBold(true);
   QFont cBody;   cBody.setPointSize(9);
   QFont cSmall;  cSmall.setPointSize(8);
   QFont cMono;   cMono.setPointSize(9);
   cMono.setStyleHint(QFont::Monospace);
   cMono.setFamily("Monospace");

   const int nX = 12;               /* panel left */
   const int nPad = 12;             /* inner padding */
   const int nW = 350;              /* panel width */
   const int nInner = nW - 2 * nPad;
   const int nBtnH = 26;
   const int nGap = 8;
   const bool bSel = (m_pcSelected != NULL);

   /* ---- Main console panel ---- */
   int nH = nPad + 18 + nGap          /* title */
          + nBtnH + nGap              /* fleet buttons */
          + nBtnH + nGap              /* status toggle */
          + 18 + 4                    /* selected label */
          + nBtnH + nGap              /* per-robot buttons */
          + 16 + 14 + nPad;           /* 2 hint lines */
   QRect cPanel(nX, 12, nW, nH);
   c_painter.setPen(Qt::NoPen);
   c_painter.setBrush(COL_PANEL_BG);
   c_painter.drawRoundedRect(cPanel, 6, 6);

   int nY = 12 + nPad;

   c_painter.setFont(cTitle);
   c_painter.setPen(COL_TEXT);
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 18), Qt::AlignLeft | Qt::AlignVCenter,
                      VN("BẢNG ĐIỀU KHIỂN VẬN HÀNH"));
   nY += 18 + nGap;

   /* Fleet-wide commands */
   c_painter.setFont(cBody);
   int nBtnW = (nInner - 2 * nGap) / 3;
   AddButton(c_painter, QRect(nX + nPad, nY, nBtnW, nBtnH),
             VN("E-STOP (X)"), COL_ESTOP, ACT_FLEET_ESTOP, true);
   AddButton(c_painter, QRect(nX + nPad + nBtnW + nGap, nY, nBtnW, nBtnH),
             VN("CHẠY LẠI (R)"), COL_RESUME, ACT_FLEET_RESUME, true);
   AddButton(c_painter, QRect(nX + nPad + 2 * (nBtnW + nGap), nY, nInner - 2 * (nBtnW + nGap), nBtnH),
             VN("VỀ SẠC (H)"), COL_RECALL, ACT_FLEET_RECALL, true);
   nY += nBtnH + nGap;

   /* Status board toggle — the "check every robot's work state" button */
   AddButton(c_painter, QRect(nX + nPad, nY, nInner, nBtnH),
             m_bShowStatus ? VN("ẨN TRẠNG THÁI ROBOT (T)")
                           : VN("KIỂM TRA TRẠNG THÁI ROBOT (T)"),
             COL_NEUTRAL, ACT_TOGGLE_STATUS, true);
   nY += nBtnH + nGap;

   /* Selected robot + per-robot commands */
   c_painter.setFont(cBody);
   c_painter.setPen(bSel ? COL_TEXT : COL_TEXT_DIM);
   QString strSel;
   if(bSel) {
      strSel = VN("Robot: ") + QString::fromStdString(m_strSelectedId)
             + "  ·  " + ModeName(m_pcSelected->GetOverride())
             + "  ·  " + StateName(*m_pcSelected)
             + "  ·  " + QString::number((int)(m_pcSelected->GetCharge() * 100.0)) + "%";
   }
   else {
      strSel = VN("Robot: chưa chọn — Shift+Click lên một robot");
   }
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 18), Qt::AlignLeft | Qt::AlignVCenter, strSel);
   nY += 18 + 4;

   AddButton(c_painter, QRect(nX + nPad, nY, nBtnW, nBtnH),
             VN("DỪNG (1)"), COL_ESTOP, ACT_SEL_STOP, bSel);
   AddButton(c_painter, QRect(nX + nPad + nBtnW + nGap, nY, nBtnW, nBtnH),
             VN("VỀ SẠC (2)"), COL_RECALL, ACT_SEL_CHARGE, bSel);
   AddButton(c_painter, QRect(nX + nPad + 2 * (nBtnW + nGap), nY, nInner - 2 * (nBtnW + nGap), nBtnH),
             VN("TỰ ĐỘNG (3)"), COL_RESUME, ACT_SEL_AUTO, bSel);
   nY += nBtnH + nGap;

   /* Hints */
   c_painter.setFont(cSmall);
   c_painter.setPen(COL_TEXT_DIM);
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 16), Qt::AlignLeft | Qt::AlignVCenter,
                      VN("Toàn đàn: X dừng · R chạy · H về sạc — Robot chọn: 1/2/3"));
   nY += 16;
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 14), Qt::AlignLeft | Qt::AlignVCenter,
                      VN("Camera: W/A/S/D/Q/E · chuột kéo — Chọn robot: Shift+Click"));

   nY = 12 + nH + 8;

   /* ---- Per-robot status board (the status-check view) ---- */
   std::vector<std::pair<std::string, CFootBotWarehouse*> > vecFleet = Fleet();

   /* Fleet summary strip (always visible) */
   {
      int nAuto = 0, nStop = 0, nRecall = 0, nDead = 0;
      for(size_t i = 0; i < vecFleet.size(); ++i) {
         if(vecFleet[i].second->IsDead()) { ++nDead; continue; }
         switch(vecFleet[i].second->GetOverride()) {
            case CFootBotWarehouse::OP_STOPPED: ++nStop;   break;
            case CFootBotWarehouse::OP_RECALL:  ++nRecall; break;
            default:                            ++nAuto;   break;
         }
      }
      QRect cSum(nX, nY, nW, 24);
      c_painter.setPen(Qt::NoPen);
      c_painter.setBrush(COL_PANEL_BG);
      c_painter.drawRoundedRect(cSum, 6, 6);
      c_painter.setFont(cSmall);
      c_painter.setPen(COL_TEXT);
      QString strSum = VN("Đàn: %1 tự động · %2 dừng · %3 về sạc")
                          .arg(nAuto).arg(nStop).arg(nRecall);
      if(nDead > 0) strSum += VN(" · %1 chết").arg(nDead);
      strSum += VN("  |  Đã giao: %1").arg(m_cLF.GetDelivered());
      c_painter.drawText(cSum.adjusted(nPad, 0, -nPad, 0),
                         Qt::AlignLeft | Qt::AlignVCenter, strSum);
      nY += 24 + 8;
   }

   if(m_bShowStatus) {
      const int nRowH = 17;
      int nBoardH = nPad + 16 + 4 + nRowH * (int)vecFleet.size() + nPad;
      QRect cBoard(nX, nY, nW, nBoardH);
      c_painter.setPen(Qt::NoPen);
      c_painter.setBrush(COL_PANEL_BG);
      c_painter.drawRoundedRect(cBoard, 6, 6);

      int nRowY = nY + nPad;
      const int nCol0 = nX + nPad;        /* ID */
      const int nCol1 = nX + nPad + 52;   /* mode */
      const int nCol2 = nX + nPad + 128;  /* state */
      const int nCol3 = nX + nPad + 218;  /* battery */
      const int nCol4 = nX + nPad + 272;  /* parcel */

      c_painter.setFont(cSmall);
      c_painter.setPen(COL_TEXT_DIM);
      c_painter.drawText(QPoint(nCol0, nRowY + 12), "ID");
      c_painter.drawText(QPoint(nCol1, nRowY + 12), VN("CHẾ ĐỘ"));
      c_painter.drawText(QPoint(nCol2, nRowY + 12), VN("TRẠNG THÁI"));
      c_painter.drawText(QPoint(nCol3, nRowY + 12), "PIN");
      c_painter.drawText(QPoint(nCol4, nRowY + 12), VN("HÀNG"));
      nRowY += 16 + 4;

      c_painter.setFont(cMono);
      for(size_t i = 0; i < vecFleet.size(); ++i) {
         CFootBotWarehouse& cCtrl = *vecFleet[i].second;
         bool bIsSel = (vecFleet[i].second == m_pcSelected);
         if(bIsSel) {
            c_painter.setPen(Qt::NoPen);
            c_painter.setBrush(QColor(255, 255, 255, 26));
            c_painter.drawRect(QRect(nX + 4, nRowY, nW - 8, nRowH));
         }
         int nTextY = nRowY + 13;
         c_painter.setPen(COL_TEXT);
         c_painter.drawText(QPoint(nCol0, nTextY),
                            QString::fromStdString(vecFleet[i].first));
         c_painter.setPen(cCtrl.IsDead() ? COL_TEXT_DIM : ModeColor(cCtrl.GetOverride()));
         c_painter.drawText(QPoint(nCol1, nTextY),
                            cCtrl.IsDead() ? VN("—") : ModeName(cCtrl.GetOverride()));
         c_painter.setPen(COL_TEXT);
         c_painter.drawText(QPoint(nCol2, nTextY), StateName(cCtrl));
         Real fFrac = cCtrl.GetCharge();
         QColor cPin = fFrac > 0.5 ? COL_RESUME
                     : fFrac > 0.25 ? QColor(235, 180, 60) : COL_ESTOP;
         c_painter.setPen(cPin);
         c_painter.drawText(QPoint(nCol3, nTextY),
                            QString::number((int)(fFrac * 100.0)) + "%");
         c_painter.setPen(COL_TEXT);
         c_painter.drawText(QPoint(nCol4, nTextY),
                            cCtrl.IsCarrying()
                               ? QString(QChar('A' + cCtrl.GetCarriedAddress()))
                               : VN("–"));
         nRowY += nRowH;
      }
   }

   c_painter.restore();
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWarehouseQTUserFunctions, "warehouse_qt_user_functions")
