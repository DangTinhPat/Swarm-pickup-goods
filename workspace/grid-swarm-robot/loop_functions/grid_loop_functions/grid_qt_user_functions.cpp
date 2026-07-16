/**
 * grid_qt_user_functions.cpp — Lớp vẽ debug 3D + bảng điều khiển vận
 * hành. DrawInWorld() là API tương đương "PostDraw" của ARGoS3 (không
 * tồn tại PostDraw/DrawLine trong API thật; DrawRay là hàm vẽ đoạn
 * thẳng).
 *
 * Lớp phủ che đĩa QR CHỈ mang tính hiển thị: cảm biến sàn đọc thẳng
 * GetFloorColor() nên định vị không bị ảnh hưởng bởi bất cứ thứ gì vẽ
 * ở đây.
 */

#include "grid_qt_user_functions.h"
#include "grid_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

#include <QPainter>
#include <QKeyEvent>
#include <QMouseEvent>
#include <algorithm>
#include <sstream>

namespace argos {

/****************************************/
/****************************************/

/* Bảng màu console (đồng bộ với warehouse-swarm) */
static const QColor COL_PANEL_BG(12, 16, 24, 215);
static const QColor COL_TEXT(235, 238, 245);
static const QColor COL_TEXT_DIM(150, 158, 170);
static const QColor COL_ESTOP(220, 60, 50);
static const QColor COL_RESUME(70, 190, 90);
static const QColor COL_RECALL(70, 140, 240);
static const QColor COL_NEUTRAL(120, 128, 140);

static QString VN(const char* pch_utf8) { return QString::fromUtf8(pch_utf8); }

static QString ModeName(CFootBotGrid::EOverride e_op) {
   switch(e_op) {
      case CFootBotGrid::OP_STOPPED: return VN("DỪNG");
      case CFootBotGrid::OP_RECALL:  return VN("VỀ SẠC");
      default:                       return VN("TỰ ĐỘNG");
   }
}

static QColor ModeColor(CFootBotGrid::EOverride e_op) {
   switch(e_op) {
      case CFootBotGrid::OP_STOPPED: return COL_ESTOP;
      case CFootBotGrid::OP_RECALL:  return COL_RECALL;
      default:                       return COL_RESUME;
   }
}

static QColor BoxQColor(UInt8 un_color) {
   CColor c = BoxCColor(un_color);
   return QColor(c.GetRed(), c.GetGreen(), c.GetBlue());
}

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
   if(pcCtrl->GetOverride() == CFootBotGrid::OP_STOPPED)     oss << " [STOP]";
   else if(pcCtrl->GetOverride() == CFootBotGrid::OP_RECALL) oss << " [VE SAC]";
   if(pcCtrl->IsInTraffic()) oss << " *";
   Real fBatt = pcCtrl->GetBatteryFrac();
   CColor cCol = (fBatt < 0.20) ? CColor::RED
               : (fBatt < 0.70) ? CColor(200, 120, 0)
                                : CColor(0, 120, 0);
   DrawText(CVector3(-0.08, 0.0, 0.22), oss.str(), cCol);
}

/****************************************/
/****************************************/

std::vector<std::pair<UInt8, CFootBotGrid*> >
CGridQTUserFunctions::Fleet() const {
   std::vector<std::pair<UInt8, CFootBotGrid*> > vecFleet;
   CSpace::TMapPerType& cFootBots =
      CSimulator::GetInstance().GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFB = *any_cast<CFootBotEntity*>(it->second);
      CFootBotGrid* pcCtrl = dynamic_cast<CFootBotGrid*>(
         &cFB.GetControllableEntity().GetController());
      if(pcCtrl != nullptr)
         vecFleet.push_back(std::make_pair(pcCtrl->GetRobotId(), pcCtrl));
   }
   std::sort(vecFleet.begin(), vecFleet.end(),
             [](const std::pair<UInt8, CFootBotGrid*>& a,
                const std::pair<UInt8, CFootBotGrid*>& b) {
                return a.first < b.first;
             });
   return vecFleet;
}

void CGridQTUserFunctions::FleetOverride(CFootBotGrid::EOverride e_op) {
   std::vector<std::pair<UInt8, CFootBotGrid*> > vecFleet = Fleet();
   for(size_t i = 0; i < vecFleet.size(); ++i) {
      vecFleet[i].second->SetOverride(e_op);
   }
}

void CGridQTUserFunctions::RunAction(EAction e_action) {
   switch(e_action) {
      case ACT_FLEET_ESTOP:   FleetOverride(CFootBotGrid::OP_STOPPED); break;
      case ACT_FLEET_RESUME:  FleetOverride(CFootBotGrid::OP_AUTO);    break;
      case ACT_FLEET_RECALL:  FleetOverride(CFootBotGrid::OP_RECALL);  break;
      case ACT_TOGGLE_STATUS: m_bShowStatus = !m_bShowStatus;          break;
      case ACT_SEL_STOP:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotGrid::OP_STOPPED);
         break;
      case ACT_SEL_CHARGE:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotGrid::OP_RECALL);
         break;
      case ACT_SEL_AUTO:
         if(m_pcSelected) m_pcSelected->SetOverride(CFootBotGrid::OP_AUTO);
         break;
   }
}

/****************************************/
/****************************************/

void CGridQTUserFunctions::KeyPressed(QKeyEvent* pc_event) {
   /* W/A/S/D/Q/E + mũi tên là phím camera — console chỉ dùng phím khác,
    * phím lạ chuyển tiếp cho handler mặc định */
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

void CGridQTUserFunctions::MouseKeyReleased(QMouseEvent* pc_event) {
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

void CGridQTUserFunctions::EntitySelected(CEntity& c_entity) {
   CFootBotEntity* pcFB = dynamic_cast<CFootBotEntity*>(&c_entity);
   if(pcFB != nullptr) {
      m_pcSelected = dynamic_cast<CFootBotGrid*>(
         &pcFB->GetControllableEntity().GetController());
   }
   else {
      m_pcSelected = nullptr;
   }
   GetQTOpenGLWidget().update();
}

void CGridQTUserFunctions::EntityDeselected(CEntity& c_entity) {
   m_pcSelected = nullptr;
   GetQTOpenGLWidget().update();
}

/****************************************/
/****************************************/

void CGridQTUserFunctions::AddButton(QPainter& c_painter, const QRect& c_rect,
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

void CGridQTUserFunctions::DrawOverlay(QPainter& c_painter) {
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

   const int nX = 12;
   const int nPad = 12;
   const int nW = 350;
   const int nInner = nW - 2 * nPad;
   const int nBtnH = 26;
   const int nGap = 8;
   const bool bSel = (m_pcSelected != nullptr);

   /* ---- Panel chính ---- */
   int nH = nPad + 18 + nGap
          + nBtnH + nGap
          + nBtnH + nGap
          + 18 + 4
          + nBtnH + nGap
          + 16 + 14 + nPad;
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

   c_painter.setFont(cBody);
   int nBtnW = (nInner - 2 * nGap) / 3;
   AddButton(c_painter, QRect(nX + nPad, nY, nBtnW, nBtnH),
             VN("E-STOP (X)"), COL_ESTOP, ACT_FLEET_ESTOP, true);
   AddButton(c_painter, QRect(nX + nPad + nBtnW + nGap, nY, nBtnW, nBtnH),
             VN("CHẠY LẠI (R)"), COL_RESUME, ACT_FLEET_RESUME, true);
   AddButton(c_painter, QRect(nX + nPad + 2 * (nBtnW + nGap), nY, nInner - 2 * (nBtnW + nGap), nBtnH),
             VN("VỀ SẠC (H)"), COL_RECALL, ACT_FLEET_RECALL, true);
   nY += nBtnH + nGap;

   AddButton(c_painter, QRect(nX + nPad, nY, nInner, nBtnH),
             m_bShowStatus ? VN("ẨN TRẠNG THÁI ROBOT (T)")
                           : VN("KIỂM TRA TRẠNG THÁI ROBOT (T)"),
             COL_NEUTRAL, ACT_TOGGLE_STATUS, true);
   nY += nBtnH + nGap;

   c_painter.setFont(cBody);
   c_painter.setPen(bSel ? COL_TEXT : COL_TEXT_DIM);
   QString strSel;
   if(bSel) {
      strSel = VN("Robot: fb") + QString::number((int)m_pcSelected->GetRobotId())
             + "  ·  " + ModeName(m_pcSelected->GetOverride())
             + "  ·  " + QString::fromLatin1(m_pcSelected->GetStateName())
             + "  ·  " + QString::number((int)(m_pcSelected->GetBatteryFrac() * 100.0)) + "%";
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

   c_painter.setFont(cSmall);
   c_painter.setPen(COL_TEXT_DIM);
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 16), Qt::AlignLeft | Qt::AlignVCenter,
                      VN("Toàn đàn: X dừng · R chạy · H về sạc — Robot chọn: 1/2/3"));
   nY += 16;
   c_painter.drawText(QRect(nX + nPad, nY, nInner, 14), Qt::AlignLeft | Qt::AlignVCenter,
                      VN("Camera: W/A/S/D/Q/E · chuột kéo — Chọn robot: Shift+Click"));

   nY = 12 + nH + 8;

   /* ---- Dải tổng hợp + bảng trạng thái ---- */
   std::vector<std::pair<UInt8, CFootBotGrid*> > vecFleet = Fleet();

   {
      int nAuto = 0, nStop = 0, nRecall = 0;
      for(size_t i = 0; i < vecFleet.size(); ++i) {
         switch(vecFleet[i].second->GetOverride()) {
            case CFootBotGrid::OP_STOPPED: ++nStop;   break;
            case CFootBotGrid::OP_RECALL:  ++nRecall; break;
            default:                       ++nAuto;   break;
         }
      }
      QRect cSum(nX, nY, nW, 24);
      c_painter.setPen(Qt::NoPen);
      c_painter.setBrush(COL_PANEL_BG);
      c_painter.drawRoundedRect(cSum, 6, 6);
      c_painter.setFont(cSmall);
      c_painter.setPen(COL_TEXT);
      QString strSum = VN("Đàn: %1 tự động · %2 dừng · %3 về sạc  |  Đã giao: %4")
                          .arg(nAuto).arg(nStop).arg(nRecall)
                          .arg(LF().GetDeliveredTotal());
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
      const int nCol1 = nX + nPad + 52;   /* chế độ */
      const int nCol2 = nX + nPad + 128;  /* trạng thái */
      const int nCol3 = nX + nPad + 250;  /* pin */
      const int nCol4 = nX + nPad + 296;  /* hàng */

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
         CFootBotGrid& cCtrl = *vecFleet[i].second;
         if(vecFleet[i].second == m_pcSelected) {
            c_painter.setPen(Qt::NoPen);
            c_painter.setBrush(QColor(255, 255, 255, 26));
            c_painter.drawRect(QRect(nX + 4, nRowY, nW - 8, nRowH));
         }
         int nTextY = nRowY + 13;
         c_painter.setPen(COL_TEXT);
         c_painter.drawText(QPoint(nCol0, nTextY),
                            "fb" + QString::number((int)vecFleet[i].first));
         c_painter.setPen(ModeColor(cCtrl.GetOverride()));
         c_painter.drawText(QPoint(nCol1, nTextY), ModeName(cCtrl.GetOverride()));
         c_painter.setPen(COL_TEXT);
         c_painter.drawText(QPoint(nCol2, nTextY),
                            QString::fromLatin1(cCtrl.GetStateName()));
         Real fBatt = cCtrl.GetBatteryFrac();
         QColor cPin = fBatt > 0.5 ? COL_RESUME
                     : fBatt > 0.2 ? QColor(235, 180, 60) : COL_ESTOP;
         c_painter.setPen(cPin);
         c_painter.drawText(QPoint(nCol3, nTextY),
                            QString::number((int)(fBatt * 100.0)) + "%");
         if(cCtrl.IsCarrying()) {
            c_painter.setPen(Qt::NoPen);
            c_painter.setBrush(BoxQColor(cCtrl.GetCarriedColor()));
            c_painter.drawRect(QRect(nCol4, nRowY + 3, 12, 12));
         }
         else {
            c_painter.setPen(COL_TEXT_DIM);
            c_painter.drawText(QPoint(nCol4, nTextY), VN("–"));
         }
         nRowY += nRowH;
      }
   }

   c_painter.restore();
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CGridQTUserFunctions, "grid_qt_user_functions")

}  /* namespace argos */
