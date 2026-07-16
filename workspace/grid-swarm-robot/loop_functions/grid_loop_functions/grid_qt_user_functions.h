/**
 * grid_qt_user_functions.h — LỚP VẼ TRÊN GIAO DIỆN 3D + BẢNG ĐIỀU KHIỂN
 * VẬN HÀNH (operator console).
 *
 * Lưu ý tên gọi: một số framework gọi hook vẽ sau mỗi khung hình là
 * PostDraw(); trong ARGoS3, API tương đương của CQTOpenGLUserFunctions
 * là DrawInWorld() (vẽ trong hệ tọa độ thế giới, sau khi arena đã vẽ
 * xong) và Draw(CFootBotEntity&) (vẽ đè trong hệ tọa độ từng robot).
 *
 * Vẽ: 21+21 đường kẻ lưới 3D, hộp hàng đang chờ trên băng chuyền, hộp
 * "hologram" báo màu đang yêu cầu trên ngăn xếp, nhãn trạng thái + %
 * pin + hộp đang chở trên lưng từng robot.
 *
 * Console: lớp lệnh của NGƯỜI VẬN HÀNH đè lên tự hành (như fleet-manager
 * thật) — chỉ lật cờ EOverride của từng controller, không điều phối hộ.
 * Headless không nạp file này nên benchmark giữ nguyên.
 *   Nút (click)        E-STOP / CHẠY LẠI / VỀ SẠC (toàn đàn) · DỪNG /
 *                      VỀ SẠC / TỰ ĐỘNG (robot đang chọn) · bảng trạng thái
 *   Phím               X e-stop · R chạy lại · H về sạc (toàn đàn) ·
 *                      T bảng trạng thái · 1/2/3 (robot đang chọn)
 *   Shift+Click robot  chọn robot cho lệnh riêng
 */

#ifndef GRID_QT_USER_FUNCTIONS_H
#define GRID_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_grid/footbot_grid.h>

#include <QRect>
#include <string>
#include <utility>
#include <vector>

class QPainter;
class QKeyEvent;
class QMouseEvent;

namespace argos {

class CGridLoopFunctions;

class CGridQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CGridQTUserFunctions();
   virtual ~CGridQTUserFunctions() {}

   /* Vẽ trong hệ thế giới — "PostDraw" của ARGoS */
   virtual void DrawInWorld();

   /* Vẽ trong hệ tọa độ cục bộ của từng foot-bot */
   void Draw(CFootBotEntity& c_entity);

   /* --- Bảng điều khiển vận hành --- */
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

   /* Nút HUD click được: dựng lại mỗi lần DrawOverlay, hit-test trong
    * MouseKeyReleased (click thuần = nhấn + thả không kéo chuột) */
   struct SButton {
      QRect   Rect;
      EAction Action;
   };

   /* Cả đàn dưới dạng (id, controller), sắp theo số hiệu robot */
   std::vector<std::pair<UInt8, CFootBotGrid*> > Fleet() const;
   void FleetOverride(CFootBotGrid::EOverride e_op);
   void RunAction(EAction e_action);
   void AddButton(QPainter& c_painter, const QRect& c_rect,
                  const QString& c_label, const QColor& c_border,
                  EAction e_action, bool b_enabled);

   CGridLoopFunctions& LF();
   CGridLoopFunctions* m_pcLF = nullptr;

   /* Trạng thái console */
   CFootBotGrid* m_pcSelected = nullptr;   /* robot chọn qua Shift+Click */
   bool          m_bShowStatus = false;    /* bảng trạng thái bật/tắt   */
   std::vector<SButton> m_vecButtons;
};

}  /* namespace argos */

#endif
