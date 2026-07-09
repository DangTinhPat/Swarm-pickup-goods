/**
 * grid_qt_user_functions.h — LỚP VẼ DEBUG TRÊN GIAO DIỆN 3D CỦA ARGoS
 *
 * Lưu ý tên gọi: một số framework gọi hook vẽ sau mỗi khung hình là
 * PostDraw(); trong ARGoS3, API tương đương của CQTOpenGLUserFunctions
 * là DrawInWorld() (vẽ trong hệ tọa độ thế giới, sau khi arena đã vẽ
 * xong) và Draw(CFootBotEntity&) (vẽ đè trong hệ tọa độ từng robot).
 *
 * Vẽ: 21+21 đường kẻ lưới 3D, hộp hàng đang chờ trên băng chuyền, hộp
 * "hologram" báo màu đang yêu cầu trên ngăn xếp, nhãn trạng thái + %
 * pin + hộp đang chở trên lưng từng robot.
 */

#ifndef GRID_QT_USER_FUNCTIONS_H
#define GRID_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

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

private:

   CGridLoopFunctions& LF();
   CGridLoopFunctions* m_pcLF = nullptr;
};

}  /* namespace argos */

#endif
