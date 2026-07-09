/**
 * grid_floor_render.cpp — VẼ SÀN NHÀ KHO (texture sàn ARGoS truy vấn
 * qua GetFloorColor cho từng pixel; đồng thời là dữ liệu mà cảm biến
 * sàn của robot đọc được — "camera gầm chụp mã QR").
 *
 * Bảng màu quanh "độ xám" mà footbot_motor_ground trả về (0=đen..1=trắng):
 *
 *   - ĐĨA QR ĐỊNH VỊ: ĐEN tuyệt đối, bán kính vật lý 0.085 m tại hồng
 *     tâm MỌI ô lưu thông được. QUAN TRỌNG: foot-bot thật trong ARGoS
 *     có ĐÚNG 8 cảm biến sàn gắn cố định trên một vòng bán kính ~0.08 m
 *     quanh tâm thân robot (hằng số phần cứng, xem footbot_entity.cpp)
 *     — một đĩa bán kính 0.02 m như số liệu "vùng tiếp cận" trong đề
 *     bài sẽ NẰM HOÀN TOÀN BÊN TRONG vòng cảm biến và không cảm biến
 *     nào chạm tới được (đã kiểm chứng thực nghiệm: 0 lần chốt QR dù
 *     robot đi qua chính xác tâm ô). Do đó đĩa phải vẽ đủ lớn (0.085 m,
 *     nhỉnh hơn vòng cảm biến 0.08 m) để phần cứng THẤY ĐƯỢC nó; số
 *     0.02 m của đề bài được dùng làm NGƯỠNG TIN CẬY vị trí ước lượng
 *     trước khi THỰC THI gán cưỡng bức (m_fQrSnapRadius trong
 *     footbot_grid_nav.cpp) — đúng câu chữ "khi robot đã tiếp cận
 *     vùng bán kính <=0.02 m" mới "gán cưỡng bức", tách bạch với việc
 *     cảm biến có nhìn thấy đĩa hay không.
 *   - Vạch lưới      : xám 150, dải mỏng 0.005 m sát biên ô (ô chỉ
 *     0.2 m, đĩa QR đã chiếm phần lớn bán kính 0.1 m của ô).
 *   - Ô CELL_OBSTACLE: xám đậm (thân ngăn xếp — có <box> vật lý đè
 *     khít bên trên trong .argos, đây chỉ là màu nền dự phòng).
 *   - Ô CELL_DOCK    : viền xanh nhạt quanh đĩa QR (10 ô, 2 biên).
 *   - Ô CELL_CONVEYOR / mặt kệ: viền xám/be quanh đĩa QR; MÀU HỘP/
 *     YÊU CẦU không vẽ vành khăn trên sàn (không đủ chỗ cạnh đĩa QR ở
 *     ô 0.2 m) mà hiển thị bằng khối "hologram" 3D nổi trên sàn —
 *     xem grid_qt_user_functions.cpp DrawInWorld().
 */

#include "grid_loop_functions.h"

namespace argos {

/****************************************/
/****************************************/

CColor CGridLoopFunctions::GetFloorColor(const CVector2& c_pos) {
   const SInt32 nRow = WorldXToRow(c_pos.GetX());
   const SInt32 nCol = WorldYToCol(c_pos.GetY());
   const Real   fDx  = c_pos.GetX() - RowToWorldX(nRow);
   const Real   fDy  = c_pos.GetY() - ColToWorldY(nCol);
   const Real   fR2  = fDx * fDx + fDy * fDy;

   const EGridCellType eType = CellTypeOf(nRow, nCol);

   if(eType == CELL_OBSTACLE) return CColor(90, 90, 95);

   /* 1. Đĩa QR đen bán kính 0.085 m ở hồng tâm mọi ô lưu thông được
    * (đủ lớn để vòng 8 cảm biến sàn thật, bán kính ~0.08 m, luôn có
    * ít nhất 1 cảm biến chạm đĩa khi robot căn tâm — xem giải thích
    * đầu file). */
   if(fR2 <= 0.085 * 0.085) return CColor::BLACK;

   /* 2. Vạch kẻ lưới mảnh 0.005 m dọc biên ô (nửa cạnh = 0.1 m) */
   if(fDx > 0.095 || fDx < -0.095 || fDy > 0.095 || fDy < -0.095)
      return CColor(150, 150, 150);

   /* 3. Viền màu mỏng còn lại (0.085-0.095 m) phân biệt loại ô — màu
    * hộp/yêu cầu cụ thể chuyển sang hologram 3D (không đủ chỗ ở đây) */
   if(eType == CELL_DOCK)     return CColor(170, 218, 232);
   if(eType == CELL_CONVEYOR) return CColor(205, 205, 210);

   for(const SDemand& sDem : m_vecDemands) {
      if(!sDem.Active) continue;
      SGridCell sFaceA = StackFaceCell(sDem.Cell, false);
      SGridCell sFaceB = StackFaceCell(sDem.Cell, true);
      if(SGridCell(nRow, nCol) == sFaceA || SGridCell(nRow, nCol) == sFaceB)
         return CColor(232, 226, 208);
   }

   return CColor::WHITE;
}

}  /* namespace argos */
