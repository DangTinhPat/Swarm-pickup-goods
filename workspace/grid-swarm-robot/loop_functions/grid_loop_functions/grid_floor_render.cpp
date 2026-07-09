/**
 * grid_floor_render.cpp — GetFloorColor(): nguồn texture sàn, đồng thời
 * là DỮ LIỆU MÀ CẢM BIẾN SÀN CỦA ROBOT ĐỌC (độc lập với lớp vẽ 3D).
 *
 * Ràng buộc phần cứng: foot-bot có 8 cảm biến sàn cố định trên vòng
 * bán kính ~0.08 m quanh tâm thân -> đĩa QR định vị phải vẽ bán kính
 * >= 0.085 m thì cảm biến mới chạm tới được. Đĩa QR là vùng DUY NHẤT
 * trên sàn có độ xám < 0.08 (điều kiện nhận diện phía controller).
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

   /* Đĩa QR đen tại hồng tâm mọi ô lưu thông được */
   if(fR2 <= 0.085 * 0.085) return CColor::BLACK;

   /* Vạch kẻ lưới mảnh dọc biên ô (nửa cạnh ô = 0.1 m) */
   if(fDx > 0.095 || fDx < -0.095 || fDy > 0.095 || fDy < -0.095)
      return CColor(150, 150, 150);

   /* Viền màu mỏng còn lại phân biệt loại ô; màu hộp/yêu cầu cụ thể
    * hiển thị bằng hologram 3D (grid_qt_user_functions.cpp) */
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
