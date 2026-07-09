/**
 * grid_floor_render.cpp — VẼ SÀN NHÀ KHO (texture sàn do ARGoS truy
 * vấn qua GetFloorColor cho từng pixel, đồng thời là dữ liệu mà cảm
 * biến sàn của robot đọc được).
 *
 * Bảng màu được chọn có chủ đích quanh "độ xám" mà cảm biến
 * footbot_motor_ground trả về (0 = đen .. 1 = trắng):
 *
 *   - ĐĨA QR ĐỊNH VỊ  : ĐEN tuyệt đối (độ xám 0.00) bán kính 10 cm tại
 *     hồng tâm MỌI ô — thứ duy nhất trên sàn có độ xám < 0.08, nên
 *     controller nhận diện tuyệt đối an toàn.
 *   - Vạch lưới        : xám 150 (độ xám ~0.59)
 *   - Ô dock           : xanh nhạt (5 ô biên trái + 5 ô biên phải)
 *   - Ô băng chuyền    : xám nền + VÀNH KHĂN màu hộp đang chờ
 *   - Ô ngăn xếp       : be nền + VÀNH KHĂN màu đang yêu cầu
 *     (vành khăn bán kính 24..42 cm — không bao giờ đè lên đĩa QR)
 */

#include "grid_loop_functions.h"

namespace argos {

/****************************************/
/****************************************/

CColor CGridLoopFunctions::GetFloorColor(const CVector2& c_pos) {
   /* Ánh xạ pixel (x, y) -> ô (Row, Col) rồi tính độ lệch so với tâm ô */
   const SInt32 nCol = WorldXToCol(c_pos.GetX());
   const SInt32 nRow = WorldYToRow(c_pos.GetY());
   const Real   fDx  = c_pos.GetX() - ColToWorldX(nCol);
   const Real   fDy  = c_pos.GetY() - RowToWorldY(nRow);
   const Real   fR2  = fDx * fDx + fDy * fDy;

   /* 1. Đĩa QR đen ở hồng tâm mọi ô (mốc xóa sai số odometry) */
   if(fR2 <= 0.10 * 0.10) return CColor::BLACK;

   /* 2. Vạch kẻ lưới rộng 6 cm dọc biên ô — nhìn rõ ô 1 m x 1 m */
   if(fDx > 0.47 || fDx < -0.47 || fDy > 0.47 || fDy < -0.47)
      return CColor(150, 150, 150);

   const EGridCellType eType = CellTypeOf(nRow, nCol);

   if(eType == CELL_DOCK) {
      /* nền xanh dịu — nhìn phát biết ngay bãi sạc hai biên */
      return CColor(170, 218, 232);
   }

   if(eType == CELL_CONVEYOR) {
      for(const SConveyor& sConv : m_vecConveyors) {
         if(!(sConv.Cell == SGridCell(nRow, nCol))) continue;
         if(sConv.HasBox && fR2 >= 0.24 * 0.24 && fR2 <= 0.42 * 0.42)
            return BoxCColor(sConv.Color);      /* hộp chờ trên miệng băng */
         break;
      }
      return CColor(205, 205, 210);
   }

   if(eType == CELL_STACK) {
      for(const SDemand& sDem : m_vecDemands) {
         if(!(sDem.Cell == SGridCell(nRow, nCol))) continue;
         if(sDem.Active && fR2 >= 0.24 * 0.24 && fR2 <= 0.42 * 0.42)
            return BoxCColor(sDem.Color);       /* "đang cần 1 hộp màu này" */
         break;
      }
      return CColor(232, 226, 208);
   }

   /* 3. Sàn lưu thông: trắng */
   return CColor::WHITE;
}

}  /* namespace argos */
