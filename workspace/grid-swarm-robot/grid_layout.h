/**
 * grid_layout.h
 *
 * NGUỒN CHÂN LÝ DUY NHẤT về hình học nhà kho — cả Controller lẫn Loop
 * Functions include file này, nhờ đó ma trận lưới, vật cản cứng, dock
 * sạc, băng chuyền, ngăn xếp và giao thức RAB luôn nhất quán 2 phía.
 *
 * PHIÊN BẢN LƯỚI MỊN 50 x 50 Ô, MỖI Ô 0.2 m x 0.2 m (arena 10 m x 10 m).
 * Kích cỡ ô = đường kính thân Foot-bot thực đo (2 * 0.085036758 m =
 * 0.17007 m, ARGoS) + 0.03 m biên an toàn — ô chỉ vừa khít một robot,
 * KHÔNG còn dư địa cho sai số định vị lớn, nên bộ lái + chốt QR phải
 * rất chính xác (xem footbot_grid_nav.cpp).
 *
 * BỐ CỤC (Row parametrize trục X, Col parametrize trục Y — xem công
 * thức ánh xạ bên dưới):
 *
 *   Col=0 (y=-4.9)                                    Col=49 (y=+4.9)
 *   dock TÂY (5 ô, Row 22-26) ............... dock ĐÔNG (5 ô, Row 22-26)
 *
 *   Row=12 (x=-2.5)  ████████████████████████████████  <- ngăn xếp 1 (vật cản)
 *   Row=13..24 (x=-2.3..-0.1) ..... hành lang lưu thông tự do .....
 *   Row=25 (x=+0.1)  ████████████████████████████████  <- ngăn xếp 2 (vật cản)
 *   Row=26..37 (x=+0.3..+2.5) ..... hành lang lưu thông tự do .....
 *   Row=38 (x=+2.7)  ████████████████████████████████  <- ngăn xếp 3 (vật cản)
 *
 *   Col=5 (y=-3.9), Row=5/20/35 (x=-3.9/-0.9/+2.1): 3 miệng băng chuyền
 *
 *   ████ = CELL_OBSTACLE thật (có <box> vật lý movable="false" đè khít
 *   trong .argos) — robot KHÔNG được lập kế hoạch đi vào, và vật lý
 *   dynamics2d cũng chặn cứng nếu có sai số cố tình lấn vào.
 */

#ifndef GRID_LAYOUT_H
#define GRID_LAYOUT_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/general.h>

#include <array>
#include <cstdlib>
#include <utility>

namespace argos {

/****************************************/
/* 1. KÍCH THƯỚC LƯỚI & ÁNH XẠ TỌA ĐỘ   */
/****************************************/

constexpr Real   CELL_SIZE  = 0.2;      /* cạnh một ô lưới [m]            */
constexpr SInt32 GRID_ROWS  = 50;       /* số hàng (parametrize trục X)   */
constexpr SInt32 GRID_COLS  = 50;       /* số cột  (parametrize trục Y)   */
/* Arena 10m x 10m đặt tâm tại gốc -> mép âm nằm ở -5.0 m mỗi trục */
constexpr Real   HALF_SPAN  = GRID_ROWS * CELL_SIZE * 0.5;   /* = 5.0 m   */

/**
 * CÔNG THỨC ÁNH XẠ TỌA ĐỘ LIÊN TỤC (x, y) -> MA TRẬN NGUYÊN (Row, Col)
 * ---------------------------------------------------------------------
 *   Row = (int)((x + 5.0) / 0.2)      Col = (int)((y + 5.0) / 0.2)
 *
 * Diễn giải: tịnh tiến gốc tọa độ từ tâm arena (0,0) về góc dưới của
 * dải [-5,+5) trên từng trục (cộng HALF_SPAN để mọi giá trị thành
 * không âm), rồi chia cho cạnh ô 0.2 m và LẤY PHẦN NGUYÊN (không làm
 * tròn) — floor() cho số dương tương đương ép kiểu (int). Row bám
 * trục X, Col bám trục Y — ĐẢO NGƯỢC so với quy ước (row~y, col~x)
 * thường gặp, đây là lựa chọn tường minh của bố cục nhà kho này (các
 * "hàng ngăn xếp" là các dải Row cố định chạy dọc theo trục Y/Col).
 * Kết quả kẹp về [0, 49] để chống lỗi số thực khi robot chạm sát tường.
 */
inline SInt32 WorldXToRow(Real f_x) {
   SInt32 nRow = static_cast<SInt32>(Floor((f_x + HALF_SPAN) / CELL_SIZE));
   return (nRow < 0) ? 0 : (nRow >= GRID_ROWS ? GRID_ROWS - 1 : nRow);
}
inline SInt32 WorldYToCol(Real f_y) {
   SInt32 nCol = static_cast<SInt32>(Floor((f_y + HALF_SPAN) / CELL_SIZE));
   return (nCol < 0) ? 0 : (nCol >= GRID_COLS ? GRID_COLS - 1 : nCol);
}

/**
 * CÔNG THỨC NGƯỢC: (Row, Col) -> TÂM Ô (x, y) TRONG THẾ GIỚI LIÊN TỤC
 * ---------------------------------------------------------------------
 *   x_tâm = -5.0 + (Row + 0.5) * 0.2      y_tâm = -5.0 + (Col + 0.5) * 0.2
 *
 * Cộng 0.5 để lấy đúng HỒNG TÂM ô — nơi dán "mã QR sàn" (đĩa đen bán
 * kính 0.02 m, khớp đúng ngưỡng khử sai số ở footbot_grid_nav.cpp).
 */
inline Real RowToWorldX(SInt32 n_row) { return -HALF_SPAN + (n_row + 0.5) * CELL_SIZE; }
inline Real ColToWorldY(SInt32 n_col) { return -HALF_SPAN + (n_col + 0.5) * CELL_SIZE; }

/** Một ô lưới (địa chỉ nguyên trong ma trận) */
struct SGridCell {
   SInt32 Row = -1;
   SInt32 Col = -1;
   SGridCell() = default;
   SGridCell(SInt32 n_row, SInt32 n_col) : Row(n_row), Col(n_col) {}
   bool operator==(const SGridCell& s) const { return Row == s.Row && Col == s.Col; }
   bool operator!=(const SGridCell& s) const { return !(*this == s); }
   bool operator<(const SGridCell& s) const {
      return (Row != s.Row) ? (Row < s.Row) : (Col < s.Col);
   }
   bool IsValid() const {
      return Row >= 0 && Row < GRID_ROWS && Col >= 0 && Col < GRID_COLS;
   }
   /* Khoảng cách Manhattan — heuristic chuẩn cho A* trên lưới 4 hướng */
   SInt32 ManhattanTo(const SGridCell& s) const {
      return std::abs(Row - s.Row) + std::abs(Col - s.Col);
   }
};

/****************************************/
/* 2. VẬT CẢN CỨNG: 3 HÀNG NGĂN XẾP     */
/****************************************/

/* 3 hàng ngăn xếp = 3 dải Row CỐ ĐỊNH, chạy dọc Col=10..39 (30 ô = 6.0 m,
 * khớp đúng bề dài <box size="0.2,6.0,0.5">). Đề bài viết "Col=10 đến
 * 40" nhưng kích thước box neo cứng là 6.0 m = 30 ô -> dùng biên trên
 * là 39 (không phải 40) để khớp khít vật lý với box, tránh hở 0.2 m.
 * Dồn 3 hàng lên khu vực PHÍA TRÊN bản đồ (Row lớn), mỗi hàng chỉ cách
 * hàng kế 4 Row (3 hàng trống làm hành lang) — dày hơn hẳn bố cục cũ
 * (từng cách nhau 13 Row) theo yêu cầu; băng chuyền dồn xuống đáy, xem
 * mục 3 bên dưới. */
constexpr std::array<SInt32, 3> STACK_ROWS    = { 38, 42, 46 };
constexpr SInt32                STACK_COL_MIN = 10;
constexpr SInt32                STACK_COL_MAX = 39;
constexpr SInt32 STACK_CELLS_PER_ROW = STACK_COL_MAX - STACK_COL_MIN + 1;  /* 30 */
constexpr SInt32 NUM_STACK_CELLS     = 3 * STACK_CELLS_PER_ROW;           /* 90 */

inline SGridCell StackCell(SInt32 n_idx) {
   return SGridCell(STACK_ROWS[n_idx / STACK_CELLS_PER_ROW],
                    STACK_COL_MIN + (n_idx % STACK_CELLS_PER_ROW));
}
inline bool IsStackObstacle(SInt32 n_row, SInt32 n_col) {
   if(n_col < STACK_COL_MIN || n_col > STACK_COL_MAX) return false;
   for(SInt32 r : STACK_ROWS) if(r == n_row) return true;
   return false;
}

/**
 * Ô ngăn xếp là VẬT CẢN VẬT LÝ (CELL_OBSTACLE) — robot không được và
 * không thể lái vào. Việc "giao hàng vào ô (Row,Col)" thực chất là
 * đứng ở Ô MẶT KỆ (Face Cell) — ô lưu thông TRỐNG liền kề ngay trước
 * hoặc sau dải vật cản (Row-1 hoặc Row+1) — và đặt/nhận hộp qua mặt
 * kệ, giống băng tải hai mặt trong kho thật. Mỗi ô ngăn xếp có 2 mặt
 * kệ khả dụng (phía hành lang trước và sau); robot chọn mặt GẦN HƠN.
 */
inline SGridCell StackFaceCell(const SGridCell& s_stack, bool b_far_side) {
   SInt32 nRow = s_stack.Row + (b_far_side ? 1 : -1);
   return SGridCell(nRow, s_stack.Col);
}

/****************************************/
/* 3. DOCK SẠC & BĂNG CHUYỀN            */
/****************************************/

/* 10 dock sạc chia đều 2 biên Đông/Tây: Col=0 (Tây) và Col=49 (Đông),
 * Row liên tục 22..26 (5 ô mỗi bên). */
constexpr SInt32 NUM_DOCKS = 10;
constexpr SInt32 DOCK_ROW_MIN = 22;
inline SGridCell DockCell(SInt32 n_idx) {
   /* dock 0..4 ở biên TÂY (Col 0), dock 5..9 ở biên ĐÔNG (Col 49) */
   SInt32 nRow = DOCK_ROW_MIN + (n_idx % 5);
   return (n_idx < 5) ? SGridCell(nRow, 0) : SGridCell(nRow, GRID_COLS - 1);
}

/* 3 miệng băng chuyền dồn xuống ĐÁY bản đồ (Row nhỏ, đối diện phía kệ
 * hàng đã dồn lên đỉnh): cùng Row=2 cố định, trải dọc theo Col=12/25/38
 * để dễ phân biệt và chừa lối đi rộng lên khu vực dock/kệ hàng. */
constexpr SInt32 NUM_CONVEYORS = 3;
constexpr SInt32 CONVEYOR_ROW  = 2;
constexpr std::array<SInt32, NUM_CONVEYORS> CONVEYOR_COLS = { 12, 25, 38 };
inline SGridCell ConveyorCell(SInt32 n_idx) {
   return SGridCell(CONVEYOR_ROW, CONVEYOR_COLS[n_idx]);
}

/****************************************/
/* 4. PHÂN LOẠI Ô                        */
/****************************************/

enum EGridCellType : UInt8 {
   CELL_FREE     = 0,   /* sàn trống, lưu thông tự do                     */
   CELL_OBSTACLE = 1,   /* vật cản cứng (thân ngăn xếp) — cấm tuyệt đối   */
   CELL_DOCK     = 2,   /* dock sạc: chỉ vào khi đây là ĐÍCH của bạn      */
   CELL_CONVEYOR = 3    /* miệng băng chuyền: chỉ vào khi là đích nhận    */
};

inline EGridCellType CellTypeOf(SInt32 n_row, SInt32 n_col) {
   static const auto arrTable = [] {
      std::array<UInt8, GRID_ROWS * GRID_COLS> arr{};
      arr.fill(CELL_FREE);
      for(SInt32 r = 0; r < GRID_ROWS; ++r)
         for(SInt32 c = 0; c < GRID_COLS; ++c)
            if(IsStackObstacle(r, c)) arr[r * GRID_COLS + c] = CELL_OBSTACLE;
      for(SInt32 i = 0; i < NUM_DOCKS; ++i) {
         SGridCell s = DockCell(i);
         arr[s.Row * GRID_COLS + s.Col] = CELL_DOCK;
      }
      for(SInt32 i = 0; i < NUM_CONVEYORS; ++i) {
         SGridCell s = ConveyorCell(i);
         arr[s.Row * GRID_COLS + s.Col] = CELL_CONVEYOR;
      }
      return arr;
   }();
   if(n_row < 0 || n_row >= GRID_ROWS || n_col < 0 || n_col >= GRID_COLS)
      return CELL_OBSTACLE;   /* ngoài biên coi như vật cản */
   return static_cast<EGridCellType>(arrTable[n_row * GRID_COLS + n_col]);
}
inline EGridCellType CellTypeOf(const SGridCell& s) { return CellTypeOf(s.Row, s.Col); }

/****************************************/
/* 5. MÀU HỘP HÀNG                       */
/****************************************/

enum EBoxColor : UInt8 { BOX_RED = 0, BOX_GREEN = 1, BOX_BLUE = 2, NUM_BOX_COLORS = 3 };

inline const CColor& BoxCColor(UInt8 un_color) {
   static const CColor arr[NUM_BOX_COLORS] = {
      CColor(230,  30,  30),   /* Đỏ         */
      CColor( 25, 190,  25),   /* Xanh lá    */
      CColor( 50, 110, 255)    /* Xanh dương */
   };
   return arr[un_color % NUM_BOX_COLORS];
}
inline const char* BoxColorName(UInt8 un_color) {
   static const char* arr[NUM_BOX_COLORS] = { "DO", "XANH-LA", "XANH-DUONG" };
   return arr[un_color % NUM_BOX_COLORS];
}

/****************************************/
/* 6. MỨC ƯU TIÊN GIAO THÔNG BẤT ĐỐI XỨNG */
/****************************************/
/* Số nhỏ = quyền cao. Ưu tiên là hàm THUẦN của trạng thái FSM (không
 * phụ thuộc %pin tức thời) để tránh "nhấp nháy" mức ưu tiên giữa
 * đường: một khi vào EMERGENCY_CHARGE thì giữ ưu tiên 1 tới khi rời
 * trạng thái đó (đạt 70%), không hạ dù pin đã nhích qua 20%.        */
constexpr UInt8 PRIO_EMERGENCY = 1;  /* STATE_EMERGENCY_CHARGE            */
constexpr UInt8 PRIO_DELIVERING = 2; /* STATE_DELIVERING (đang chở hàng)  */
constexpr UInt8 PRIO_IDLE      = 3;  /* mọi trạng thái không tải khác     */

/****************************************/
/* 7. GIAO THỨC BẢN TIN RAB (Range & Bearing) */
/****************************************/
constexpr UInt32 RAB_MSG_SIZE   = 16;
constexpr UInt32 RAB_IDX_ID     = 0;
constexpr UInt32 RAB_IDX_PRIO   = 1;
constexpr UInt32 RAB_IDX_STATE  = 2;
constexpr UInt32 RAB_IDX_CUR_R  = 3;
constexpr UInt32 RAB_IDX_CUR_C  = 4;
constexpr UInt32 RAB_IDX_NEXT_R = 5;   /* ô sắp bước vào - hàng (255=không) */
constexpr UInt32 RAB_IDX_NEXT_C = 6;
constexpr UInt32 RAB_IDX_FLAGS  = 7;   /* bit0=chở hàng, bit1=đang detour/yield */
constexpr UInt32 RAB_IDX_BATT   = 8;
constexpr UInt8  RAB_NO_CELL    = 255;
constexpr UInt8  RAB_FLAG_CARRY = 0x01;
constexpr UInt8  RAB_FLAG_YIELD = 0x02;

}  /* namespace argos */

#endif /* GRID_LAYOUT_H */
