/**
 * grid_layout.h
 *
 * NGUỒN CHÂN LÝ DUY NHẤT (single source of truth) về hình học nhà kho:
 * cả Controller lẫn Loop Functions đều include file này, nhờ đó bản đồ
 * lưới, vị trí dock sạc, băng chuyền, hàng ngăn xếp và giao thức bản tin
 * RAB luôn nhất quán giữa hai phía.
 *
 * BỐ CỤC NHÀ KHO 20 x 20 ô (mỗi ô 1 m x 1 m, arena 20 m x 20 m):
 *
 *   hàng 19  . . . . S S S S S S S S S S S S . . . .   <- ngăn xếp 3 (đỉnh)
 *   hàng 17-18  <------ HÀNH LANG 2 Ô (2 chiều) ---->
 *   hàng 16  . . . . S S S S S S S S S S S S . . . .   <- ngăn xếp 2
 *   hàng 14-15  <------ HÀNH LANG 2 Ô (2 chiều) ---->
 *   hàng 13  . . . . S S S S S S S S S S S S . . . .   <- ngăn xếp 1
 *   hàng 12  .  (sàn trống - vùng lưu thông tự do)  .
 *   ...      D                                     D   <- dock trái/phải
 *   hàng  0  . . . C . . . . . C . . . . . C . . . .   <- 3 băng chuyền
 *            cột 0                              cột 19
 *
 *   D = 10 dock sạc chia đều 2 biên: 5 ở cột 0, 5 ở cột 19 (đối xứng)
 *   C = băng chuyền nhả hộp hàng 3 màu   S = ô ngăn xếp yêu cầu màu
 */

#ifndef GRID_LAYOUT_H
#define GRID_LAYOUT_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/general.h>

#include <array>
#include <cstdlib>

namespace argos {

/****************************************/
/* 1. KÍCH THƯỚC LƯỚI & ÁNH XẠ TỌA ĐỘ   */
/****************************************/

constexpr SInt32 GRID_ROWS  = 20;      /* số hàng (trục y)                */
constexpr SInt32 GRID_COLS  = 20;      /* số cột  (trục x)                */
constexpr Real   CELL_SIZE  = 1.0;     /* cạnh một ô lưới [m]             */
/* Arena đặt tâm tại gốc tọa độ nên mép trái/dưới nằm ở -10 m */
constexpr Real   HALF_W     = GRID_COLS * CELL_SIZE * 0.5;   /* = 10 m    */
constexpr Real   HALF_H     = GRID_ROWS * CELL_SIZE * 0.5;   /* = 10 m    */

/**
 * CÔNG THỨC ÁNH XẠ TỌA ĐỘ LIÊN TỤC (x, y) -> MA TRẬN NGUYÊN (Row, Col)
 * ---------------------------------------------------------------------
 * Thế giới liên tục:  x ∈ [-10, +10),  y ∈ [-10, +10)
 * Ma trận rời rạc:    Col ∈ [0, 19],   Row ∈ [0, 19]
 *
 *   Col = floor( (x + HALF_W) / CELL_SIZE )
 *   Row = floor( (y + HALF_H) / CELL_SIZE )
 *
 * Diễn giải: tịnh tiến gốc từ tâm arena về góc Tây-Nam (cộng HALF_W /
 * HALF_H để mọi tọa độ thành số không âm), rồi chia cho cạnh ô và lấy
 * phần nguyên dưới (floor). Ví dụ x = -9.5 m -> (-9.5+10)/1 = 0.5 ->
 * Col 0; x = +9.99 m -> 19.99 -> Col 19. Kết quả được kẹp (clamp) vào
 * biên để chống lỗi số thực khi robot chạm sát tường.
 */
inline SInt32 WorldXToCol(Real f_x) {
   SInt32 nCol = static_cast<SInt32>(Floor((f_x + HALF_W) / CELL_SIZE));
   return (nCol < 0) ? 0 : (nCol >= GRID_COLS ? GRID_COLS - 1 : nCol);
}
inline SInt32 WorldYToRow(Real f_y) {
   SInt32 nRow = static_cast<SInt32>(Floor((f_y + HALF_H) / CELL_SIZE));
   return (nRow < 0) ? 0 : (nRow >= GRID_ROWS ? GRID_ROWS - 1 : nRow);
}

/**
 * CÔNG THỨC NGƯỢC: (Row, Col) -> TÂM Ô (x, y) TRONG THẾ GIỚI LIÊN TỤC
 * ---------------------------------------------------------------------
 *   x_tâm = -HALF_W + (Col + 0.5) * CELL_SIZE
 *   y_tâm = -HALF_H + (Row + 0.5) * CELL_SIZE
 *
 * Cộng 0.5 để lấy đúng HỒNG TÂM ô vuông — đây là điểm mà bộ điều khiển
 * tỷ lệ luôn nhắm tới, và cũng là nơi dán "mã QR sàn" (đĩa đen) để robot
 * xóa sai số odometry mỗi lần đi ngang qua.
 */
inline Real ColToWorldX(SInt32 n_col) { return -HALF_W + (n_col + 0.5) * CELL_SIZE; }
inline Real RowToWorldY(SInt32 n_row) { return -HALF_H + (n_row + 0.5) * CELL_SIZE; }

/** Một ô lưới (địa chỉ nguyên trong ma trận) */
struct SGridCell {
   SInt32 Row = -1;
   SInt32 Col = -1;
   SGridCell() = default;
   SGridCell(SInt32 n_row, SInt32 n_col) : Row(n_row), Col(n_col) {}
   bool operator==(const SGridCell& s) const { return Row == s.Row && Col == s.Col; }
   bool operator!=(const SGridCell& s) const { return !(*this == s); }
   bool IsValid() const {
      return Row >= 0 && Row < GRID_ROWS && Col >= 0 && Col < GRID_COLS;
   }
   /* Khoảng cách Manhattan — heuristic chuẩn cho A* trên lưới 4 hướng */
   SInt32 ManhattanTo(const SGridCell& s) const {
      return std::abs(Row - s.Row) + std::abs(Col - s.Col);
   }
};

/****************************************/
/* 2. PHÂN LOẠI Ô & BỐ TRÍ CƠ SỞ VẬT CHẤT */
/****************************************/

enum EGridCellType : UInt8 {
   CELL_FREE     = 0,   /* sàn trống, đi lại tự do                        */
   CELL_DOCK     = 1,   /* dock sạc: chỉ được vào khi đây là ĐÍCH của bạn */
   CELL_CONVEYOR = 2,   /* miệng băng chuyền: chỉ vào khi là đích nhận    */
   CELL_STACK    = 3    /* ô ngăn xếp: chỉ vào khi là đích giao hàng      */
};

/* --- 10 dock sạc chia đều 2 biên (5 trái cột 0 + 5 phải cột 19) ------ */
constexpr SInt32 NUM_DOCKS = 10;
/* Hàng của dock: 3,6,9,12,15 — giãn cách 3 ô để robot ra/vào không kẹt */
constexpr std::array<SInt32, 5> DOCK_ROWS = { 3, 6, 9, 12, 15 };
inline SGridCell DockCell(SInt32 n_idx) {
   /* dock 0..4 ở biên TRÁI (cột 0), dock 5..9 ở biên PHẢI (cột 19) */
   return (n_idx < 5) ? SGridCell(DOCK_ROWS[n_idx], 0)
                      : SGridCell(DOCK_ROWS[n_idx - 5], GRID_COLS - 1);
}

/* --- 3 băng chuyền cấp hàng ở đáy bản đồ (hàng 0) -------------------- */
constexpr SInt32 NUM_CONVEYORS = 3;
constexpr std::array<SInt32, NUM_CONVEYORS> CONVEYOR_COLS = { 3, 9, 15 };
inline SGridCell ConveyorCell(SInt32 n_idx) {
   return SGridCell(0, CONVEYOR_COLS[n_idx]);
}

/* --- 3 hàng ngăn xếp + hành lang 2 ô giữa chúng ----------------------
 * Hàng 13 / 16 / 19: giữa hai hàng liên tiếp là ĐÚNG 2 hàng trống
 * (14-15 và 17-18) tạo hành lang hai chiều: hai robot xuất phát từ hai
 * phía dock trái/phải có thể đi song song hoặc đối đầu rồi né cục bộ. */
constexpr std::array<SInt32, 3> STACK_ROWS     = { 13, 16, 19 };
constexpr SInt32                STACK_COL_MIN  = 4;
constexpr SInt32                STACK_COL_MAX  = 15;
constexpr SInt32 STACK_CELLS_PER_ROW = STACK_COL_MAX - STACK_COL_MIN + 1; /* 12 */
constexpr SInt32 NUM_STACK_CELLS     = 3 * STACK_CELLS_PER_ROW;           /* 36 */
inline SGridCell StackCell(SInt32 n_idx) {
   return SGridCell(STACK_ROWS[n_idx / STACK_CELLS_PER_ROW],
                    STACK_COL_MIN + (n_idx % STACK_CELLS_PER_ROW));
}

/**
 * Tra cứu loại ô — bảng dựng sẵn một lần (static) để A* gọi hàng nghìn
 * lần mỗi giây mà không tốn vòng lặp.
 */
inline EGridCellType CellTypeOf(SInt32 n_row, SInt32 n_col) {
   static const auto arrTable = [] {
      std::array<UInt8, GRID_ROWS * GRID_COLS> arr{};
      arr.fill(CELL_FREE);
      for(SInt32 i = 0; i < NUM_DOCKS; ++i) {
         SGridCell c = DockCell(i);
         arr[c.Row * GRID_COLS + c.Col] = CELL_DOCK;
      }
      for(SInt32 i = 0; i < NUM_CONVEYORS; ++i) {
         SGridCell c = ConveyorCell(i);
         arr[c.Row * GRID_COLS + c.Col] = CELL_CONVEYOR;
      }
      for(SInt32 i = 0; i < NUM_STACK_CELLS; ++i) {
         SGridCell c = StackCell(i);
         arr[c.Row * GRID_COLS + c.Col] = CELL_STACK;
      }
      return arr;
   }();
   if(n_row < 0 || n_row >= GRID_ROWS || n_col < 0 || n_col >= GRID_COLS)
      return CELL_STACK; /* ngoài biên coi như vật cản */
   return static_cast<EGridCellType>(arrTable[n_row * GRID_COLS + n_col]);
}
inline EGridCellType CellTypeOf(const SGridCell& s) { return CellTypeOf(s.Row, s.Col); }

/****************************************/
/* 3. MÀU HỘP HÀNG                       */
/****************************************/

enum EBoxColor : UInt8 { BOX_RED = 0, BOX_GREEN = 1, BOX_BLUE = 2, NUM_BOX_COLORS = 3 };

/* Màu vẽ (đã nâng độ chói kênh lam/lục để độ xám mà cảm biến sàn đọc
 * được luôn > 0.10, không bao giờ bị nhầm với đĩa đen định vị < 0.08) */
inline const CColor& BoxCColor(UInt8 un_color) {
   static const CColor arr[NUM_BOX_COLORS] = {
      CColor(230,  30,  30),   /* Đỏ       */
      CColor( 25, 190,  25),   /* Xanh lá  */
      CColor( 50, 110, 255)    /* Xanh dương */
   };
   return arr[un_color % NUM_BOX_COLORS];
}
inline const char* BoxColorName(UInt8 un_color) {
   static const char* arr[NUM_BOX_COLORS] = { "DO", "XANH-LA", "XANH-DUONG" };
   return arr[un_color % NUM_BOX_COLORS];
}

/****************************************/
/* 4. MỨC ƯU TIÊN GIAO THÔNG (số nhỏ = ưu tiên cao) */
/****************************************/

constexpr UInt8 PRIO_EMERGENCY = 1;  /* pin < 20%, đang chạy về trạm sạc  */
constexpr UInt8 PRIO_CARRYING  = 2;  /* đang chở hộp hàng đi giao         */
constexpr UInt8 PRIO_IDLE      = 3;  /* chạy không tải / về dock nghỉ     */

/****************************************/
/* 5. GIAO THỨC BẢN TIN RAB (Range & Bearing) */
/****************************************/
/* Mỗi tick robot phát quảng bá cục bộ (bán kính ~3 m) gói 16 byte:      */
constexpr UInt32 RAB_MSG_SIZE   = 16;
constexpr UInt32 RAB_IDX_ID     = 0;   /* định danh robot 0..9            */
constexpr UInt32 RAB_IDX_PRIO   = 1;   /* mức ưu tiên 1/2/3               */
constexpr UInt32 RAB_IDX_STATE  = 2;   /* trạng thái FSM (để debug)       */
constexpr UInt32 RAB_IDX_CUR_R  = 3;   /* ô hiện tại - hàng               */
constexpr UInt32 RAB_IDX_CUR_C  = 4;   /* ô hiện tại - cột                */
constexpr UInt32 RAB_IDX_NEXT_R = 5;   /* ô sắp bước vào - hàng (255=không) */
constexpr UInt32 RAB_IDX_NEXT_C = 6;   /* ô sắp bước vào - cột  (255=không) */
constexpr UInt32 RAB_IDX_FLAGS  = 7;   /* bit0=đang chở hàng, bit1=đang nhường */
constexpr UInt32 RAB_IDX_BATT   = 8;   /* % pin (0..100)                  */
constexpr UInt8  RAB_NO_CELL    = 255; /* giá trị "không có ô"            */
constexpr UInt8  RAB_FLAG_CARRY = 0x01;
constexpr UInt8  RAB_FLAG_YIELD = 0x02;

}  /* namespace argos */

#endif /* GRID_LAYOUT_H */
