/**
 * grid_map.h — Bố cục cơ sở vật chất nhà kho: 3 dãy kệ (vật cản cứng),
 * 10 dock sạc hai biên, 3 băng chuyền, và bảng phân loại ô.
 *
 * Vị trí kệ ở đây phải khớp với các <box> vật lý trong grid_swarm.argos
 * (không có kiểm tra chéo lúc chạy).
 */

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <common/grid_geometry.h>

#include <array>

namespace argos {

/* --- 3 dãy kệ: dải Row cố định, cách nhau 4 Row (3 hàng hành lang), ---
 * --- trải Col=5..24 (20 ô = 4.0 m, khớp box size="0.2,4.0,0.5").    --- */
constexpr std::array<SInt32, 3> STACK_ROWS    = { 18, 22, 26 };
constexpr SInt32                STACK_COL_MIN = 5;
constexpr SInt32                STACK_COL_MAX = 24;
constexpr SInt32 STACK_CELLS_PER_ROW = STACK_COL_MAX - STACK_COL_MIN + 1;
constexpr SInt32 NUM_STACK_CELLS     = 3 * STACK_CELLS_PER_ROW;

inline SGridCell StackCell(SInt32 n_idx) {
   return SGridCell(STACK_ROWS[n_idx / STACK_CELLS_PER_ROW],
                    STACK_COL_MIN + (n_idx % STACK_CELLS_PER_ROW));
}
inline bool IsStackObstacle(SInt32 n_row, SInt32 n_col) {
   if(n_col < STACK_COL_MIN || n_col > STACK_COL_MAX) return false;
   for(SInt32 r : STACK_ROWS) if(r == n_row) return true;
   return false;
}

/* Ô kệ là vật cản — robot giao hàng đứng ở Ô MẶT KỆ: ô trống liền kề
 * trước (Row-1) hoặc sau (Row+1) dải kệ, chọn mặt gần hơn. */
inline SGridCell StackFaceCell(const SGridCell& s_stack, bool b_far_side) {
   return SGridCell(s_stack.Row + (b_far_side ? 1 : -1), s_stack.Col);
}

/* --- 10 dock sạc ẩn danh: Col=0 (Tây) và Col=29 (Đông), Row 10..14 --- */
constexpr SInt32 NUM_DOCKS    = 10;
constexpr SInt32 DOCK_ROW_MIN = 10;
inline SGridCell DockCell(SInt32 n_idx) {
   SInt32 nRow = DOCK_ROW_MIN + (n_idx % 5);
   return (n_idx < 5) ? SGridCell(nRow, 0) : SGridCell(nRow, GRID_COLS - 1);
}

/* --- 3 miệng băng chuyền ở đáy bản đồ: Row=2, Col=7/15/23 --- */
constexpr SInt32 NUM_CONVEYORS = 3;
constexpr SInt32 CONVEYOR_ROW  = 2;
constexpr std::array<SInt32, NUM_CONVEYORS> CONVEYOR_COLS = { 7, 15, 23 };
inline SGridCell ConveyorCell(SInt32 n_idx) {
   return SGridCell(CONVEYOR_ROW, CONVEYOR_COLS[n_idx]);
}

/* --- Phân loại ô --- */
enum EGridCellType : UInt8 {
   CELL_FREE     = 0,   /* lưu thông tự do                          */
   CELL_OBSTACLE = 1,   /* thân kệ — cấm tuyệt đối                  */
   CELL_DOCK     = 2,   /* chỉ vào khi là đích của chính robot      */
   CELL_CONVEYOR = 3    /* chỉ vào khi là đích của chính robot      */
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
      return CELL_OBSTACLE;
   return static_cast<EGridCellType>(arrTable[n_row * GRID_COLS + n_col]);
}
inline EGridCellType CellTypeOf(const SGridCell& s) { return CellTypeOf(s.Row, s.Col); }

}  /* namespace argos */

#endif
