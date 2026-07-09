/**
 * grid_geometry.h — Kích thước lưới và ánh xạ tọa độ liên tục ⇄ ma trận.
 *
 * Quy ước toàn cục: Row bám TRỤC X, Col bám TRỤC Y (đảo so với quy ước
 * row~y/col~x thường gặp) — các "dãy kệ" là dải Row cố định chạy dọc Y.
 */

#ifndef GRID_GEOMETRY_H
#define GRID_GEOMETRY_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/general.h>

#include <cstdlib>

namespace argos {

/* CELL_SIZE = đường kính thân foot-bot (0.17007 m) + 0.03 m biên an toàn
 * — một ô chỉ vừa khít một robot. */
constexpr Real   CELL_SIZE = 0.2;
constexpr SInt32 GRID_ROWS = 30;
constexpr SInt32 GRID_COLS = 30;
constexpr Real   HALF_SPAN = GRID_ROWS * CELL_SIZE * 0.5;   /* = 3.0 m */

/* Row = (int)((x + HALF_SPAN) / CELL_SIZE); kẹp biên chống lỗi số thực. */
inline SInt32 WorldXToRow(Real f_x) {
   SInt32 nRow = static_cast<SInt32>(Floor((f_x + HALF_SPAN) / CELL_SIZE));
   return (nRow < 0) ? 0 : (nRow >= GRID_ROWS ? GRID_ROWS - 1 : nRow);
}
inline SInt32 WorldYToCol(Real f_y) {
   SInt32 nCol = static_cast<SInt32>(Floor((f_y + HALF_SPAN) / CELL_SIZE));
   return (nCol < 0) ? 0 : (nCol >= GRID_COLS ? GRID_COLS - 1 : nCol);
}

/* Chiều ngược trả về HỒNG TÂM ô — nơi đặt "mã QR sàn". */
inline Real RowToWorldX(SInt32 n_row) { return -HALF_SPAN + (n_row + 0.5) * CELL_SIZE; }
inline Real ColToWorldY(SInt32 n_col) { return -HALF_SPAN + (n_col + 0.5) * CELL_SIZE; }

/** Địa chỉ một ô lưới trong ma trận. */
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
   SInt32 ManhattanTo(const SGridCell& s) const {
      return std::abs(Row - s.Row) + std::abs(Col - s.Col);
   }
};

}  /* namespace argos */

#endif
