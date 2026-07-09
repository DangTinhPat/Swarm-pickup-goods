/**
 * grid_protocol.h — Các quy ước chung giữa mọi robot: màu hộp hàng,
 * bảng mức ưu tiên giao thông, và layout gói bản tin RAB 16 byte.
 */

#ifndef GRID_PROTOCOL_H
#define GRID_PROTOCOL_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/color.h>

namespace argos {

/* --- Màu hộp hàng --- */
enum EBoxColor : UInt8 { BOX_RED = 0, BOX_GREEN = 1, BOX_BLUE = 2, NUM_BOX_COLORS = 3 };

inline const CColor& BoxCColor(UInt8 un_color) {
   static const CColor arr[NUM_BOX_COLORS] = {
      CColor(230,  30,  30),
      CColor( 25, 190,  25),
      CColor( 50, 110, 255)
   };
   return arr[un_color % NUM_BOX_COLORS];
}
inline const char* BoxColorName(UInt8 un_color) {
   static const char* arr[NUM_BOX_COLORS] = { "DO", "XANH-LA", "XANH-DUONG" };
   return arr[un_color % NUM_BOX_COLORS];
}

/* --- Ưu tiên giao thông (số nhỏ = quyền cao). Là hàm THUẦN của trạng
 * thái FSM (không theo %pin tức thời) để không nhấp nháy giữa đường. --- */
constexpr UInt8 PRIO_EMERGENCY  = 1;   /* STATE_EMERGENCY_CHARGE          */
constexpr UInt8 PRIO_DELIVERING = 2;   /* STATE_DELIVERING (đang chở hộp) */
constexpr UInt8 PRIO_IDLE       = 3;   /* mọi trạng thái không tải khác   */

/* --- Gói RAB 16 byte, phát quảng bá mỗi tick --- */
constexpr UInt32 RAB_MSG_SIZE   = 16;
constexpr UInt32 RAB_IDX_ID     = 0;
constexpr UInt32 RAB_IDX_PRIO   = 1;
constexpr UInt32 RAB_IDX_STATE  = 2;
constexpr UInt32 RAB_IDX_CUR_R  = 3;
constexpr UInt32 RAB_IDX_CUR_C  = 4;
constexpr UInt32 RAB_IDX_NEXT_R = 5;   /* 255 = không có ô kế             */
constexpr UInt32 RAB_IDX_NEXT_C = 6;
constexpr UInt32 RAB_IDX_FLAGS  = 7;
constexpr UInt32 RAB_IDX_BATT   = 8;
constexpr UInt8  RAB_NO_CELL    = 255;
constexpr UInt8  RAB_FLAG_CARRY = 0x01;
constexpr UInt8  RAB_FLAG_YIELD = 0x02;

}  /* namespace argos */

#endif
