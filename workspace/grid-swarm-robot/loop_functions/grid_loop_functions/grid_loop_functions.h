/**
 * grid_loop_functions.h
 *
 * LOOP FUNCTIONS = HẠ TẦNG NHÀ KHO + "BẢNG ĐEN" CHIA SẺ, tuyệt đối
 * KHÔNG phải máy chủ điều phối trung tâm:
 *
 *   - Giữ MA TRẬN CHIẾM DỤNG THEO THỜI GIAN của lưới: mỗi ô lưu
 *     (chủ đặt chỗ, tick hết hạn) — dạng nén thưa của Grid[Row][Col][Tick]
 *     (thay vì nhân bản 20x20 cho từng tick, mỗi ô ghi MỘT cửa sổ
 *     [now..expiry] đang có hiệu lực — đủ cho kiểm tra va chạm).
 *     Loop Functions chỉ TRẢ LỜI xin đặt chỗ, không bao giờ áp đặt
 *     đường đi lên robot.
 *   - Sinh hộp hàng ngẫu nhiên trên 3 băng chuyền, sinh yêu cầu màu
 *     ngẫu nhiên trên các ô ngăn xếp.
 *   - Nạp pin cho MỌI robot đang đứng trong ô dock (dock ẩn danh:
 *     không phân biệt ID robot nào đang sạc, ai đến trước dùng trước).
 *   - Vẽ sàn (lưới + đĩa QR đen + màu trạm) và gom số liệu thống kê.
 */

#ifndef GRID_LOOP_FUNCTIONS_H
#define GRID_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

#include <grid_layout.h>

#include <array>
#include <vector>
#include <mutex>

namespace argos {

class CFootBotEntity;
class CFloorEntity;
class CFootBotGrid;

class CGridLoopFunctions : public CLoopFunctions {

public:

   /* Một miệng băng chuyền: nhả tối đa 1 hộp chờ tại một thời điểm */
   struct SConveyor {
      SGridCell Cell;
      bool      HasBox    = false;
      UInt8     Color     = 0;
      SInt32    ClaimedBy = -1;   /* robot đã nhận hộp này (-1: chưa ai) */
      UInt32    RespawnAt = 0;    /* tick băng chuyền nhả hộp kế tiếp    */
      UInt32    Produced  = 0;
   };

   /* Một ô ngăn xếp: khi Active nó "đang yêu cầu 1 hộp màu Color" */
   struct SDemand {
      SGridCell Cell;
      bool      Active        = false;
      UInt8     Color         = 0;
      SInt32    ClaimedBy     = -1;
      UInt32    CooldownUntil = 0;  /* nghỉ sau khi được nạp đầy         */
      UInt32    Fulfilled     = 0;
   };

   /* Cửa sổ đặt chỗ đang hiệu lực của một ô lưới */
   struct SCellReservation {
      SInt32 Owner  = -1;
      UInt32 Expiry = 0;
   };

public:

   CGridLoopFunctions();
   virtual ~CGridLoopFunctions() {}

   virtual void   Init(TConfigurationNode& t_tree);
   virtual void   Reset();
   virtual void   PostStep();
   virtual void   PostExperiment();
   virtual CColor GetFloorColor(const CVector2& c_pos);   /* grid_floor_render.cpp */

   /* ================== API BẢNG ĐEN cho controller ================== */

   UInt32 Tick() const;

   /* --- Ma trận chiếm dụng (đặt chỗ ô theo cửa sổ tick) --- */
   bool   TryReserveCell(UInt8 un_id, const SGridCell& s_cell, UInt32 un_until);
   void   ReleaseCell(UInt8 un_id, const SGridCell& s_cell, UInt32 un_grace);
   SInt32 CellReserver(const SGridCell& s_cell) const;

   /* --- Bảng việc (chỉ ghi nhận claim, không phân việc) --- */
   const std::vector<SConveyor>& GetConveyors() const { return m_vecConveyors; }
   const std::vector<SDemand>&   GetDemands()   const { return m_vecDemands; }
   bool TryClaimTask(UInt8 un_id, SInt32 n_conv, SInt32 n_dem);
   bool PickUpBox(UInt8 un_id, SInt32 n_conv, UInt8& un_color);
   bool DeliverBox(UInt8 un_id, SInt32 n_dem, UInt8 un_color);
   void AbandonTask(UInt8 un_id, bool b_keep_demand);

   /* --- Trạng thái dock ẩn danh ở hai biên bản đồ --- */
   SInt32 NearestFreeDock(const SGridCell& s_from, UInt8 un_id) const;
   bool   DockFree(SInt32 n_dock, UInt8 un_id) const;

   /* --- Thống kê --- */
   void   NotifyEmergency(UInt8 un_id) { ++m_unEmergencies; }
   UInt32 GetDeliveredTotal() const { return m_unDeliveredTotal; }
   const std::array<UInt32, NUM_BOX_COLORS>& GetDeliveredPerColor() const {
      return m_arrDeliveredPerColor;
   }

private:

   void InitDynamicState();          /* dùng chung cho Init() và Reset() */
   void SpawnDemandIfPossible();
   UInt8 PickBoxColor();
   UInt8 PickDemandColor();
   void LogStatus();
   SGridCell RobotCellOf(size_t un_idx) const;

private:

   /* Tham số XML */
   UInt32 m_unMaxActiveDemands = 7;
   UInt32 m_unDemandPeriod     = 40;    /* tick giữa 2 lần thử mở yêu cầu */
   UInt32 m_unBoxRespawnMin    = 30;
   UInt32 m_unBoxRespawnMax    = 120;
   UInt32 m_unCooldownMin      = 200;
   UInt32 m_unCooldownMax      = 600;
   Real   m_fChargeRate        = 0.0025; /* phân số pin nạp mỗi tick      */
   /* Mô hình xả pin tuyến tính "time_motion" (tính tại đây thay cho
    * discharge model nội bộ của ARGoS vốn dính lỗi NaN ở ToAngleAxis):
    *   delta_tick = time_factor + pos_factor * |quãng đường tick này|  */
   Real   m_fTimeFactor        = 0.00001;
   Real   m_fPosFactor         = 0.005;
   UInt32 m_unLogPeriod        = 1200;

   /* Hạ tầng động */
   std::vector<SConveyor> m_vecConveyors;
   std::vector<SDemand>   m_vecDemands;
   std::array<SCellReservation, GRID_ROWS * GRID_COLS> m_arrReservations;
   mutable std::mutex     m_muxBoard;   /* an toàn nếu chạy đa luồng      */

   /* Robot */
   std::vector<CFootBotEntity*> m_vecBots;      /* chỉ số = Id robot      */
   std::vector<CFootBotGrid*>   m_vecCtrls;
   std::vector<CVector2>        m_vecLastPos;
   std::vector<Real>            m_vecDistance;
   std::array<std::array<bool, 10>, 10> m_arrConflictLatch{};

   CFloorEntity* m_pcFloor = nullptr;
   CRandom::CRNG* m_pcRNG  = nullptr;
   bool m_bFloorDirty      = false;

   /* Thống kê */
   UInt32 m_unDeliveredTotal = 0;
   std::array<UInt32, NUM_BOX_COLORS> m_arrDeliveredPerColor{};
   UInt32 m_unEmergencies    = 0;
   UInt32 m_unConflicts      = 0;   /* cặp robot áp sát < 25 cm (phải = 0) */
};

}  /* namespace argos */

#endif
