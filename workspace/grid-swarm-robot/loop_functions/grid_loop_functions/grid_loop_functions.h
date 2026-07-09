/**
 * grid_loop_functions.h
 *
 * LOOP FUNCTIONS = HẠ TẦNG NHÀ KHO + "BẢNG ĐEN" CHIA SẺ, tuyệt đối
 * KHÔNG phải máy chủ điều phối trung tâm — chỉ TRẢ LỜI xin đặt chỗ,
 * không bao giờ áp đặt đường đi lên robot:
 *
 *   - MA TRẬN CHIẾM DỤNG ĐỘNG THEO TỪNG TICK, đúng cấu trúc dữ liệu
 *     yêu cầu: m_sGridReservations[Tick][(Row,Col)] = RobotID (dùng
 *     std::map<UInt32, std::map<pair<SInt32,SInt32>, UInt8>> — thêm
 *     kiểu giá trị UInt8 cho tầng trong so với "std::set" gợi ý trong
 *     đặc tả, vì set không có chỗ lưu RobotID mà biểu thức "=Robot_ID"
 *     đòi hỏi). Có dọn rác định kỳ các tick đã qua để không phình bộ
 *     nhớ vô hạn qua một phiên chạy dài (15-20+ phút = 9000-12000+ tick).
 *   - Sinh hộp hàng ngẫu nhiên trên 3 băng chuyền, sinh yêu cầu màu
 *     ngẫu nhiên trên 90 ô "mặt kệ" của 3 hàng ngăn xếp (vật cản cứng).
 *   - Mô hình pin ĐỊNH LƯỢNG: xả tuyến tính khi robot THỰC SỰ DI
 *     CHUYỂN (đo bằng quãng đường tick thực tế, không chỉ đứng ở ô
 *     không dock), nạp khi đứng yên trong ô dock — theo đúng 2 hệ số
 *     discharging_factor / charging_factor khai báo trong .argos.
 *   - Vẽ sàn (lưới 0.2m + đĩa QR đen r=0.02m + màu trạm) và thống kê.
 */

#ifndef GRID_LOOP_FUNCTIONS_H
#define GRID_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

#include <grid_layout.h>

#include <array>
#include <vector>
#include <map>
#include <utility>
#include <mutex>

namespace argos {

class CFootBotEntity;
class CFloorEntity;
class CFootBotGrid;

class CGridLoopFunctions : public CLoopFunctions {

public:

   /* Băng chuyền "trưng bày sẵn" tối đa 3 hộp cùng lúc (hàng đợi hiển
    * thị, không bắt buộc lấy đúng thứ tự FIFO — Queue[k]/ClaimedBy[k]
    * đi theo cặp chỉ số). Sinh hộp ngẫu nhiên định kỳ khi còn chỗ. */
   struct SConveyor {
      static constexpr size_t QUEUE_CAP = 3;
      SGridCell            Cell;
      std::vector<UInt8>   Queue;       /* màu các hộp đang trưng bày   */
      std::vector<SInt32>  ClaimedBy;   /* song song Queue; -1=chưa ai nhận */
      UInt32               RespawnAt = 0;
      UInt32               Produced  = 0;
   };

   /* Một ô "mặt kệ" ngăn xếp: khi Active nó "đang yêu cầu 1 hộp màu
    * Color". Cell ở đây là ô VẬT CẢN (CELL_OBSTACLE) đại diện vị trí
    * kệ; robot giao hàng thực sự đứng ở StackFaceCell(Cell, gần/xa). */
   struct SDemand {
      SGridCell Cell;
      bool      Active        = false;
      UInt8     Color         = 0;
      SInt32    ClaimedBy     = -1;
      UInt32    CooldownUntil = 0;
      UInt32    Fulfilled     = 0;
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

   /* --- Ma trận chiếm dụng động Grid[Tick][(Row,Col)] = RobotID --- */
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

   /* --- Trạng thái dock ẩn danh ở hai biên Đông/Tây --- */
   SInt32 NearestFreeDock(const SGridCell& s_from, UInt8 un_id) const;
   bool   DockFree(SInt32 n_dock, UInt8 un_id) const;

   /* --- Thống kê --- */
   void   NotifyEmergency(UInt8 un_id) { ++m_unEmergencies; }
   UInt32 GetDeliveredTotal() const { return m_unDeliveredTotal; }
   const std::array<UInt32, NUM_BOX_COLORS>& GetDeliveredPerColor() const {
      return m_arrDeliveredPerColor;
   }

private:

   void InitDynamicState();
   void SpawnDemandIfPossible();
   UInt8 PickBoxColor();
   UInt8 PickDemandColor();
   void LogStatus();
   SGridCell RobotCellOf(size_t un_idx) const;
   void PruneOldReservations();

private:

   /* Tham số kịch bản (đọc từ <grid .../> trong .argos) */
   UInt32 m_unMaxActiveDemands = 12;
   UInt32 m_unDemandPeriod     = 30;
   UInt32 m_unBoxRespawnMin    = 20;
   UInt32 m_unBoxRespawnMax    = 80;
   UInt32 m_unCooldownMin      = 150;
   UInt32 m_unCooldownMax      = 450;
   UInt32 m_unLogPeriod        = 1200;

   /* ---- Mô hình pin định lượng (đơn vị tuyệt đối, full_charge=10000) ----
    * delta_xả  = discharging_factor * BASE_DISCHARGE_RATE  [đơn vị/tick]
    *             áp dụng CHỈ khi robot thực sự di chuyển (quãng đường
    *             đo được trong tick > ngưỡng nhiễu vật lý) và KHÔNG ở
    *             ô dock.
    * delta_sạc = charging_factor * BASE_CHARGE_RATE        [đơn vị/tick]
    *             áp dụng khi đứng yên (vận tốc bánh ~ 0, suy ra từ
    *             quãng đường đo được ~ 0) trong ô dock.
    * BASE_* hiệu chỉnh sao cho ở giá trị mặc định discharging_factor=
    * 0.05 / charging_factor=2.0 (đúng số đề bài), robot di chuyển liên
    * tục cạn pin 100%->20% sau ~10 000 tick (=1000s=16.7 phút, nằm
    * trong khoảng 15-20 phút yêu cầu), và sạc 20%->70% trong ~2500
    * tick (=250s=4.2 phút). Xem chú thích chi tiết trong .cpp. */
   Real m_fDischargingFactor   = 0.05;
   Real m_fChargingFactor      = 2.0;
   static constexpr Real BASE_DISCHARGE_RATE = 16.0;
   static constexpr Real BASE_CHARGE_RATE    = 1.0;
   static constexpr Real MOVE_EPSILON        = 0.0003; /* [m/tick] ngưỡng "đang di chuyển" */

   /* Hạ tầng động */
   std::vector<SConveyor> m_vecConveyors;
   std::vector<SDemand>   m_vecDemands;

   /* Grid[Tick][(Row,Col)] = RobotID — đúng cấu trúc yêu cầu, có dọn
    * rác các Tick đã qua mỗi PostStep (xem PruneOldReservations). */
   std::map<UInt32, std::map<std::pair<SInt32, SInt32>, UInt8>> m_sGridReservations;
   mutable std::mutex     m_muxBoard;

   std::vector<CFootBotEntity*> m_vecBots;
   std::vector<CFootBotGrid*>   m_vecCtrls;
   std::vector<CVector2>        m_vecLastPos;
   std::vector<Real>            m_vecDistance;
   std::array<std::array<bool, 10>, 10> m_arrConflictLatch{};

   CFloorEntity* m_pcFloor = nullptr;
   CRandom::CRNG* m_pcRNG  = nullptr;
   bool m_bFloorDirty      = false;

   UInt32 m_unDeliveredTotal = 0;
   std::array<UInt32, NUM_BOX_COLORS> m_arrDeliveredPerColor{};
   UInt32 m_unEmergencies    = 0;
   UInt32 m_unHardCollisions = 0;   /* thân robot chạm nhau (<0.17m, phải = 0) */
   UInt32 m_unNearMisses     = 0;   /* < 1 cạnh ô (0.20m) nhưng chưa chạm      */
};

}  /* namespace argos */

#endif
