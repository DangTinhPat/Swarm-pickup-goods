/**
 * grid_loop_functions.h — "Bảng đen" chia sẻ của nhà kho: chỉ ghi nhận
 * và trả lời (đặt chỗ ô, claim việc), tuyệt đối không áp đặt đường đi
 * lên robot. Kiêm hạ tầng: sinh hàng, sạc/xả pin, giám sát an toàn.
 *
 * Hiện thực chia theo mô-đun chức năng:
 *   grid_loop_functions.cpp  vòng đời Init/Reset/PostStep (điều phối)
 *   grid_reservations.cpp    ma trận chiếm dụng Grid[Tick][(Row,Col)]=ID
 *   grid_task_board.cpp      bảng việc: claim/bốc/giao, sinh hộp/yêu cầu
 *   grid_docks.cpp           trạng thái dock ẩn danh hai biên
 *   grid_energy_safety.cpp   pin định lượng + giám sát khoảng cách thân
 *   grid_metrics.cpp         log định kỳ + tổng kết cuối phiên
 *   grid_floor_render.cpp    GetFloorColor — nguồn dữ liệu cảm biến sàn
 */

#ifndef GRID_LOOP_FUNCTIONS_H
#define GRID_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

#include <common/grid_layout.h>

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

   /** Băng chuyền trưng bày tối đa 3 hộp (Queue/ClaimedBy song song). */
   struct SConveyor {
      static constexpr size_t QUEUE_CAP = 3;
      SGridCell            Cell;
      std::vector<UInt8>   Queue;
      std::vector<SInt32>  ClaimedBy;    /* -1 = chưa ai nhận            */
      UInt32               RespawnAt = 0;
      UInt32               Produced  = 0;
   };

   /** Yêu cầu màu tại một ô kệ; robot giao qua ô mặt kệ liền kề. */
   struct SDemand {
      SGridCell Cell;
      bool      Active        = false;
      UInt8     Color         = 0;
      SInt32    ClaimedBy     = -1;
      UInt32    CooldownUntil = 0;
      UInt32    Fulfilled     = 0;
   };

public:

   CGridLoopFunctions() {}
   virtual ~CGridLoopFunctions() {}

   virtual void   Init(TConfigurationNode& t_tree);
   virtual void   Reset();
   virtual void   PostStep();
   virtual void   PostExperiment();
   virtual CColor GetFloorColor(const CVector2& c_pos);

   /* ================= API bảng đen cho controller ================= */

   UInt32 Tick() const;

   /* --- grid_reservations.cpp --- */
   bool   TryReserveCell(UInt8 un_id, const SGridCell& s_cell, UInt32 un_until);
   void   ReleaseCell(UInt8 un_id, const SGridCell& s_cell, UInt32 un_grace);
   SInt32 CellReserver(const SGridCell& s_cell) const;

   /* --- grid_task_board.cpp --- */
   const std::vector<SConveyor>& GetConveyors() const { return m_vecConveyors; }
   const std::vector<SDemand>&   GetDemands()   const { return m_vecDemands; }
   bool TryClaimTask(UInt8 un_id, SInt32 n_conv, SInt32 n_dem);
   bool PickUpBox(UInt8 un_id, SInt32 n_conv, UInt8& un_color);
   bool DeliverBox(UInt8 un_id, SInt32 n_dem, UInt8 un_color);
   void AbandonTask(UInt8 un_id, bool b_keep_demand);

   /* --- grid_docks.cpp --- */
   SInt32 NearestFreeDock(const SGridCell& s_from, UInt8 un_id) const;
   bool   DockFree(SInt32 n_dock, UInt8 un_id) const;

   /* --- thống kê --- */
   void   NotifyEmergency(UInt8 un_id) { ++m_unEmergencies; }
   UInt32 GetDeliveredTotal() const { return m_unDeliveredTotal; }
   const std::array<UInt32, NUM_BOX_COLORS>& GetDeliveredPerColor() const {
      return m_arrDeliveredPerColor;
   }

private:

   /* grid_loop_functions.cpp */
   void InitDynamicState();
   SGridCell RobotCellOf(size_t un_idx) const;

   /* grid_reservations.cpp */
   void PruneOldReservations();

   /* grid_task_board.cpp */
   void UpdateConveyorSpawns();
   void SpawnDemandIfPossible();
   UInt8 PickBoxColor();
   UInt8 PickDemandColor();

   /* grid_energy_safety.cpp */
   void UpdateEnergyAndOdometry();
   void MonitorProximity();

   /* grid_metrics.cpp */
   void LogStatus();

private:

   /* --- Tham số kịch bản (<grid .../> trong .argos) --- */
   UInt32 m_unMaxActiveDemands = 12;
   UInt32 m_unDemandPeriod     = 30;
   UInt32 m_unBoxRespawnMin    = 20;
   UInt32 m_unBoxRespawnMax    = 80;
   UInt32 m_unCooldownMin      = 150;
   UInt32 m_unCooldownMax      = 450;
   UInt32 m_unLogPeriod        = 1200;

   /* --- Pin định lượng (đơn vị tuyệt đối, full_charge=10000):
    * xả = discharging_factor*BASE_DISCHARGE_RATE mỗi tick LĂN BÁNH
    * ngoài dock; nạp = charging_factor*BASE_CHARGE_RATE mỗi tick ĐỨNG
    * YÊN trong dock. Mặc định: cạn 100->20% sau ~16.7 phút chạy liên
    * tục, sạc 20->70% trong ~4.2 phút. Phần xả tính tại đây vì model
    * time_motion nội bộ của ARGoS lỗi NaN trong ToAngleAxis. --- */
   Real m_fDischargingFactor   = 0.05;
   Real m_fChargingFactor      = 2.0;
   static constexpr Real BASE_DISCHARGE_RATE = 16.0;
   static constexpr Real BASE_CHARGE_RATE    = 1.0;
   static constexpr Real MOVE_EPSILON        = 0.0003;   /* [m/tick]     */

   /* --- Hạ tầng động --- */
   std::vector<SConveyor> m_vecConveyors;
   std::vector<SDemand>   m_vecDemands;

   /* Ma trận chiếm dụng động theo tick; dọn rác tick quá khứ mỗi PostStep */
   std::map<UInt32, std::map<std::pair<SInt32, SInt32>, UInt8>> m_sGridReservations;
   mutable std::mutex     m_muxBoard;

   std::vector<CFootBotEntity*> m_vecBots;      /* chỉ số = Id robot     */
   std::vector<CFootBotGrid*>   m_vecCtrls;
   std::vector<CVector2>        m_vecLastPos;
   std::vector<Real>            m_vecDistance;
   std::array<std::array<bool, 10>, 10> m_arrConflictLatch{};

   CFloorEntity* m_pcFloor = nullptr;
   CRandom::CRNG* m_pcRNG  = nullptr;
   bool m_bFloorDirty      = false;

   /* --- Thống kê --- */
   UInt32 m_unDeliveredTotal = 0;
   std::array<UInt32, NUM_BOX_COLORS> m_arrDeliveredPerColor{};
   UInt32 m_unEmergencies    = 0;
   UInt32 m_unHardCollisions = 0;
   UInt32 m_unNearMisses     = 0;
};

}  /* namespace argos */

#endif
