/**
 * footbot_grid.h
 *
 * BỘ ĐIỀU KHIỂN PHI TẬP TRUNG cho một AMR (foot-bot vi sai) trong nhà
 * kho lưới 30x30 (ô 0.2 m). Mỗi robot tự mình:
 *
 *   1. ĐỊNH VỊ: dead-reckoning từ encoder 2 bánh (trôi), XÓA SẠCH sai
 *      số khi cảm biến sàn bắt được đĩa QR ở hồng tâm ô (bán kính chốt
 *      0.02 m) -> footbot_grid_nav.cpp
 *   2. QUYẾT ĐỊNH: máy trạng thái nhiệm vụ + chính sách pin trễ
 *      (hysteresis 20% khẩn cấp / 70% mới được rời dock) -> footbot_grid.cpp
 *   3. GIAO THÔNG: đặt chỗ ô kế tiếp trên bảng chiếm dụng theo TICK,
 *      trao đổi ưu tiên bất đối xứng qua RAB, và khi thua quyền thì
 *      thực hiện DẠT LÀN CỤC BỘ 3 BƯỚC (không bao giờ vẽ lại đường
 *      vòng xa qua A* toàn cục) -> footbot_grid_traffic.cpp
 */

#ifndef FOOTBOT_GRID_H
#define FOOTBOT_GRID_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>

#include <grid_layout.h>

#include <vector>
#include <string>

namespace argos {

class CGridLoopFunctions;

class CFootBotGrid : public CCI_Controller {

public:

   /* ---------------- Máy trạng thái nhiệm vụ ---------------- */
   enum EState : UInt8 {
      STATE_IDLE = 0,          /* đậu ở dock, đủ pin (>=70%), chờ có việc  */
      STATE_TO_PICKUP,         /* chạy rỗng tới băng chuyền đã nhận        */
      STATE_PICKING,           /* đứng ở băng chuyền, bốc hộp lên          */
      STATE_DELIVERING,        /* CHỞ HÀNG tới ô mặt kệ ngăn xếp (ưu tiên 2) */
      STATE_DROPPING,          /* đứng ở mặt kệ, hạ hộp xuống              */
      STATE_RETURNING,         /* hết việc, chạy về dock trống gần nhất    */
      STATE_RESTING,           /* ngủ đông tại dock (idle > 100 tick), sạc thụ động */
      STATE_EMERGENCY_CHARGE   /* pin<=20%: hủy việc, chạy về dock, sạc tới >=70% */
   };

   /* ------------- Pha giao thông cục bộ (chồng lên FSM) ------------- */
   enum ETrafficPhase : UInt8 {
      TRAFFIC_NONE = 0,
      TRAFFIC_DETOURING,    /* đang chạy 3 bước dạt làn (rẽ/song song/rẽ về) */
      TRAFFIC_YIELDING      /* cả 2 ô bên đều nghẽn -> đứng im 1-2 tick     */
   };

   /* Nhiệm vụ đã nhận: cặp (băng chuyền có hộp, ô ngăn xếp cần màu đó) */
   struct STask {
      SInt32 ConveyorIdx = -1;
      SInt32 DemandIdx   = -1;
      UInt8  Color       = 0;
      bool   HasBox      = false;
      bool   FarSide     = false;  /* mặt kệ đang nhắm: gần (-1) hay xa (+1) */
      bool   IsValid() const { return DemandIdx >= 0; }
      void   Clear() { ConveyorIdx = -1; DemandIdx = -1; Color = 0; HasBox = false; FarSide = false; }
   };

   struct SNeighbor {
      UInt8     Id       = 255;
      UInt8     Prio     = PRIO_IDLE;
      UInt8     State    = 0;
      UInt8     Flags    = 0;
      UInt8     Batt     = 0;
      SGridCell Cur;
      SGridCell Next;
      Real      Range    = 0.0;
   };

public:

   CFootBotGrid();
   virtual ~CFootBotGrid() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   /* --------- API chỉ-đọc cho Loop Functions / GUI debug --------- */
   UInt8       GetRobotId()      const { return m_unId; }
   EState      GetState()        const { return m_eState; }
   const char* GetStateName()    const;
   UInt8       GetPriority()     const;
   bool        IsCarrying()      const { return m_sTask.HasBox; }
   UInt8       GetCarriedColor() const { return m_sTask.Color; }
   Real        GetBatteryFrac()  const { return m_fBattery; }
   bool        IsInTraffic()     const { return m_eTraffic != TRAFFIC_NONE; }
   UInt32      GetSnapCount()      const { return m_unSnapCount; }
   UInt32      GetRelocCount()     const { return m_unRelocCount; }
   UInt32      GetDetourCount()    const { return m_unDetourCount; }
   const SGridCell& GetCurCell() const { return m_sCurCell; }
   const std::vector<SGridCell>& GetPath() const { return m_vecPath; }

private:

   /* ============ footbot_grid.cpp : FSM + nhiệm vụ + pin ============ */
   void RunStateMachine();
   void CheckBatteryEmergency();
   bool TryClaimBestTask();
   void AfterTaskDone();
   bool RetargetDock(bool b_must_find);
   void SetGoal(const SGridCell& s_cell);
   void UpdateLed();

   /* ============ footbot_grid_nav.cpp : định vị + A* + bám tâm ô ==== */
   void UpdateLocalization();
   bool PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level);
   void ApplySteering(const CVector2& c_target, bool b_pivot_turn);
   void StopWheels();

   /* ============ footbot_grid_traffic.cpp : đặt chỗ + dạt làn ======== */
   void StepMovement();
   void ParseNeighbors();
   void BroadcastState();
   bool IWinAgainst(const SNeighbor& s_n) const;
   bool TryStartDetour(const SGridCell& s_blocked);
   void HandleDetourPhase();
   void HandleYieldPhase();
   void KeepAliveReservation();
   SGridCell PlannedNextCell() const;

   CGridLoopFunctions& LF();

private:

   /* ----------------------- Thiết bị ----------------------- */
   CCI_DifferentialSteeringActuator* m_pcWheels     = nullptr;
   CCI_DifferentialSteeringSensor*   m_pcWheelsSens = nullptr;
   CCI_RangeAndBearingActuator*      m_pcRABAct     = nullptr;
   CCI_RangeAndBearingSensor*        m_pcRABSens    = nullptr;
   CCI_PositioningSensor*            m_pcPosSens    = nullptr;
   CCI_LEDsActuator*                 m_pcLEDs       = nullptr;
   CCI_BatterySensor*                m_pcBattery    = nullptr;
   CCI_FootBotMotorGroundSensor*     m_pcGround     = nullptr;

   CGridLoopFunctions*               m_pcLF         = nullptr;

   /* ----------------------- Tham số XML ----------------------- */
   /* V_Base: 10 cm/s chạy thẳng, 3 cm/s khi bẻ lái 90 độ tại tâm ô
    * (yêu cầu tường minh, chống trượt bánh khi quay gấp trong ô 0.2m) */
   Real m_fCruiseSpeed = 10.0;   /* V_Base chạy thẳng [cm/s]              */
   Real m_fPivotSpeed  = 3.0;    /* V_Base khi rẽ 90 độ tại tâm ô [cm/s]  */
   Real m_fKpHeading   = 3.5;
   CRadians m_cHardTurn;         /* ngưỡng coi là "rẽ 90 độ" (mặc định 55 độ) */
   Real m_fWaypointTol = 0.02;   /* [m] coi như đã chạm tâm ô trung gian  */
   Real m_fGoalTol     = 0.02;   /* [m] dừng hẳn tại tâm ô đích           */
   Real m_fQrSnapRadius = 0.02;  /* [m] bán kính "đọc được QR sàn" — khớp đề bài */
   Real m_fLowBatt     = 0.20;   /* ngưỡng xả cứng: bỏ việc đi sạc khẩn cấp */
   Real m_fLeaveBatt   = 0.70;   /* ngưỡng sạc cứng: đủ mới được rời dock */
   UInt32 m_unPickTicks = 15;
   UInt32 m_unDropTicks = 15;
   UInt32 m_unIdleRestTimeout = 100;  /* idle > 100 tick -> về dock ngủ đông */

   /* ----------------------- Định danh ----------------------- */
   UInt8  m_unId = 0;

   /* ----------------------- Định vị ----------------------- */
   CVector2  m_cEstPos;
   CRadians  m_cEstYaw;
   bool      m_bPoseInit = false;
   Real      m_fAxisM    = 0.14;
   Real      m_fOdoSinceFix = 0.0;
   SGridCell m_sLastSnapCell;
   UInt32    m_unSnapCount  = 0;
   UInt32    m_unRelocCount = 0;

   /* ----------------------- Pin ----------------------- */
   Real m_fBattery = 1.0;

   /* ----------------------- Nhiệm vụ / FSM ----------------------- */
   EState m_eState = STATE_RESTING;
   STask  m_sTask;
   UInt32 m_unActionTimer = 0;
   UInt32 m_unIdleTicks   = 0;    /* đếm liên tục ở STATE_IDLE cho ngưỡng 100 tick */
   SInt32 m_nTargetDock   = -1;
   bool   m_bFirstStep    = true;

   /* ----------------------- Điều hướng ----------------------- */
   SGridCell m_sCurCell;
   SGridCell m_sPrevCell;
   SGridCell m_sGoalCell;
   bool      m_bHaveGoal    = false;
   bool      m_bGoalReached = false;
   std::vector<SGridCell> m_vecPath;
   UInt32    m_unPathIdx    = 0;
   SGridCell m_sPathGoal;

   /* ----------------------- Giao thông cục bộ ----------------------- */
   std::vector<SNeighbor> m_vecNeighbors;
   ETrafficPhase m_eTraffic = TRAFFIC_NONE;
   std::vector<SGridCell> m_vecDetourPath;  /* đúng 3 ô: rẽ/song song/rẽ về */
   UInt32      m_unDetourIdx  = 0;
   UInt32      m_unYieldTimer = 0;
   UInt32      m_unBlockedTicks = 0;
   UInt32      m_unDetourCount  = 0;

   /* Hằng số thời gian đặt chỗ (tick). Ô 0.2m @ 10 cm/s ~ 20 tick/ô
    * (ticks_per_second=10) -> cửa sổ đặt trước phải bao trọn 1 ô. */
   static constexpr UInt32 RES_KEEPALIVE = 15;
   static constexpr UInt32 RES_AHEAD     = 25;
   static constexpr UInt32 RES_GRACE     = 4;
};

}  /* namespace argos */

#endif
