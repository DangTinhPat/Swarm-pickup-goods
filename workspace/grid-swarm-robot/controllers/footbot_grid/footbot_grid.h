/**
 * footbot_grid.h — Controller phi tập trung cho một AMR foot-bot trên
 * lưới kho 30x30 (ô 0.2 m). Mọi quyết định (nhận việc, đường đi, nhường
 * đường, năng lượng) nằm trong controller; Loop Functions chỉ là bảng
 * đen ghi nhận, không ra lệnh.
 *
 * Hiện thực chia theo mô-đun chức năng:
 *   footbot_grid.cpp               vòng đời ControlStep, LED, tiện ích
 *   footbot_grid_fsm.cpp           FSM nhiệm vụ + chính sách pin 20%/70%
 *   footbot_grid_localization.cpp  odometry + chốt QR sàn (khử drift)
 *   footbot_grid_pathfinding.cpp   A* 4 hướng, phạt rẽ, cấm vật cản
 *   footbot_grid_steering.cpp      P-controller vi sai bám tâm ô
 *   footbot_grid_traffic.cpp       đặt chỗ ô + phân xử ưu tiên
 *   footbot_grid_detour.cpp        dạt làn cục bộ 3 bước / đứng nhường
 *   footbot_grid_comms.cpp         đóng/mở gói bản tin RAB
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

#include <common/grid_layout.h>

#include <vector>
#include <string>

namespace argos {

class CGridLoopFunctions;

class CFootBotGrid : public CCI_Controller {

public:

   enum EState : UInt8 {
      STATE_IDLE = 0,          /* đậu ở dock, đủ pin, chờ việc            */
      STATE_TO_PICKUP,         /* chạy rỗng tới băng chuyền đã nhận       */
      STATE_PICKING,           /* bốc hộp tại băng chuyền                 */
      STATE_DELIVERING,        /* chở hàng tới mặt kệ (ưu tiên 2)         */
      STATE_DROPPING,          /* hạ hộp tại mặt kệ                       */
      STATE_RETURNING,         /* hết việc, về dock trống gần nhất        */
      STATE_RESTING,           /* ngủ đông tại dock, sạc thụ động         */
      STATE_EMERGENCY_CHARGE   /* pin<=20%: bỏ việc về sạc (ưu tiên 1)    */
   };

   enum ETrafficPhase : UInt8 {
      TRAFFIC_NONE = 0,
      TRAFFIC_DETOURING,       /* đang chạy 3 bước dạt làn                */
      TRAFFIC_YIELDING         /* hai phía đều nghẽn: đứng im 1-2 tick    */
   };

   /** Lớp lệnh của NGƯỜI VẬN HÀNH (bảng điều khiển trên GUI) đè lên tự
    * hành — như e-stop/recall của fleet-manager thật, KHÔNG phải điều
    * phối trung tâm: dưới OP_AUTO (mặc định, headless chỉ có chế độ này)
    * robot tự hành 100%. */
   enum EOverride : UInt8 {
      OP_AUTO = 0,   /* tự hành hoàn toàn                                 */
      OP_STOPPED,    /* e-stop: đóng băng tại chỗ nhưng VẪN gia hạn đặt
                      * chỗ ô + phát RAB để cả đàn né như vật cản đứng    */
      OP_RECALL      /* giao nốt hộp đang ôm rồi về dock và Ở LẠI đó      */
   };

   /** Nhiệm vụ: cặp (hộp trên băng chuyền, ô kệ cần đúng màu đó). */
   struct STask {
      SInt32 ConveyorIdx = -1;
      SInt32 DemandIdx   = -1;
      UInt8  Color       = 0;
      bool   HasBox      = false;
      bool   FarSide     = false;   /* mặt kệ đang nhắm: Row-1 hay Row+1  */
      bool   IsValid() const { return DemandIdx >= 0; }
      void   Clear() { ConveyorIdx = -1; DemandIdx = -1; Color = 0; HasBox = false; FarSide = false; }
   };

   /** Bản tin RAB đã giải mã của một robot lân cận. */
   struct SNeighbor {
      UInt8     Id    = 255;
      UInt8     Prio  = PRIO_IDLE;
      UInt8     State = 0;
      UInt8     Flags = 0;
      UInt8     Batt  = 0;
      SGridCell Cur;
      SGridCell Next;             /* !IsValid() nếu đang đứng yên         */
      Real      Range = 0.0;
   };

public:

   CFootBotGrid();
   virtual ~CFootBotGrid() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

   /* --- API chỉ-đọc cho Loop Functions / GUI --- */
   UInt8       GetRobotId()       const { return m_unId; }
   EState      GetState()         const { return m_eState; }
   const char* GetStateName()     const;
   UInt8       GetPriority()      const;
   bool        IsCarrying()       const { return m_sTask.HasBox; }
   UInt8       GetCarriedColor()  const { return m_sTask.Color; }
   Real        GetBatteryFrac()   const { return m_fBattery; }
   bool        IsInTraffic()      const { return m_eTraffic != TRAFFIC_NONE; }
   UInt32      GetSnapCount()     const { return m_unSnapCount; }
   UInt32      GetRelocCount()    const { return m_unRelocCount; }
   UInt32      GetDetourCount()   const { return m_unDetourCount; }
   const SGridCell& GetCurCell()  const { return m_sCurCell; }
   const std::vector<SGridCell>& GetPath() const { return m_vecPath; }

   /* --- Bảng điều khiển vận hành (grid_qt_user_functions.cpp) --- */
   void      SetOverride(EOverride e_op);
   EOverride GetOverride() const { return m_eOverride; }

private:

   /* footbot_grid_fsm.cpp */
   void RunStateMachine();
   void CheckBatteryEmergency();
   void CheckOperatorRecall();
   bool TryClaimBestTask();
   void AfterTaskDone();
   bool RetargetDock(bool b_must_find);
   void SetGoal(const SGridCell& s_cell);

   /* footbot_grid_localization.cpp */
   void UpdateLocalization();

   /* footbot_grid_pathfinding.cpp */
   bool PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level);

   /* footbot_grid_steering.cpp */
   void ApplySteering(const CVector2& c_target, bool b_final);
   void StopWheels();

   /* footbot_grid_traffic.cpp */
   void StepMovement();
   bool IWinAgainst(const SNeighbor& s_n) const;
   void KeepAliveReservation();

   /* footbot_grid_detour.cpp */
   bool TryStartDetour(const SGridCell& s_blocked);
   void HandleDetourPhase();
   void HandleYieldPhase();

   /* footbot_grid_comms.cpp */
   void ParseNeighbors();
   void BroadcastState();
   SGridCell PlannedNextCell() const;

   /* footbot_grid.cpp — LF khởi tạo sau controller nên lấy trễ từ tick đầu */
   CGridLoopFunctions& LF();
   void UpdateLed();

private:

   /* --- Thiết bị --- */
   CCI_DifferentialSteeringActuator* m_pcWheels     = nullptr;
   CCI_DifferentialSteeringSensor*   m_pcWheelsSens = nullptr;
   CCI_RangeAndBearingActuator*      m_pcRABAct     = nullptr;
   CCI_RangeAndBearingSensor*        m_pcRABSens    = nullptr;
   CCI_PositioningSensor*            m_pcPosSens    = nullptr;
   CCI_LEDsActuator*                 m_pcLEDs       = nullptr;
   CCI_BatterySensor*                m_pcBattery    = nullptr;
   CCI_FootBotMotorGroundSensor*     m_pcGround     = nullptr;

   CGridLoopFunctions*               m_pcLF         = nullptr;

   /* --- Tham số XML --- */
   Real m_fCruiseSpeed = 10.0;    /* V_Base chạy thẳng [cm/s]             */
   Real m_fPivotSpeed  = 3.0;     /* V_Base xoay 90 độ tại tâm ô [cm/s]   */
   Real m_fKpHeading   = 3.5;
   CRadians m_cHardTurn;          /* ngưỡng chuyển sang xoay tại chỗ      */
   Real m_fWaypointTol = 0.02;    /* [m] chạm tâm ô trung gian            */
   Real m_fGoalTol     = 0.02;    /* [m] dừng hẳn tại tâm ô đích          */
   Real m_fQrSnapRadius = 0.02;   /* [m] ngưỡng tin cậy trước khi chốt QR */
   Real m_fLowBatt     = 0.20;    /* ngưỡng khẩn cấp                      */
   Real m_fLeaveBatt   = 0.70;    /* ngưỡng được rời dock                 */
   UInt32 m_unPickTicks = 15;
   UInt32 m_unDropTicks = 15;
   UInt32 m_unIdleRestTimeout = 100;

   /* --- Định danh --- */
   UInt8  m_unId = 0;

   /* --- Định vị --- */
   CVector2  m_cEstPos;
   CRadians  m_cEstYaw;
   bool      m_bPoseInit = false;
   Real      m_fAxisM    = 0.14;
   Real      m_fOdoSinceFix = 0.0;
   SGridCell m_sLastSnapCell;
   UInt32    m_unSnapCount  = 0;
   UInt32    m_unRelocCount = 0;

   /* --- Pin --- */
   Real m_fBattery = 1.0;

   /* --- Nhiệm vụ / FSM --- */
   EState m_eState = STATE_RESTING;
   EOverride m_eOverride = OP_AUTO;   /* lệnh operator, xem enum trên */
   STask  m_sTask;
   UInt32 m_unActionTimer = 0;
   UInt32 m_unIdleTicks   = 0;
   SInt32 m_nTargetDock   = -1;
   bool   m_bFirstStep    = true;

   /* --- Điều hướng --- */
   SGridCell m_sCurCell;
   SGridCell m_sPrevCell;
   SGridCell m_sGoalCell;
   bool      m_bHaveGoal    = false;
   bool      m_bGoalReached = false;
   std::vector<SGridCell> m_vecPath;
   UInt32    m_unPathIdx    = 0;
   SGridCell m_sPathGoal;

   /* --- Giao thông cục bộ --- */
   std::vector<SNeighbor> m_vecNeighbors;
   ETrafficPhase m_eTraffic = TRAFFIC_NONE;
   std::vector<SGridCell> m_vecDetourPath;
   UInt32      m_unDetourIdx    = 0;
   UInt32      m_unYieldTimer   = 0;
   UInt32      m_unBlockedTicks = 0;
   UInt32      m_unDetourCount  = 0;

   /* Cửa sổ đặt chỗ [tick]: ô 0.2m @ 10cm/s ~ 20 tick/ô nên RES_AHEAD
    * phải bao trọn hơn 1 ô; RES_GRACE giữ ô vừa rời vì đuôi xe còn vắt. */
   static constexpr UInt32 RES_KEEPALIVE = 15;
   static constexpr UInt32 RES_AHEAD     = 25;
   static constexpr UInt32 RES_GRACE     = 4;
};

}  /* namespace argos */

#endif
