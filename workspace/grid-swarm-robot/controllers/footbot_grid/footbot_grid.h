/**
 * footbot_grid.h
 *
 * BỘ ĐIỀU KHIỂN PHI TẬP TRUNG cho một AMR (foot-bot vi sai) trong nhà
 * kho lưới 20x20. Mỗi robot tự mình:
 *
 *   1. ĐỊNH VỊ: tích phân odometry từ encoder 2 bánh (có trôi), và XÓA
 *      SẠCH sai số mỗi lần cảm biến sàn bắt được đĩa đen ở hồng tâm ô
 *      (mô phỏng camera gầm đọc mã QR dán sàn) -> footbot_grid_nav.cpp
 *   2. QUYẾT ĐỊNH: máy trạng thái nhận việc / lấy hàng / giao hàng /
 *      về dock nghỉ / sạc khẩn cấp theo mô hình pin trễ (hysteresis
 *      20% - 70%) -> footbot_grid.cpp
 *   3. GIAO THÔNG: đặt chỗ ô kế tiếp trên bảng chiếm dụng chia sẻ,
 *      trao đổi mức ưu tiên qua RAB và tự nhường đường (đứng im hoặc
 *      dạt sang làn bên trong hành lang 2 ô) -> footbot_grid_traffic.cpp
 *
 * Loop Functions KHÔNG ra lệnh cho robot — nó chỉ là "bảng đen" ghi
 * nhận đặt chỗ và trạng thái hàng hóa. Mọi quyết định nằm ở đây.
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

class CGridLoopFunctions;   /* bảng đen chia sẻ (khai báo trước)  */

class CFootBotGrid : public CCI_Controller {

public:

   /* ---------------- Máy trạng thái nhiệm vụ ---------------- */
   enum EState : UInt8 {
      STATE_CHARGING = 0,   /* đứng ở dock, đang sạc (khóa tới >= 70%)   */
      STATE_IDLE_DOCK,      /* đậu ở dock, đủ pin, chờ có việc           */
      STATE_TO_PICKUP,      /* chạy rỗng tới băng chuyền đã nhận         */
      STATE_PICKING,        /* đứng ở băng chuyền, bốc hộp lên           */
      STATE_TO_DELIVER,     /* CHỞ HÀNG tới ô ngăn xếp (ưu tiên 2)       */
      STATE_DELIVERING,     /* đứng ở ô ngăn xếp, hạ hộp xuống           */
      STATE_TO_REST,        /* hết việc, chạy về dock trống gần nhất     */
      STATE_TO_CHARGE       /* KHẨN CẤP pin < 20%, chạy về dock (ưu tiên 1) */
   };

   /* ------------- Pha nhường đường (chồng lên FSM) ------------- */
   enum EYieldPhase : UInt8 {
      YIELD_NONE = 0,       /* lưu thông bình thường                     */
      YIELD_SIDESTEP,       /* đang dạt sang ô bên cạnh để nhường làn    */
      YIELD_WAIT_CLEAR      /* đã dạt xong, đứng im chờ robot ưu tiên qua */
   };

   /* Nhiệm vụ đã nhận: cặp (băng chuyền có hộp, ô ngăn xếp cần màu đó) */
   struct STask {
      SInt32 ConveyorIdx = -1;
      SInt32 DemandIdx   = -1;
      UInt8  Color       = 0;
      bool   HasBox      = false;  /* true = hộp đã nằm trên lưng robot */
      bool   IsValid() const { return DemandIdx >= 0; }
      void   Clear() { ConveyorIdx = -1; DemandIdx = -1; Color = 0; HasBox = false; }
   };

   /* Bản tin RAB đã giải mã của một robot hàng xóm */
   struct SNeighbor {
      UInt8     Id       = 255;
      UInt8     Prio     = PRIO_IDLE;
      UInt8     State    = 0;
      UInt8     Flags    = 0;
      UInt8     Batt     = 0;
      SGridCell Cur;
      SGridCell Next;          /* .IsValid()==false nếu đang đứng yên   */
      Real      Range    = 0.0; /* khoảng cách [m]                       */
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
   bool        IsYielding()      const { return m_eYield != YIELD_NONE; }
   UInt32      GetSnapCount()    const { return m_unSnapCount; }
   UInt32      GetRelocCount()   const { return m_unRelocCount; }
   UInt32      GetSidestepCount() const { return m_unSidestepCount; }
   const SGridCell& GetCurCell() const { return m_sCurCell; }
   const std::vector<SGridCell>& GetPath() const { return m_vecPath; }

private:

   /* ============ footbot_grid.cpp : FSM + nhiệm vụ + pin ============ */
   void RunStateMachine();
   void CheckBatteryEmergency();
   bool TryClaimBestTask();          /* robot TỰ chọn việc, bảng đen chỉ ghi nhận */
   void AfterTaskDone();             /* tìm việc mới hoặc về dock nghỉ    */
   bool RetargetDock(bool b_must_find);
   void SetGoal(const SGridCell& s_cell);
   void UpdateLed();

   /* ============ footbot_grid_nav.cpp : định vị + A* + bám tâm ô ==== */
   void UpdateLocalization();        /* odometry + chốt lại bằng QR sàn  */
   bool PlanPath(const SGridCell& s_goal, UInt32 un_avoid_level);
   void ApplySteering(const CVector2& c_target, bool b_final);
   void StopWheels();

   /* ============ footbot_grid_traffic.cpp : đặt chỗ + nhường đường == */
   void StepMovement();              /* vòng ngoài: đi theo path đã đặt chỗ */
   void ParseNeighbors();
   void BroadcastState();
   bool IWinAgainst(const SNeighbor& s_n) const;
   bool TrySidestep(const SNeighbor& s_opp, const SGridCell& s_blocked);
   void HandleYieldPhase();
   void KeepAliveReservation();
   SGridCell PlannedNextCell() const;

   /* Bảng đen chia sẻ — lấy trễ ở tick đầu (LF khởi tạo sau controller) */
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
   Real m_fMaxSpeed    = 25.0;  /* tốc độ hành trình [cm/s]              */
   Real m_fTurnSpeed   = 12.0;  /* tốc độ bánh khi xoay tại chỗ [cm/s]   */
   Real m_fKpHeading   = 3.5;   /* hệ số P bẻ lái theo sai số góc        */
   CRadians m_cHardTurn;        /* ngưỡng xoay tại chỗ (mặc định 65 độ)  */
   Real m_fWaypointTol = 0.08;  /* [m] coi như đã chạm tâm ô trung gian  */
   Real m_fGoalTol     = 0.05;  /* [m] dừng hẳn tại tâm ô đích           */
   Real m_fLowBatt     = 0.20;  /* ngưỡng xả cứng: bỏ việc đi sạc        */
   Real m_fLeaveBatt   = 0.70;  /* ngưỡng sạc cứng: đủ mới được rời dock */
   UInt32 m_unPickTicks = 15;   /* thời gian bốc hộp (tick)              */
   UInt32 m_unDropTicks = 15;   /* thời gian hạ hộp (tick)               */

   /* ----------------------- Định danh ----------------------- */
   UInt8  m_unId = 0;           /* rút từ "fbN"                          */

   /* ----------------------- Định vị ----------------------- */
   CVector2  m_cEstPos;         /* ước lượng (x,y) từ odometry [m]       */
   CRadians  m_cEstYaw;         /* ước lượng góc hướng                   */
   bool      m_bPoseInit = false;
   Real      m_fAxisM    = 0.14;      /* khoảng cách 2 bánh [m]          */
   Real      m_fOdoSinceFix = 0.0;    /* quãng đường từ lần chốt QR cuối */
   SGridCell m_sLastSnapCell;         /* chống chốt lặp trong cùng một ô */
   UInt32    m_unSnapCount  = 0;      /* số lần xóa trôi nhờ QR sàn      */
   UInt32    m_unRelocCount = 0;      /* số lần cứu hộ vì lạc > 3 m      */

   /* ----------------------- Pin ----------------------- */
   Real m_fBattery = 1.0;       /* phân số 0..1 từ cảm biến pin          */

   /* ----------------------- Nhiệm vụ / FSM ----------------------- */
   EState m_eState = STATE_CHARGING;
   STask  m_sTask;
   UInt32 m_unActionTimer = 0;  /* đếm lùi bốc/hạ hộp                    */
   SInt32 m_nTargetDock   = -1; /* dock đang nhắm về (nghỉ hoặc sạc)     */
   bool   m_bFirstStep    = true;

   /* ----------------------- Điều hướng ----------------------- */
   SGridCell m_sCurCell;        /* ô "logic" robot đang sở hữu           */
   SGridCell m_sPrevCell;       /* ô vừa rời (ứng viên lùi khi kẹt)      */
   SGridCell m_sGoalCell;
   bool      m_bHaveGoal    = false;
   bool      m_bGoalReached = false;
   std::vector<SGridCell> m_vecPath;  /* chuỗi ô từ sau ô hiện tại tới đích */
   UInt32    m_unPathIdx    = 0;
   SGridCell m_sPathGoal;             /* đích mà path hiện có phục vụ     */

   /* ----------------------- Giao thông ----------------------- */
   std::vector<SNeighbor> m_vecNeighbors;
   EYieldPhase m_eYield = YIELD_NONE;
   SGridCell   m_sSideCell;       /* ô đang dạt sang                     */
   SGridCell   m_sYieldBlocked;   /* ô tranh chấp đã nhường              */
   UInt8       m_unYieldOppId = 255;
   UInt32      m_unYieldTimer = 0;
   UInt32      m_unBlockedTicks = 0;  /* leo thang: chờ -> né -> vẽ lại đường */
   UInt32      m_unSideCooldown = 0;
   UInt32      m_unSidestepCount = 0; /* số lần dạt làn nhường robot ưu tiên */

   /* Hằng số thời gian đặt chỗ (tick): ô 1 m @ 25 cm/s ~ 40 tick        */
   static constexpr UInt32 RES_KEEPALIVE = 60;   /* gia hạn ô đang đứng  */
   static constexpr UInt32 RES_AHEAD     = 110;  /* đặt ô sắp bước vào   */
   static constexpr UInt32 RES_GRACE     = 12;   /* giữ ô vừa rời thêm   */
};

}  /* namespace argos */

#endif
