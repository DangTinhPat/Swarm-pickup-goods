/**
 * grid_energy_safety.cpp — Pin định lượng theo vị trí/chuyển động và
 * giám sát khoảng cách thân giữa mọi cặp robot.
 */

#include "grid_loop_functions.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

/****************************************/
/****************************************/

void CGridLoopFunctions::UpdateEnergyAndOdometry() {
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      const CVector3& cP3 =
         m_vecBots[i]->GetEmbodiedEntity().GetOriginAnchor().Position;
      CVector2 cP(cP3.GetX(), cP3.GetY());
      const Real fStep = (cP - m_vecLastPos[i]).Length();
      m_vecDistance[i] += fStep;
      m_vecLastPos[i]   = cP;

      CBatteryEquippedEntity& cBatt =
         m_vecBots[i]->GetBatterySensorEquippedEntity();
      const bool bAtDock =
         CellTypeOf(WorldXToRow(cP.GetX()), WorldYToCol(cP.GetY())) == CELL_DOCK;
      const bool bMoving = fStep > MOVE_EPSILON;

      if(bAtDock && !bMoving) {
         /* Đứng yên trong ô dock -> nạp (dock ẩn danh, không cần bắt tay) */
         Real fDelta = m_fChargingFactor * BASE_CHARGE_RATE;
         cBatt.SetAvailableCharge(
            Min(cBatt.GetFullCharge(), cBatt.GetAvailableCharge() + fDelta));
      }
      else if(bMoving) {
         Real fDelta = m_fDischargingFactor * BASE_DISCHARGE_RATE;
         cBatt.SetAvailableCharge(Max<Real>(0.0, cBatt.GetAvailableCharge() - fDelta));
      }
      /* Đứng yên ngoài dock (chờ giao thông/bốc-hạ): trung tính */
   }
}

/****************************************/
/****************************************/

void CGridLoopFunctions::MonitorProximity() {
   const UInt32 unTick = Tick();
   for(size_t i = 0; i < m_vecBots.size(); ++i) {
      if(m_vecBots[i] == nullptr) continue;
      for(size_t j = i + 1; j < m_vecBots.size(); ++j) {
         if(m_vecBots[j] == nullptr) continue;
         Real fD = (m_vecLastPos[i] - m_vecLastPos[j]).Length();
         if(fD < 0.17 && !m_arrConflictLatch[i][j]) {
            ++m_unHardCollisions;
            m_arrConflictLatch[i][j] = true;
            LOGERR << "[grid-swarm] t=" << unTick << " VA CHAM THAN fb" << i
                   << "-fb" << j << " d=" << fD << " m" << std::endl;
         }
         /* Ngưỡng "gần chạm" 0.19 (không phải 0.20): khoảng cách dock
          * liền kề đúng bằng 0.20 m, sai số dấu phẩy động sẽ báo khống
          * hàng xóm dock đang đứng yên nếu để ngưỡng trùng 0.20. */
         else if(fD < 0.19) {
            ++m_unNearMisses;
         }
         else if(fD > 0.22) {
            m_arrConflictLatch[i][j] = false;
         }
      }
   }
}

}  /* namespace argos */
