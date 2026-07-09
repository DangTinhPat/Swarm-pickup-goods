/*
 * Collision / wall-clearance metrics (same convention as ball-collector):
 * body radius 0.085 m. Energy metrics (m_fMinChargeSeen/m_unDeadTicks)
 * are tracked in warehouse_robot_update.cpp instead, right where each
 * robot's battery is already fetched for the docking/charging pass.
 */
#include "warehouse_loop_functions.h"

/****************************************/
/****************************************/

void CWarehouseLoopFunctions::UpdateCollisionMetrics(const std::vector<CVector2>& c_positions) {
   for(size_t i = 0; i < c_positions.size(); ++i) {
      Real fWall = 3.95 - Max(Abs(c_positions[i].GetX()),
                              Abs(c_positions[i].GetY()));
      if(fWall < m_fMinWallClearance) {
         m_fMinWallClearance = fWall;
      }
      for(size_t j = i + 1; j < c_positions.size(); ++j) {
         Real fDist = (c_positions[i] - c_positions[j]).Length();
         if(fDist < 0.181) {
            ++m_unCollisionTicks;
         }
         if(fDist < m_fMinPairDistance) {
            m_fMinPairDistance = fDist;
         }
      }
   }
}
