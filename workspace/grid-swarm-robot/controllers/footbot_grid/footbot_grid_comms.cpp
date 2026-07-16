/**
 * footbot_grid_comms.cpp — Đóng/mở gói bản tin RAB 16 byte (layout:
 * common/grid_protocol.h). Mỗi tick robot phát: id, mức ưu tiên, ô
 * hiện tại, ô sắp bước vào, cờ chở hàng/nhường, %pin — đủ để mọi hàng
 * xóm tự phân xử tranh chấp không cần trọng tài trung tâm.
 */

#include "footbot_grid.h"

namespace argos {

/****************************************/
/****************************************/

SGridCell CFootBotGrid::PlannedNextCell() const {
   /* Bị operator đóng băng: dù lộ trình còn dang dở, với cả đàn robot
    * này ĐANG ĐỨNG YÊN — phát Next hợp lệ sẽ khiến hàng xóm nhường
    * đường mãi cho một xe không bao giờ bước tới. */
   if(m_eOverride == OP_STOPPED) return SGridCell();
   if(m_eTraffic == TRAFFIC_DETOURING && m_unDetourIdx < m_vecDetourPath.size())
      return m_vecDetourPath[m_unDetourIdx];
   if(m_eTraffic == TRAFFIC_YIELDING) return SGridCell();
   if(m_bHaveGoal && !m_bGoalReached && m_unPathIdx < m_vecPath.size())
      return m_vecPath[m_unPathIdx];
   return SGridCell();
}

/****************************************/
/****************************************/

void CFootBotGrid::ParseNeighbors() {
   m_vecNeighbors.clear();
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   for(size_t i = 0; i < tMsgs.size(); ++i) {
      const CCI_RangeAndBearingSensor::SPacket& sPkt = tMsgs[i];
      if(sPkt.Data.Size() < RAB_MSG_SIZE) continue;
      SNeighbor sN;
      sN.Id = sPkt.Data[RAB_IDX_ID];
      if(sN.Id == m_unId || sN.Id >= NUM_DOCKS) continue;
      sN.Prio  = sPkt.Data[RAB_IDX_PRIO];
      sN.State = sPkt.Data[RAB_IDX_STATE];
      sN.Flags = sPkt.Data[RAB_IDX_FLAGS];
      sN.Batt  = sPkt.Data[RAB_IDX_BATT];
      sN.Cur   = SGridCell(sPkt.Data[RAB_IDX_CUR_R], sPkt.Data[RAB_IDX_CUR_C]);
      if(sPkt.Data[RAB_IDX_NEXT_R] != RAB_NO_CELL)
         sN.Next = SGridCell(sPkt.Data[RAB_IDX_NEXT_R], sPkt.Data[RAB_IDX_NEXT_C]);
      sN.Range = sPkt.Range * 0.01;   /* cm -> m */
      m_vecNeighbors.push_back(sN);
   }
}

/****************************************/
/****************************************/

void CFootBotGrid::BroadcastState() {
   SGridCell sNext = PlannedNextCell();
   m_pcRABAct->SetData(RAB_IDX_ID,    m_unId);
   m_pcRABAct->SetData(RAB_IDX_PRIO,  GetPriority());
   m_pcRABAct->SetData(RAB_IDX_STATE, static_cast<UInt8>(m_eState));
   m_pcRABAct->SetData(RAB_IDX_CUR_R, static_cast<UInt8>(m_sCurCell.Row));
   m_pcRABAct->SetData(RAB_IDX_CUR_C, static_cast<UInt8>(m_sCurCell.Col));
   m_pcRABAct->SetData(RAB_IDX_NEXT_R,
      sNext.IsValid() ? static_cast<UInt8>(sNext.Row) : RAB_NO_CELL);
   m_pcRABAct->SetData(RAB_IDX_NEXT_C,
      sNext.IsValid() ? static_cast<UInt8>(sNext.Col) : RAB_NO_CELL);
   UInt8 unFlags = 0;
   if(m_sTask.HasBox)              unFlags |= RAB_FLAG_CARRY;
   if(m_eTraffic != TRAFFIC_NONE)  unFlags |= RAB_FLAG_YIELD;
   m_pcRABAct->SetData(RAB_IDX_FLAGS, unFlags);
   m_pcRABAct->SetData(RAB_IDX_BATT,
      static_cast<UInt8>(Max<Real>(0.0, Min<Real>(1.0, m_fBattery)) * 100.0));
}

}  /* namespace argos */
