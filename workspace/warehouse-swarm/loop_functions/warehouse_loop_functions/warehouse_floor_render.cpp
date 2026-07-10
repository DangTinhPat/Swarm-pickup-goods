/*
 * Floor painting: dock/charging bay pads (with the stigmergy tint) and
 * the address-zone color legend. Purely rendering — reads state, writes
 * nothing.
 */
#include "warehouse_loop_functions.h"

/****************************************/
/****************************************/

CColor CWarehouseLoopFunctions::AddressColor(UInt32 un_addr) {
   switch(un_addr) {
      case 0:  return CColor::RED;      /* A */
      case 1:  return CColor::GREEN;    /* B */
      case 2:  return CColor::BLUE;     /* C */
      case 3:  return CColor::YELLOW;   /* D */
      default: return CColor::MAGENTA;  /* E */
   }
}

/****************************************/
/****************************************/

CColor CWarehouseLoopFunctions::GetFloorColor(const CVector2& c_pos) {
   /* Charging bays: darker square per slot — GREEN while it is
    * actively charging a robot; a light pad tile around each slot */
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(Abs(c_pos.GetX() - m_cDockSlots[s].GetX()) < 0.14 &&
         Abs(c_pos.GetY() - m_cDockSlots[s].GetY()) < 0.14) {
         if(m_unSlotStatus[s] == 2) return CColor(60, 200, 90);    /* charging */
         if(m_unSlotStatus[s] == 1) return CColor(235, 180, 60);    /* warming up */
         return CColor::GRAY50;
      }
   }
   for(size_t s = 0; s < m_cDockSlots.size(); ++s) {
      if(Abs(c_pos.GetX() - m_cDockSlots[s].GetX()) < 0.45 &&
         Abs(c_pos.GetY() - m_cDockSlots[s].GetY()) < 0.45) {
         /* Stigmergy: tint the side's approach area darker the more recent
          * arrival activity it has seen — a real fading trace the robots'
          * ground sensor reads when nearby, capped so it stays a "floor". */
         UInt32 unSide = SlotIsLeftSide(s) ? 0 : 1;
         Real fDarkness = Min<Real>(1.0, m_fSideActivity[unSide] / 6.0);
         UInt8 unLevel = (UInt8)(200 - 90 * fDarkness);
         return CColor(unLevel, unLevel, unLevel);
      }
   }
   /* Address zones: light shade of the address color */
   for(UInt32 a = 0; a < NUM_ADDRS; ++a) {
      if(Abs(c_pos.GetX() - m_cAddrPos[a].GetX()) < m_fZoneHalf &&
         Abs(c_pos.GetY() - m_cAddrPos[a].GetY()) < m_fZoneHalf) {
         CColor cFull = AddressColor(a);
         return CColor((cFull.GetRed() + 2 * 255) / 3,
                       (cFull.GetGreen() + 2 * 255) / 3,
                       (cFull.GetBlue() + 2 * 255) / 3);
      }
   }
   return CColor::WHITE;
}
