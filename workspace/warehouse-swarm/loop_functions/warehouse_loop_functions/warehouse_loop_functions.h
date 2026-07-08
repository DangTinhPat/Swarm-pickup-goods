/*
 * Loop functions for the warehouse experiment — the "physical facility":
 *
 * - 3 conveyor belts continuously drop parcels into their bins (FIFO
 *   queues). Each parcel carries a random destination address a..e.
 * - Handover: when an empty-handed robot is within pickup_radius of a
 *   bin's pickup point and the bin is not empty, the front parcel is
 *   loaded onto that robot (nearest robot wins, one handover per belt
 *   per tick — like a worker handing over one box at a time).
 * - Delivery: when a loaded robot enters the zone matching its parcel's
 *   address, the parcel is accepted and counted.
 * - Facility WMS feed: every tick each robot receives the three queue
 *   lengths (public IoT information; the *decision* what to do with it
 *   stays fully decentralized in the controllers).
 * - Floor painting: depot circle + 5 colored address zones.
 * - Metrics: per-address and total deliveries, collision pair-ticks,
 *   closest robot-robot pass, closest wall approach.
 */

#ifndef WAREHOUSE_LOOP_FUNCTIONS_H
#define WAREHOUSE_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>
#include <deque>

using namespace argos;

class CWarehouseLoopFunctions : public CLoopFunctions {

public:

   static const UInt32 NUM_BELTS = 3;
   static const UInt32 NUM_ADDRS = 5;

   CWarehouseLoopFunctions();
   virtual ~CWarehouseLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PreStep();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

   /* Parcel color per address (A..E) — shared with the QT user functions */
   static CColor AddressColor(UInt32 un_addr);

   /* Read by the QT user functions for drawing */
   const std::deque<UInt8>& GetBeltQueue(UInt32 b) const { return m_cQueues[b]; }
   const CVector2& GetBeltPickup(UInt32 b) const { return m_cBeltPickup[b]; }
   const CVector2& GetAddrPos(UInt32 a) const { return m_cAddrPos[a]; }
   const CVector2& GetDockCenter() const { return m_cDockCenter; }
   UInt32 GetDelivered() const { return m_unDelivered; }
   const UInt32* GetDeliveredPerAddr() const { return m_unDeliveredPerAddr; }

private:

   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   /* Layout */
   CVector2 m_cBeltPickup[NUM_BELTS];
   CVector2 m_cAddrPos[NUM_ADDRS];
   /* Docking grid (matches the controller's layout) */
   CVector2 m_cDockCenter;
   std::vector<CVector2> m_cDockSlots;
   Real     m_fDockSpacing;
   UInt32   m_unDockRows;
   UInt32   m_unDockCols;
   Real     m_fZoneHalf;      /* address zones are squares of this half-size */

   /* Parcel flow */
   UInt32 m_unSpawnPeriod;    /* ticks between parcel arrivals */
   UInt32 m_unQueueCap;       /* bin capacity per belt */
   Real   m_fPickupRadius;    /* handover distance at the bin */

   std::deque<UInt8> m_cQueues[NUM_BELTS];

   /* Metrics */
   UInt32 m_unDelivered;
   UInt32 m_unDeliveredPerAddr[NUM_ADDRS];
   UInt32 m_unCollisionTicks;
   Real   m_fMinPairDistance;
   Real   m_fMinWallClearance;
};

#endif
