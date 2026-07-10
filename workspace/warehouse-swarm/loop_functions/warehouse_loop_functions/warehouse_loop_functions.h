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
 * - Belt ground truth: robots only get to SENSE (not globally read) each
 *   belt's queue length + blocked flag — see footbot_comms.cpp.
 * - Floor painting: dock/charging bays (+ stigmergy tint) and 5 colored
 *   address zones.
 * - Metrics: per-address and total deliveries, collision pair-ticks,
 *   closest robot-robot pass, closest wall approach, energy.
 *
 * PreStep() (in warehouse_loop_functions.cpp) is a thin per-tick
 * orchestrator; the actual work is split by behavior across:
 *   warehouse_loop_functions.cpp  - lifecycle (Init/Reset/Destroy) + PreStep
 *   warehouse_floor_render.cpp    - GetFloorColor / AddressColor
 *   warehouse_spawning.cpp        - parcel spawn, belt ground truth, handover
 *   warehouse_robot_update.cpp    - per-robot per-tick pass: docking/
 *                                    charging/stigmergy, energy metrics,
 *                                    delivery detection
 *   warehouse_metrics.cpp         - collision / wall-clearance metrics
 */

#ifndef WAREHOUSE_LOOP_FUNCTIONS_H
#define WAREHOUSE_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <vector>
#include <deque>
#include <map>
#include <string>

using namespace argos;

class CFootBotWarehouse;

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

   /* ==== warehouse_spawning.cpp — parcel/belt lifecycle ==== */
   void SpawnParcel();
   /* Fills per-belt queue length + whether a dead robot blocks the
    * pickup point — this is the ground truth the controllers only get
    * to SENSE locally (see footbot_comms.cpp), never read globally */
   void ComputeBeltGroundTruth(UInt8* pun_queue_lens, bool* pb_blocked);
   void HandoverAtBelts(const std::vector<CVector2>& c_positions,
                         const std::vector<CFootBotWarehouse*>& c_controllers);

   /* ==== warehouse_robot_update.cpp — per-robot per-tick pass ==== */
   /* Stigmergy trail decay, applied once per tick regardless of activity
    * (what makes it a genuine fading trace, not a permanent record) */
   void UpdateStigmergyDecay();
   /* One pass over the fleet: injects belt ground truth, handles
    * docking/charging/stigmergy-bump, tracks energy metrics, and detects
    * deliveries — bundled into a single pass (rather than 4 separate
    * fleet iterations) since all four operate on the same one robot at
    * a time with no cross-robot ordering dependency. Fills c_positions/
    * c_controllers for the callers that need them afterward (handover,
    * collision metrics). */
   void UpdateRobots(const UInt8* pun_queue_lens, const bool* pb_blocked,
                      std::vector<CVector2>& c_positions,
                      std::vector<CFootBotWarehouse*>& c_controllers);

   /* ==== warehouse_metrics.cpp — collision / wall-clearance ==== */
   void UpdateCollisionMetrics(const std::vector<CVector2>& c_positions);

   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   /* Layout */
   CVector2 m_cBeltPickup[NUM_BELTS];
   CVector2 m_cAddrPos[NUM_ADDRS];
   /* Docking/charging bays (must match the controller's layout) */
   CVector2 m_cDockCenter;
   std::vector<CVector2> m_cDockSlots;
   /* Bay status for floor painting: 0 = free/idle, 1 = robot present,
    * warming up (5 s handshake before power flows), 2 = charging */
   std::vector<UInt8> m_unSlotStatus;
   /* Consecutive on-bay ticks per robot (keyed by robot id) */
   std::map<std::string, UInt32> m_mapWarmup;
   Real     m_fZoneHalf;      /* address zones are squares of this half-size */

   /* Stigmergy: a fading trace of how much recent activity each dock
    * side (0=left/y>0, 1=right/y<0) has seen, painted as a gray tint
    * near that row and sensed by robots' real ground sensor — not a
    * number handed to the controller. Bumped when a robot newly settles
    * onto a bay on that side (edge-triggered on the false->true parked
    * transition, tracked per robot id), decayed a little every tick. */
   Real m_fSideActivity[2];
   std::map<std::string, bool> m_mapWasParked;
   Real m_fStigmergyDecay;    /* multiplicative decay per tick */
   Real m_fStigmergyGain;     /* activity added per fresh arrival */
   bool SlotIsLeftSide(size_t un_slot) const {
      return m_cDockSlots[un_slot].GetY() > 0.0;
   }

   /* Parcel flow */
   UInt32 m_unSpawnPeriod;    /* ticks between parcel arrivals */
   UInt32 m_unQueueCap;       /* bin capacity per belt */
   Real   m_fPickupRadius;    /* handover distance at the bin */

   /* Charging: dock slots double as charging bays */
   Real   m_fChargeRate;      /* charge fraction added per tick on a bay */
   UInt32 m_unChargeWarmup;   /* ticks on the bay before power flows */


   Real m_fDrainTime;         /* charge drained per tick regardless of motion */
   Real m_fDrainMove;         /* charge drained per metre travelled */
   std::map<std::string, CVector2> m_mapLastPos;  /* for per-tick step distance */

   std::deque<UInt8> m_cQueues[NUM_BELTS];

   /* Metrics */
   UInt32 m_unDelivered;
   UInt32 m_unDeliveredPerAddr[NUM_ADDRS];
   UInt32 m_unCollisionTicks;
   Real   m_fMinPairDistance;
   /* Energy metrics */
   Real   m_fMinChargeSeen;   /* lowest battery fraction ever observed */
   UInt32 m_unDeadTicks;      /* robot-ticks spent with an empty battery */
   Real   m_fMinWallClearance;
};

#endif
