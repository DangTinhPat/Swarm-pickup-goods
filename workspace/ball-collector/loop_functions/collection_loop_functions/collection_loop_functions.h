/*
 * Loop functions for the ball-collection experiment.
 *
 * Responsibilities:
 * - Spawn balls at random free positions, continuously (every
 *   spawn_period ticks, up to max_balls on the ground at once).
 * - Non-physical attachment: when a robot gets within pickup_radius of a
 *   ball, the ball is removed from the ground and the robot is flagged
 *   as carrying (the ball is drawn on top of it by the QT user functions).
 * - Deposit: when a carrying robot enters the nest circle, the ball is
 *   dropped and the score increases.
 * - Virtual ball sensor: each tick, every free robot is told the position
 *   of the nearest free ball within sight_range (if any).
 * - Floor painting: the nest is drawn as a gray disk on the floor.
 */

#ifndef COLLECTION_LOOP_FUNCTIONS_H
#define COLLECTION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CCollectionLoopFunctions : public CLoopFunctions {

public:

   CCollectionLoopFunctions();
   virtual ~CCollectionLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PreStep();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

   /* Read by the QT user functions for drawing */
   const std::vector<CVector2>& GetBalls() const { return m_cBalls; }
   const CVector2& GetNestPos() const { return m_cNestPos; }
   Real GetNestRadius() const { return m_fNestRadius; }
   UInt32 GetScore() const { return m_unScore; }

private:

   CVector2 RandomBallPosition();

   std::vector<CVector2> m_cBalls;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   /* Parameters from the XML configuration */
   UInt32 m_unSpawnPeriod;   /* ticks between spawns */
   UInt32 m_unMaxBalls;      /* max balls on the ground at once */
   Real   m_fPickupRadius;   /* robot-to-ball distance that counts as touching */
   Real   m_fSightRange;     /* virtual ball sensor range */
   CVector2 m_cNestPos;
   Real   m_fNestRadius;
   CRange<Real> m_cSpawnRangeX;
   CRange<Real> m_cSpawnRangeY;

   UInt32 m_unScore;
   /* Collision metrics: pair-ticks in body contact (centers < 18.1 cm;
    * body radius 8.5 cm -> touch at 17.1 cm) + closest pass ever seen */
   UInt32 m_unCollisionTicks;
   Real   m_fMinPairDistance;
   /* Closest robot-center-to-wall-face distance seen (walls at +/-4,
    * 0.05 half-thickness -> inner face 3.95; body touches at 0.085) */
   Real   m_fMinWallClearance;
};

#endif
