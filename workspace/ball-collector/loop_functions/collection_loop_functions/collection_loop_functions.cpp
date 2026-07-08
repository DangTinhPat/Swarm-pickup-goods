#include "collection_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_collector/footbot_collector.h>

/****************************************/
/****************************************/

CCollectionLoopFunctions::CCollectionLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unSpawnPeriod(20),
   m_unMaxBalls(12),
   m_fPickupRadius(0.18),
   m_fSightRange(1.0),
   m_fNestRadius(0.45),
   m_cSpawnRangeX(-0.7, 1.7),
   m_cSpawnRangeY(-1.7, 1.7),
   m_unScore(0),
   m_unCollisionTicks(0),
   m_fMinPairDistance(1000.0),
   m_fMinWallClearance(1000.0) {
}

/****************************************/
/****************************************/

void CCollectionLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tCollection = GetNode(t_node, "collection");
      m_pcFloor = &GetSpace().GetFloorEntity();
      m_pcRNG = CRandom::CreateRNG("argos");

      GetNodeAttribute(tCollection, "spawn_period", m_unSpawnPeriod);
      GetNodeAttribute(tCollection, "max_balls", m_unMaxBalls);
      GetNodeAttribute(tCollection, "pickup_radius", m_fPickupRadius);
      GetNodeAttribute(tCollection, "sight_range", m_fSightRange);
      GetNodeAttribute(tCollection, "nest", m_cNestPos);
      GetNodeAttribute(tCollection, "nest_radius", m_fNestRadius);
      CVector2 cSpawnMin, cSpawnMax;
      GetNodeAttribute(tCollection, "spawn_min", cSpawnMin);
      GetNodeAttribute(tCollection, "spawn_max", cSpawnMax);
      m_cSpawnRangeX.Set(cSpawnMin.GetX(), cSpawnMax.GetX());
      m_cSpawnRangeY.Set(cSpawnMin.GetY(), cSpawnMax.GetY());
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing collection loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CCollectionLoopFunctions::Reset() {
   m_cBalls.clear();
   m_unScore = 0;
   m_unCollisionTicks = 0;
   m_fMinPairDistance = 1000.0;
   m_fMinWallClearance = 1000.0;
}

/****************************************/
/****************************************/

void CCollectionLoopFunctions::Destroy() {
   LOG << "[collection] Final score: " << m_unScore
       << " | collision pair-ticks: " << m_unCollisionTicks
       << " | closest pass: " << m_fMinPairDistance << " m"
       << " | closest wall: " << m_fMinWallClearance << " m" << std::endl;
   LOG.Flush();
}

/****************************************/
/****************************************/

CVector2 CCollectionLoopFunctions::RandomBallPosition() {
   /* Rejection sampling: keep balls out of the nest circle */
   CVector2 cPos;
   do {
      cPos.Set(m_pcRNG->Uniform(m_cSpawnRangeX),
               m_pcRNG->Uniform(m_cSpawnRangeY));
   } while((cPos - m_cNestPos).Length() < m_fNestRadius + 0.1);
   return cPos;
}

/****************************************/
/****************************************/

CColor CCollectionLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if((c_position_on_plane - m_cNestPos).Length() < m_fNestRadius) {
      return CColor::GRAY50;
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CCollectionLoopFunctions::PreStep() {
   /* Spawn a new ball periodically, keeping at most m_unMaxBalls around */
   if(GetSpace().GetSimulationClock() % m_unSpawnPeriod == 0 &&
      m_cBalls.size() < m_unMaxBalls) {
      m_cBalls.push_back(RandomBallPosition());
   }

   CSpace::TMapPerType& cFootBots = GetSpace().GetEntitiesByType("foot-bot");
   std::vector<CVector2> cRobotPositions;
   for(CSpace::TMapPerType::iterator it = cFootBots.begin();
       it != cFootBots.end();
       ++it) {
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotCollector& cController =
         dynamic_cast<CFootBotCollector&>(cFootBot.GetControllableEntity().GetController());
      CVector2 cPos(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                    cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      cRobotPositions.push_back(cPos);

      /* Virtual camera: nearest free ball within sight range. Carrying
       * robots also get sightings — they broadcast them so free
       * neighbors can be recruited. */
      SInt32 nPicked = -1;
      Real fBestSight = m_fSightRange;
      SInt32 nSighted = -1;
      for(size_t i = 0; i < m_cBalls.size(); ++i) {
         Real fDist = (cPos - m_cBalls[i]).Length();
         if(!cController.IsCarrying() && fDist < m_fPickupRadius) {
            nPicked = i;
            break;
         }
         if(fDist < fBestSight) {
            fBestSight = fDist;
            nSighted = i;
         }
      }

      if(cController.IsCarrying()) {
         if(nSighted >= 0) {
            cController.SetBallSighting(m_cBalls[nSighted]);
         }
         /* Deposit when inside the nest circle */
         if((cPos - m_cNestPos).Length() < m_fNestRadius) {
            cController.Drop();
            ++m_unScore;
            LOG << "[collection] " << cFootBot.GetId()
                << " deposited a ball. Total: " << m_unScore << std::endl;
         }
      }
      else {
         /* Touching a ball? Non-physical pickup: ball sticks to the robot */
         if(nPicked >= 0) {
            m_cBalls.erase(m_cBalls.begin() + nPicked);
            cController.PickUp();
         }
         else if(nSighted >= 0) {
            cController.SetBallSighting(m_cBalls[nSighted]);
         }
      }
   }

   /* Collision metric: foot-bot body radius = 0.085 m, so two bodies
    * touch when centers are < 0.171 m apart; count with a 1 cm margin */
   for(size_t i = 0; i < cRobotPositions.size(); ++i) {
      Real fWall = 3.95 - Max(Abs(cRobotPositions[i].GetX()),
                              Abs(cRobotPositions[i].GetY()));
      if(fWall < m_fMinWallClearance) {
         m_fMinWallClearance = fWall;
      }
      for(size_t j = i + 1; j < cRobotPositions.size(); ++j) {
         Real fDist = (cRobotPositions[i] - cRobotPositions[j]).Length();
         if(fDist < 0.181) {
            ++m_unCollisionTicks;
         }
         if(fDist < m_fMinPairDistance) {
            m_fMinPairDistance = fDist;
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CCollectionLoopFunctions, "collection_loop_functions")
