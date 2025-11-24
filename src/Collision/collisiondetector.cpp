




#include <AdamLib/Collision/CollisionDetector.hpp>
#include "CollisionDetectorInternal.hpp"
#include "AABB.hpp"

using namespace AdamLib;

aabb::Tree detectors[NUM_COLLISION_LAYERS];

/*------------------------------------*/

void CollisionDetector::addCollisionNode(const CollisionNode* _addition)
{
  
  detectors[_addition->getCollisionLayer()].insertParticle(_addition);
}

void CollisionDetector::removeCollisionNode(const CollisionNode* _removal)
{
  detectors[_removal->getCollisionLayer()].removeParticle(_removal);
}

void CollisionDetector::queryTreeForCollisions()
{
  for(int i=0; i<NUM_COLLISION_LAYERS; ++i)
    detectors[i].queryAll();
  
}

void CollisionDetector::updateCollisionNode(const CollisionNode *_update)
{
  detectors[_update->getCollisionLayer()].updateParticle(_update);
}



