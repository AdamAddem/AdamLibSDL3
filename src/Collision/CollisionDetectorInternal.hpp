#pragma once
#include <AdamLib/Nodes/CollisionNode.hpp>



namespace AdamLib::CollisionDetector
{
  void addCollisionNode(const CollisionNode* _addition);
  void removeCollisionNode(const CollisionNode* _removal);

  void updateCollisionNode(const CollisionNode* _update);
}
