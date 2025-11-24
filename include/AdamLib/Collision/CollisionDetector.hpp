#pragma once
namespace AdamLib::CollisionDetector
{
  
  //! Iterates over each node and checks for possible collisions
  void queryTreeForCollisions();

  //! Update the positions of all active trees
  void updateTrees();
  
}

