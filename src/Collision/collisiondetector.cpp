#include <AdamLib/Collision/CollisionShapes.hpp>
#include <AdamLib/Collision/CollisionDetector.hpp>
#include <AdamLib/Nodes/CollisionNode.hpp>
#include <AdamLib/Defines.hpp>

#include "AABB.hpp"
#include "cute_c2.hpp"
#include <cassert>
#include <memory>

using namespace AdamLib;

//aabb::Tree detectors[16];

constexpr c2AABB aabbConversion(const CollisionRectangle* _rect, const double _xoffset, const double _yoffset)
{
  return 
  {
    {_rect->center_.x - _rect->width_height_.x/2 + _xoffset, _rect->center_.y - _rect->width_height_.y/2 + _yoffset}, 
    {_rect->center_.x + _rect->width_height_.x/2 + _xoffset, _rect->center_.y + _rect->width_height_.y/2 + _yoffset}
  };
}
constexpr c2Circle circleConversion(const CollisionCircle* _circle, const double _xoffset, const double _yoffset)
{
  return 
  {
    {_circle->center_.x + _xoffset, -_circle->center_.y + _yoffset}, 
    _circle->r_
  };
}
constexpr c2Capsule capsuleConversion(const CollisionCapsule* _capsule, const double _xoffset, const double _yoffset)
{
  return 
  {
    {_capsule->a_center_.x + _xoffset, _capsule->a_center_.y + _yoffset}, 
    {_capsule->b_center_.x + _xoffset, _capsule->b_center_.y + _yoffset}, 
    _capsule->r_
  };
}

/*------------------------------------*/

CollisionDetector::CollisionDetector(const Vec2& _dimensions) : tree_(std::make_unique<CollisionTree>(SKIN_THICKNESS, _dimensions))
{
  
}

CollisionDetector::~CollisionDetector()
{

}


void CollisionDetector::addCollisionNode(CollisionNode* _addition)
{
  if(contained_nodes_.insert(_addition).second == true)
  {
    tree_->insertParticle(_addition);
  }

}

void CollisionDetector::removeCollisionNode(CollisionNode* _removal)
{
  if(contained_nodes_.erase(_removal) == 1)
  {
    tree_->removeParticle(_removal);
  }
}

void CollisionDetector::queryTreeForCollisions()
{
  for(auto node : contained_nodes_)
  {
    std::vector<CollisionNode*> colliders = tree_->query(node);
    for(CollisionNode* collider : colliders)
    {
      if(!determineCollisionBetween(node, collider))
        continue;

      if(node->controller_)
        static_cast<CollisionNodeInstanceController*>(node->controller_.get())->onCollisionWith(collider);


      if(collider->controller_)
        static_cast<CollisionNodeInstanceController*>(collider->controller_.get())->onCollisionWith(node);
      
      
    }
  }
}

bool CollisionDetector::determineCollisionBetween(CollisionNode* _c1, CollisionNode* _c2) const
{
  C2_TYPE type1, type2;
  c2AABB rect1, rect2;
  c2Circle circ1, circ2;
  c2Capsule cap1, cap2;
  void *c2_1 = NULL, *c2_2 = NULL;

  //center on the origin for floating point precision purposes.
  double xoffset = std::min(_c1->pos_.x, _c2->pos_.x);
  double yoffset = std::min(_c1->pos_.y, _c2->pos_.y);


  
  

  if(const auto* rect = std::get_if<CollisionRectangle>(&_c1->shape_))
  {
    type1 = C2_TYPE_AABB;
    rect1 = aabbConversion(rect, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
    c2_1 = &rect1;
  }
  else if(const auto* circ = std::get_if<CollisionCircle>(&_c1->shape_))
  {
    type1 = C2_TYPE_CIRCLE;
    circ1 = circleConversion(circ, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
    c2_1 = &circ1;
  }
  else if(const auto* caps = std::get_if<CollisionCapsule>(&_c1->shape_))
  {
    type1 = C2_TYPE_CAPSULE;
    cap1 = capsuleConversion(caps, xoffset + _c1->pos_.x, yoffset + _c1->pos_.y);
    c2_1 = &cap1;
  }
  else if(auto* ray = std::get_if<CollisionRay>(&_c1->shape_))
  {
    assert(0);
  }
  else
    assert(0);


  if(const auto* rect = std::get_if<CollisionRectangle>(&_c2->shape_))
  {
    type2 = C2_TYPE_AABB;
    rect2 = aabbConversion(rect, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
    c2_2 = &rect2;
  }
  else if(const auto* circ = std::get_if<CollisionCircle>(&_c2->shape_))
  {
    type2 = C2_TYPE_CIRCLE;
    circ2 = circleConversion(circ, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
    c2_2 = &circ2;
  }
  else if(const auto* caps = std::get_if<CollisionCapsule>(&_c2->shape_))
  {
    type2 = C2_TYPE_CAPSULE;
    cap2 = capsuleConversion(caps, xoffset + _c2->pos_.x, yoffset + _c2->pos_.y);
    c2_2 = &cap2;
  }
  else if(auto* ray = std::get_if<CollisionRay>(&_c2->shape_))
  {
    assert(0);
  }
  else
    assert(0);



  return c2Collided(c2_1, NULL, type1, c2_2, NULL, type2);

}


void CollisionDetector::updateTree()
{
  for(auto particle : contained_nodes_)
  {
    updateNodePosition(particle);
  }
}

void CollisionDetector::updateNodePosition(CollisionNode* _update)
{
  tree_->updateParticle(_update);
}


