#include <AdamLib/Collision/CollisionShapes.hpp>
#include <AdamLib/Utilities/Math.hpp>
#include <AdamLib/Nodes/CollisionNode.hpp>

#include "../Core/RendererInternal.hpp"
#include "../Collision/CollisionDetectorInternal.hpp"

#include <cassert>
#include <memory>


using namespace AdamLib;



CollisionNode::CollisionNode(const std::string& _name, const CollisionShape& _shape, uint8_t _collision_layer, bool _doRendering, NodeInstanceController* _controller,  Node* _parent) :
  Node(_name, _controller, _parent), shape_(_shape), points_to_render_(std::make_unique<Renderer::SetOfPoints>()), doRendering_(_doRendering), collision_layer_(_collision_layer)
{
  if(const CollisionRectangle* rect = std::get_if<CollisionRectangle>(&shape_))
  {
    Vec2 bleft = {rect->center_.x - rect->width_height_.x/2, rect->center_.y + rect->width_height_.y/2};
    Vec2 tright = {rect->center_.x + rect->width_height_.x/2, rect->center_.y - rect->width_height_.y/2};
    bleft += pos_;
    tright += pos_;

    aabb_ = {bleft, tright};
    points_to_render_->points_.resize(5);

    points_to_render_->points_[0].x = aabb_.bottom_left_.x;
    points_to_render_->points_[0].y = aabb_.bottom_left_.y;

    points_to_render_->points_[1].x = aabb_.bottom_left_.x;
    points_to_render_->points_[1].y = aabb_.top_right_.y;

    points_to_render_->points_[2].x = aabb_.top_right_.x;
    points_to_render_->points_[2].y = aabb_.top_right_.y;

    points_to_render_->points_[3].x = aabb_.top_right_.x;
    points_to_render_->points_[3].y = aabb_.bottom_left_.y;

    points_to_render_->points_[4] = points_to_render_->points_[0];
  }
  else if(const CollisionCircle* circ = std::get_if<CollisionCircle>(&shape_))
  {
    aabb_ = {{circ->center_.x - circ->r_, circ->center_.y + circ->r_}, {circ->center_.x + circ->r_, circ->center_.y - circ->r_}};
    points_to_render_->points_.resize(9);
    const Vec2 center = circ->center_ + pos_;
    const double& radius = circ->r_;

    points_to_render_->points_[0].x = center.x;
    points_to_render_->points_[0].y = center.y + radius;

    points_to_render_->points_[1].x = center.x - radius*0.71;
    points_to_render_->points_[1].y = center.y + radius*0.71;

    points_to_render_->points_[2].x = center.x - radius;
    points_to_render_->points_[2].y = center.y;
    
    points_to_render_->points_[3].x = center.x - radius*0.71;
    points_to_render_->points_[3].y = center.y - radius*0.71;

    points_to_render_->points_[4].x = center.x;
    points_to_render_->points_[4].y = center.y - radius;

    points_to_render_->points_[5].x = center.x + radius*0.71;
    points_to_render_->points_[5].y = center.y - radius*0.71;

    points_to_render_->points_[6].x = center.x + radius;
    points_to_render_->points_[6].y = center.y;

    points_to_render_->points_[7].x = center.x + radius*0.71;
    points_to_render_->points_[7].y = center.y + radius*0.71;

    points_to_render_->points_[8] = points_to_render_->points_[0];
    
  }
  else if(const CollisionCapsule* caps = std::get_if<CollisionCapsule>(&shape_))
  {
    double rightmost = std::max(caps->a_center_.x, caps->b_center_.x) + caps->r_;
    double leftmost = std::min(caps->a_center_.x, caps->b_center_.x) - caps->r_;

    //increased y is lower height, origin is top left
    double upmost = std::min(caps->a_center_.y, caps->b_center_.y) - caps->r_;
    double downmost = std::max(caps->a_center_.y, caps->a_center_.y) + caps->r_;

    aabb_ = {{leftmost, downmost}, {rightmost, upmost}};

    points_to_render_->points_.resize(11);
  }
  else if(const CollisionRay* ray = std::get_if<CollisionRay>(&shape_))
  {
    const double leftmost = std::min(ray->p1_.x, ray->p2_.x);
    const double rightmost = std::max(ray->p1_.x, ray->p2_.x);

    //increased y is lower height, origin is top left
    const double upmost = std::min(ray->p1_.y, ray->p2_.y);
    const double downmost = std::max(ray->p1_.y, ray->p2_.y);

    Vec2 bleft = {leftmost, downmost};
    Vec2 tright = {rightmost, upmost};
    bleft += pos_;
    tright += pos_;
    aabb_ = {bleft, tright};

    points_to_render_->points_.resize(2);


    points_to_render_->points_[0].x = ray->p1_.x + pos_.x;
    points_to_render_->points_[0].y = ray->p1_.y + pos_.y;

    points_to_render_->points_[1].x = ray->p2_.x + pos_.x;
    points_to_render_->points_[1].y = ray->p2_.y + pos_.y;
  }
  else
    assert(false);

  if(_doRendering)
    Renderer::addSetPoints(points_to_render_.get());

  if(_collision_layer >= NUM_COLLISION_LAYERS)
      _collision_layer = NUM_COLLISION_LAYERS - 1;
  CollisionDetector::addCollisionNode(this);
}

CollisionNode::~CollisionNode()
{
  Renderer::removeSetPoints(points_to_render_.get());
  CollisionDetector::removeCollisionNode(this);
}

void CollisionNode::setCollisionRendering(const bool _renderCollision)
{
  if(doRendering_ && !_renderCollision)
    Renderer::addSetPoints(points_to_render_.get());
  else if(!doRendering_)
    Renderer::removeSetPoints(points_to_render_.get());

  doRendering_ = _renderCollision;
}

void CollisionNode::movePos(const Vec2& _move)
{
  Node::movePos(_move);
  aabb_.bottom_left_ += _move;
  aabb_.top_right_ += _move;

  CollisionDetector::updateCollisionNode(this);

  for(auto& [px, py] : points_to_render_->points_)
  {
    px += _move.x;
    py += _move.y;
  }
}

void CollisionNode::signalCollisionWith(const CollisionNode* _collider) const
{
  if(controller_)
    static_cast<CollisionNodeInstanceController*>(controller_.get())->onCollisionWith(_collider);

}

/*----- CollisionNodeTemplate -----*/

CollisionNodeTemplate::CollisionNodeTemplate(const std::string& _name, const CollisionShape& _shape, const std::function<CollisionNodeInstanceController*()>& _controller_factory) :
NodeTemplate(_name, _controller_factory), shape_(_shape) {}

/*----- CollisionTemplate -----*/
