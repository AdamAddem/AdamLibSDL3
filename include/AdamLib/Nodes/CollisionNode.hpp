#pragma once

#include <AdamLib/Utilities/Math.hpp>

#include <AdamLib/Nodes/Node.hpp>
#include <AdamLib/Collision/CollisionShapes.hpp>
#include <AdamLib/Core/Rendering.hpp>
#include <memory>

namespace aabb {
  class Tree;
}

namespace AdamLib
{

class CollisionNode;
class CollisionNodeTemplate;
struct CollisionNodeInstanceController;

//! Node derivative for collision detection
/*!
    Handles narrow-phase collision detection.
    Shape contains data used in detection.
    Must be added to a Detection Tree for use, and later removed.
*/
class CollisionNode : public Node
{
  friend class CollisionNodeTemplate;
  friend class aabb::Tree;
  
  CollisionShape shape_;

  std::unique_ptr<Renderer::SetOfPoints> points_to_render_;
  
  AABB aabb_;
  bool doRendering_;
  uint8_t collision_layer_;

protected:
  CollisionNode(const std::string& _name, const CollisionShape& _shape, uint8_t _collision_layer = 0,
    bool _doRendering = false, NodeInstanceController* _controller = nullptr, Node* _parent = nullptr);

public:
  virtual ~CollisionNode() override;

  void setCollisionRendering(bool _renderCollision);
  virtual void movePos(const Vec2& _move) override;
  void signalCollisionWith(const CollisionNode* _collider) const;

  
  const AABB& getAABB() const {return aabb_;}
  uint8_t getCollisionLayer() const {return collision_layer_;}

};


//! NodeTemplate derivative for CollisionNode
class CollisionNodeTemplate : public NodeTemplate
{
  CollisionShape shape_;
protected:
  virtual Node* createNode(NodeInstanceController* _controller) override {return new CollisionNode(default_name_, shape_, collision_layer_, renderCollision, _controller);}

public:
  CollisionNodeTemplate(const std::string& _name, const CollisionShape& _shape, const std::function<CollisionNodeInstanceController*()>& _controller_factory = nullptr);
  virtual ~CollisionNodeTemplate() override = default;
  bool renderCollision = false;
  uint8_t collision_layer_ = 0;
};

//! NodeInstanceController derivative for CollisionNode
struct CollisionNodeInstanceController : public NodeInstanceController
{
  CollisionNodeInstanceController() = default;
  virtual CollisionNode* self() override {return dynamic_cast<CollisionNode*>(self_);}
  virtual void onCollisionWith(const CollisionNode* _collider) {}
};

#define CollisionController(Typename) ([] () -> CollisionNodeInstanceController* {return static_cast<CollisionNodeInstanceController*>(new Typename());})


}
