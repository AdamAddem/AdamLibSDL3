#include <iostream>
#include <ostream>
#include <AdamLib/Collision/CollisionShapes.hpp>
#include <AdamLib/Utilities/Math.hpp>
#include <AdamLib/Nodes/SpriteNode.hpp>
#include <AdamLib/Nodes/CollisionNode.hpp>
#include <AdamLib/Collision/CollisionDetector.hpp>
#include <AdamLib/Core/Input.hpp>

using namespace AdamLib;

struct Player : CollisionNodeInstanceController
{
  double speed = 500;
  Vec2 velocity{0,0};

  void process(double _dt) override
  {

    self()->movePos(velocity * _dt);
    CollisionDetector::queryTreeForCollisions();
  }

  void onReady() override
  {
    RegisterKeyChangeConnection(KEY_RIGHT, doMovement);
    RegisterKeyChangeConnection(KEY_LEFT, doMovement);
    RegisterKeyChangeConnection(KEY_UP, doMovement);
    RegisterKeyChangeConnection(KEY_DOWN, doMovement);
    RegisterKeyChangeConnection(KEY_LSHIFT, doMovement);
  }

  void onCollisionWith(const CollisionNode *_collider) override
  {
    std::cout << "Dick!" << std::endl;
  }

  void doMovement()
  {
    velocity.x = Input::keystate(KEY_RIGHT) - Input::keystate(KEY_LEFT);
    velocity.y = Input::keystate(KEY_DOWN) - Input::keystate(KEY_UP);



    velocity.normalize();
    velocity *= speed;
    if(Input::keystate(KEY_LSHIFT))
      velocity /= 4.0;
  }

};

CollisionNodeTemplate player_coll{"Player_Collision", CollisionRectangle(Vec2(0,0), 144, 144), CollisionController(Player)};
CollisionNodeTemplate box_coll{"Box_Collision", CollisionRectangle(Vec2(0,0), 144, 144)};


void loadgame() {
  Node& root = Node::getRoot();

  player_coll.default_pos_ = {200,200};
  player_coll.renderCollision = true;

  box_coll.default_pos_ = {400,400};
  box_coll.renderCollision = true;

  CollisionNode* player = (CollisionNode*)player_coll.createInstance();
  CollisionNode* box = (CollisionNode*)box_coll.createInstance();

  root.addChild(player);
  root.addChild(box);
  
}