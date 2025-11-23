#pragma once

#include <AdamLib/Utilities/Math.hpp>
#include <variant>

namespace AdamLib
{

/*
  To Do:
    -Consider changing CollisionRay constructor to accept angle instead of direction normalized

*/



//! Contains bottom leftmost and top rightmost point of rectangle
struct AABB
{
  Vec2 bottom_left_;
  Vec2 top_right_;
};


struct CollisionRectangle
{
  CollisionRectangle(const Vec2& _center, double _width, double _height);

  Vec2 center_; //!<Center of rectangle, relative to parent
  Vec2 width_height_; //!<The width and height of rectangle, x = w,  y = h
};

struct CollisionCircle
{
  CollisionCircle(const Vec2& _center, double _r);




  Vec2 center_; //!<Center of circle, relative to parent
  double r_; //!<Radius
};

struct CollisionCapsule
{
  CollisionCapsule(const Vec2& _a_center, const Vec2& _b_center, double _r);


  Vec2 a_center_; //!<Circle a center, relative to parent
  Vec2 b_center_; //!<Circle b center, relative to parent
  double r_; //!<Radius
};

struct CollisionRay
{
  CollisionRay(const Vec2& _center, const Vec2& _direction_normalized, double _len);
  
  Vec2 direction_normalized_; //!<The direction of the ray, with positive y pointing downwards

  Vec2 p1_; //!<Point 1 (Origin point of ray, relative to parent)
  Vec2 p2_; //!<Point 2 (Calculated with direction and length, relative to parent)
  
  double len_; //!<Length/magnitude of direction
};

using CollisionShape = std::variant<CollisionRectangle, CollisionCircle, CollisionCapsule, CollisionRay>;


// #define Rectangle(offset, width, height) std::bind([] (const Vec2 _offset, const double _width, const double _height) -> CollisionShape* { return new CollisionRectangle(_offset, _width, _height);}, offset, width, height)
// #define Circle(offset, radius) std::bind([] (const Vec2 _offset, const float _r) -> CollisionShape* { return new CollisionCircle(_offset, _r);}, offset, radius)
// #define Capsule(a_offset, b_offset, radius) std::bind([] (const Vec2 _a_offset, const Vec2 _b_offset, const float _r) -> CollisionShape* { return new CollisionCapsule(_a_offset, _b_offset, _r);}, a_offset, b_offset, radius)
// #define Ray(offset, direction, length) std::bind([] (const Vec2 _offset, const Vec2 _direction, const float _len) -> CollisionShape* { return new CollisionRay(_offset, _direction, _len);}, offset, direction, length)


}

//To Do: Implement
// struct CollisionPolygon : public CollisionShape
// {
//   CollisionPolygon();
//   AABB createAABB() override;

//   int numvert_;
//   Vec2 verts[MAX_POLYGON_VERTEXS];
//   Vec2 norms[MAX_POLYGON_VERTEXS];

// };