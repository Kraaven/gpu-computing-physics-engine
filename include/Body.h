#pragma once
#include "Vec2.h"
#include "raylib.h"

enum class ShapeType { Circle, AABB };

struct Collider {
  ShapeType type = ShapeType::Circle;
  float radius = 10.0f;
  Vec2 halfExtents = {10, 10};
};

struct Body {
  Vec2 position;
  Vec2 prevPosition; // used for render interpolation
  Vec2 velocity;
  Vec2 force;
  float mass = 1.0f;
  float invMass = 1.0f;
  Collider collider;
  Color color = BLUE;
  bool isStatic = false;
  float restitution = 0.6f; // per-body restitution (defaults to world)

  Body() = default;
  Body(const Vec2 &pos, float m = 1.0f) : position(pos), prevPosition(pos), mass(m) {
    invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
  }
};
