#pragma once
#include "Vec2.h"
#include "raylib.h"
#include <cmath>

struct Collider {
  // Circle-only collider now
  float radius = 10.0f;
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

  Body() {
    position = Vec2::zero();
    prevPosition = position;
    velocity = Vec2::zero();
    force = Vec2::zero();
  }

  Body(const Vec2 &pos, float m = 1.0f)
      : position(pos), prevPosition(pos), velocity(Vec2::zero()), force(Vec2::zero()), mass(m) {
    invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
  }

  void clampSanity(float maxPos = 1e6f, float maxVel = 5000.0f) {
    // clamp large positions / velocities and fix NaNs/Infs
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
      position = prevPosition;
      velocity = Vec2::zero();
      force = Vec2::zero();
    }
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) {
      velocity = Vec2::zero();
    }

    if (std::abs(position.x) > maxPos) position.x = (position.x > 0) ? maxPos : -maxPos;
    if (std::abs(position.y) > maxPos) position.y = (position.y > 0) ? maxPos : -maxPos;

    float vsq = velocity.length2();
    if (vsq > maxVel * maxVel) {
      float vlen = std::sqrt(vsq);
      velocity = velocity * (maxVel / vlen);
    }
  }
};