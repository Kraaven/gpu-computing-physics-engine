#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"
#include <algorithm>

enum class InputMode { Object, Force };

class InputManager {
public:
  InputMode mode = InputMode::Object;
  float spawnRadius = 15.0f;
  float forceStrength = 200000.0f;
  float influenceRadius = 200.0f;

  // debug toggles
  bool showContacts = false;
  bool gravityEnabled = true;

  void handleInput(PhysicsWorld &world) {
    // Toggle between Object and Force mode with SPACE
    if (IsKeyPressed(KEY_SPACE)) {
      mode = (mode == InputMode::Object) ? InputMode::Force : InputMode::Object;
    }

    // Debug toggles
    if (IsKeyPressed(KEY_D)) showContacts = !showContacts;
    if (IsKeyPressed(KEY_G)) gravityEnabled = !gravityEnabled;
    if (IsKeyPressed(KEY_C)) clearDynamic(world);

    Vector2 mouse = GetMousePosition();
    Vec2 mpos(mouse.x, mouse.y);

    if (mode == InputMode::Object) {
      handleObjectMode(world, mpos);
    } else {
      handleForceMode(world, mpos);
    }

    // apply gravity toggle to world
    if (!gravityEnabled) world.gravity = Vec2::zero();
    else world.gravity = {0.0f, 300.0f};
  }

private:
  void clearDynamic(PhysicsWorld &world) {
    // remove non-static bodies
    for (int i = (int)world.bodies.size() - 1; i >= 0; --i) {
      if (!world.bodies[i].isStatic)
        world.bodies.erase(world.bodies.begin() + i);
    }
  }

  // Object Mode
  void handleObjectMode(PhysicsWorld &world, const Vec2 &mousePos) {
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      Body b(mousePos, 1.0f);
      b.collider.type = ShapeType::Circle;
      b.collider.radius = spawnRadius;
      b.color = ColorFromHSV(GetRandomValue(0, 359), 0.8f, 0.9f);
      world.addBody(b);
    }

    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
      for (int i = (int)world.bodies.size() - 1; i >= 0; --i) {
        Body &b = world.bodies[i];
        if (b.isStatic) continue;
        if (b.collider.type != ShapeType::Circle) continue;

        Vec2 d = mousePos - b.position;
        if (d.length2() <= b.collider.radius * b.collider.radius) {
          world.bodies.erase(world.bodies.begin() + i);
          break;
        }
      }
    }
  }

  // Force Mode
  void handleForceMode(PhysicsWorld &world, const Vec2 &mousePos) {
    bool attract = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);
    bool repel = IsMouseButtonDown(MOUSE_LEFT_BUTTON);

    if (!attract && !repel) return;

    for (auto &b : world.bodies) {
      if (b.isStatic) continue;

      Vec2 dir = b.position - mousePos;
      float dist2 = dir.length2();

      if (dist2 > influenceRadius * influenceRadius) continue;

      float dist = std::sqrt(std::max(dist2, 1.0f));
      Vec2 n = dir / dist; // direction from mouse → body

      if (attract) n = n * -1.0f;

      float strength = forceStrength / (dist + 100.0f);

      // apply velocity kick
      b.velocity += n * (strength * world.dt);
    }
  }
};
