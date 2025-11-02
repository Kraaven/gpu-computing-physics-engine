#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"

class Scene {
public:
    int width = 1280;
    int height = 720;

    void loadDefault(PhysicsWorld& world) {
        // 1️⃣ Walls
        float t = 10.0f; // thickness of walls

        Body ground({width / 2.0f, height - t / 2.0f}, 0.0f);
        Body ceiling({width / 2.0f, t / 2.0f}, 0.0f);
        Body leftWall({t / 2.0f, height / 2.0f}, 0.0f);
        Body rightWall({width - t / 2.0f, height / 2.0f}, 0.0f);

        ground.isStatic = ceiling.isStatic = leftWall.isStatic = rightWall.isStatic = true;
        ground.collider.type = ceiling.collider.type = leftWall.collider.type = rightWall.collider.type = ShapeType::AABB;

        ground.collider.halfExtents = {width / 2.0f, t / 2.0f};
        ceiling.collider.halfExtents = {width / 2.0f, t / 2.0f};
        leftWall.collider.halfExtents = {t / 2.0f, height / 2.0f};
        rightWall.collider.halfExtents = {t / 2.0f, height / 2.0f};

        ground.color = ceiling.color = leftWall.color = rightWall.color = GRAY;

        world.addBody(ground);
        world.addBody(ceiling);
        world.addBody(leftWall);
        world.addBody(rightWall);

        // 2️⃣ Some test dynamic objects
        Body b1({width / 2.0f - 100, 100}, 1.0f);
        b1.collider.type = ShapeType::Circle;
        b1.collider.radius = 25;
        b1.color = RED;

        Body b2({width / 2.0f + 100, 200}, 1.5f);
        b2.collider.type = ShapeType::Circle;
        b2.collider.radius = 35;
        b2.color = BLUE;

        world.addBody(b1);
        world.addBody(b2);
    }
};
