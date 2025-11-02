#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"

class Scene {
public:
    int width = 1600;
    int height = 1000;

    void loadDefault(PhysicsWorld &world) {
        float t = 10.0f; // wall thickness

        Body ground({width / 2.0f, height - t/2.0f}, 0.0f);
        Body ceiling({width / 2.0f, t/2.0f}, 0.0f);
        Body leftWall({t/2.0f, height / 2.0f}, 0.0f);
        Body rightWall({width - t/2.0f, height / 2.0f}, 0.0f);

        ground.isStatic = ceiling.isStatic = leftWall.isStatic = rightWall.isStatic = true;
        ground.collider.type = ceiling.collider.type = leftWall.collider.type = rightWall.collider.type = ShapeType::AABB;

        ground.collider.halfExtents = {width / 2.0f, t / 2.0f};
        ceiling.collider.halfExtents = {width / 2.0f, t / 2.0f};
        leftWall.collider.halfExtents = {t / 2.0f, height / 2.0f};
        rightWall.collider.halfExtents = {t / 2.0f, height / 2.0f};

        ground.color = ceiling.color = leftWall.color = rightWall.color = GRAY;

        ground.restitution = ceiling.restitution = leftWall.restitution = rightWall.restitution = 0.3f;

        world.addBody(ground);
        world.addBody(ceiling);
        world.addBody(leftWall);
        world.addBody(rightWall);

        // some dynamic bodies for testing
        Body a({width/2.0f - 100, 100}, 1.0f);
        a.collider.type = ShapeType::Circle;
        a.collider.radius = 25;
        a.color = RED;

        Body b({width/2.0f + 100, 200}, 1.5f);
        b.collider.type = ShapeType::Circle;
        b.collider.radius = 35;
        b.color = BLUE;

        world.addBody(a);
        world.addBody(b);
    }
};
