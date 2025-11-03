#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"

class Scene {
public:
    int width = 1600;
    int height = 1000;

    void loadDefault(PhysicsWorld &world) {
        world.bodies.clear();
        world.worldWidth = width; 
        world.worldHeight = height;
    
    }

    void loadLiquid(PhysicsWorld &world, float particleRadius = 4.0f) {
        world.bodies.clear();
        world.worldWidth = width; world.worldHeight = height;

        // spawn region
        float sx = width * 0.1f; float sy = height * 0.1f;
        float sw = width * 0.8f; float sh = height * 0.4f;

        int cols = (int)(sw / (particleRadius * 1.6f));
        int rows = (int)(sh / (particleRadius * 1.6f));

        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                float px = sx + (x + 0.5f) * (sw / cols);
                float py = sy + (y + 0.5f) * (sh / rows);
                Body p({px, py}, 1.0f);
                p.collider.radius = particleRadius;
                p.color = ColorFromHSV((x*7 + y*13) % 360, 0.5f, 0.9f);
                world.addBody(p);
            }
        }
    }
};