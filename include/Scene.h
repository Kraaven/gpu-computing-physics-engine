#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"

class Scene {
public:
    int width = 1600;
    int height = 1000;

    void loadDefault(PhysicsWorld &world) {
        world.bodies.clear();
        world.worldWidth = width; world.worldHeight = height;
    }

    void loadLiquid(PhysicsWorld &world, float particleRadius = 2.0f, int maxParticles = 2000) {
    world.bodies.clear();
    world.worldWidth = width;
    world.worldHeight = height;

    // spawn region
    float sx = width * 0.1f;
    float sy = height * 0.1f;
    float sw = width * 0.8f;
    float sh = height * 0.4f;

    int cols = (int)(sw / (particleRadius * 1.6f));
    int rows = (int)(sh / (particleRadius * 1.6f));

    // Calculate the maximum number of particles
    int maxPossibleParticles = cols * rows;

    // Clamp the number of particles to maxParticles
    int numParticles = std::min(maxPossibleParticles, maxParticles);

    int particleCount = 0;

    for (int y = 0; y < rows && particleCount < numParticles; ++y) {
        for (int x = 0; x < cols && particleCount < numParticles; ++x) {
            float px = sx + (x + 0.5f) * (sw / cols);
            float py = sy + (y + 0.5f) * (sh / rows);
            Body p({px, py}, 1.0f);
            p.collider.radius = particleRadius;
            p.color = ColorFromHSV((x * 7 + y * 13) % 360, 0.5f, 0.9f);
            world.addBody(p);
            ++particleCount;
        }
    }
}

};
