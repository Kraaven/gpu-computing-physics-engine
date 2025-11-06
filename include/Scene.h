#pragma once
#include "PhysicsWorld.h"
#include "raylib.h"
#include <iostream>

class Scene {
public:
    int width = 1600;
    int height = 1000;

    void loadDefault(PhysicsWorld &world) {
        world.bodies.clear();
        world.worldWidth = width;
        world.worldHeight = height;
    }

    void loadLiquid(PhysicsWorld &world, float particleRadius = 2.0f, int maxParticles = 5000) {
        world.bodies.clear();
        world.worldWidth = width;
        world.worldHeight = height;

        // Dynamic particle limit based on size
        int adjustedMax = maxParticles;
        if (particleRadius > 3.0f) {
            // Fewer large particles to maintain performance
            adjustedMax = (int)(maxParticles * (3.0f / particleRadius));
        }
        adjustedMax = std::max(500, std::min(adjustedMax, 10000));

        std::cout << "[Scene] Loading liquid with radius=" << particleRadius 
                  << ", maxParticles=" << adjustedMax << "\n";

        // Spawn region
        float sx = width * 0.1f;
        float sy = height * 0.1f;
        float sw = width * 0.8f;
        float sh = height * 0.4f;

        // Calculate grid spacing (with some separation)
        float spacing = particleRadius * 2.1f;
        int cols = (int)(sw / spacing);
        int rows = (int)(sh / spacing);

        // Clamp total particles
        int maxPossibleParticles = cols * rows;
        int numParticles = std::min(maxPossibleParticles, adjustedMax);

        std::cout << "[Scene] Creating " << numParticles << " particles ("
                  << cols << "x" << rows << " grid)\n";

        // Pre-reserve memory
        world.bodies.reserve(numParticles);

        int particleCount = 0;
        float xStep = sw / cols;
        float yStep = sh / rows;

        for (int y = 0; y < rows && particleCount < numParticles; ++y) {
            for (int x = 0; x < cols && particleCount < numParticles; ++x) {
                float px = sx + (x + 0.5f) * xStep;
                float py = sy + (y + 0.5f) * yStep;
                
                Body p({px, py}, 1.0f);
                p.collider.radius = particleRadius;
                
                // Vary color for visual interest
                float hue = ((x * 7 + y * 13) % 360);
                p.color = ColorFromHSV(hue, 0.6f, 0.95f);
                
                // Slightly lower restitution for liquid-like behavior
                p.restitution = 0.1f;
                
                world.addBody(p);
                ++particleCount;
            }
        }

        std::cout << "[Scene] Liquid scene loaded successfully\n";
    }
};