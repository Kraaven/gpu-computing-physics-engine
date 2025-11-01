#pragma once
#include <vector>
#include <omp.h>
#include "Body.h"

class PhysicsWorld {
public:
    std::vector<Body> bodies;
    Vec2 gravity = {0.0f, 300.0f};
    float damping = 0.98f;
    float dt = 1.0f / 120.0f;

    PhysicsWorld() = default;

    int addBody(const Body& b) {
        bodies.push_back(b);
        return (int)bodies.size() - 1;
    }

    void step() {
        #pragma omp parallel for
        for (int i = 0; i < (int)bodies.size(); ++i) {
            Body& b = bodies[i];
            if (b.isStatic) continue;
            b.force += gravity * b.mass;
        }

        #pragma omp parallel for
        for (int i = 0; i < (int)bodies.size(); ++i) {
            Body& b = bodies[i];
            if (b.isStatic) continue;

            Vec2 accel = b.force * b.invMass;
            b.velocity += accel * dt;
            b.position += b.velocity * dt;

            b.velocity = b.velocity * damping;

            b.force = Vec2::zero();
        }
    }
};
