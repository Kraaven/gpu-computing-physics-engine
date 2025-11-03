#pragma once
#include "Body.h"
#include "SpatialHash.h"
#include <algorithm>
#include <omp.h>
#include <vector>
#include <iostream>
#include <cmath>

struct Contact {
    int a;
    int b;
    Vec2 normal;       // A -> B
    float penetration;
    Vec2 contactPoint;
};

class PhysicsWorld {
public:
    std::vector<Body> bodies;
    Vec2 gravity = {0.0f, 300.0f};
    float damping = 0.999f;
    float dt = 1.0f / 120.0f;

    float defaultRestitution = 0.6f;
    float positionalCorrectionPercent = 0.4f;
    float positionalCorrectionSlop = 0.02f;

    // safety limits
    float maxVelocity = 3000.0f;
    float maxImpulse = 20000.0f;
    float maxPosCorrection = 200.0f;

    // world bounds (used for boundary collisions)
    int worldWidth = 800;
    int worldHeight = 600;

    // reuse spatial hash to avoid reallocation each step
    SpatialHash grid;

    PhysicsWorld(float cellSize = 64.0f) : grid(cellSize) {}

    int addBody(const Body &b) {
        bodies.push_back(b);
        // ensure prevPosition is initialized for new body
        bodies.back().prevPosition = bodies.back().position;
        return (int)bodies.size() - 1;
    }

    // Narrowphase: circle-circle only
    bool circleCircleTest(int ia, int ib, Contact &out) {
        const Body &A = bodies[ia];
        const Body &B = bodies[ib];

        float rA = A.collider.radius;
        float rB = B.collider.radius;

        Vec2 n = B.position - A.position;
        float dist2 = n.length2();
        float radii = rA + rB;
        float radii2 = radii * radii;

        if (dist2 >= radii2) return false;

        float dist = std::sqrt(dist2);
        if (dist > 1e-6f) {
            out.normal = n / dist;
            out.contactPoint = A.position + out.normal * rA;
        } else {
            out.normal = Vec2{1,0};
            out.contactPoint = A.position;
            dist = 0.0f;
        }

        out.penetration = radii - dist;
        out.a = ia; out.b = ib;
        return true;
    }

    bool testPair(int i, int j, Contact &out) {
        // All colliders are circles now
        return circleCircleTest(i, j, out);
    }

    inline void resolveCollision(const Contact &c) {
        Body &A = bodies[c.a];
        Body &B = bodies[c.b];

        if (A.isStatic && B.isStatic) return;

        Vec2 rv = B.velocity - A.velocity;
        Vec2 n = c.normal.normalized();
        float velAlongNormal = rv.dot(n);

        if (velAlongNormal > 0.0f && ! (A.isStatic || B.isStatic)) return;

        float invMassA = A.invMass;
        float invMassB = B.invMass;
        float invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0f) return;

        float e = std::min(A.restitution, B.restitution);
        e = std::min(e, defaultRestitution);
        if (std::abs(velAlongNormal) < 0.5f && c.penetration < 0.5f) e = 0.0f;

        float j = -(1.0f + e) * velAlongNormal;
        j /= invMassSum;

        if (j > maxImpulse) j = maxImpulse;
        if (j < -maxImpulse) j = -maxImpulse;

        Vec2 impulse = n * j;

        if (!A.isStatic) A.velocity -= impulse * invMassA;
        if (!B.isStatic) B.velocity += impulse * invMassB;

        // positional correction (clamped)
        const float percent = positionalCorrectionPercent;
        const float slop = positionalCorrectionSlop;
        float penetration = c.penetration;
        float correctionMagnitude = std::max(penetration - slop, 0.0f) / invMassSum * percent;
        if (correctionMagnitude > maxPosCorrection) correctionMagnitude = maxPosCorrection;
        Vec2 correction = n * correctionMagnitude;
        if (!A.isStatic) A.position -= correction * invMassA;
        if (!B.isStatic) B.position += correction * invMassB;
    }

    void step() {
        const int N = (int)bodies.size();
        if (N == 0) return;

        // apply gravity
#pragma omp parallel for schedule(static)
        for (int i = 0; i < N; ++i) {
            Body &b = bodies[i];
            if (b.isStatic) continue;
            b.force += gravity * b.mass;
        }

        // integrate (semi-implicit Euler)
#pragma omp parallel for schedule(static)
        for (int i = 0; i < N; ++i) {
            Body &b = bodies[i];
            if (b.isStatic) {
                b.velocity = Vec2::zero();
                b.force = Vec2::zero();
                continue;
            }
            Vec2 accel = b.force * b.invMass;
            b.velocity += accel * dt;
            b.position += b.velocity * dt;
            b.velocity = b.velocity * damping;
            b.force = Vec2::zero();
        }

        // broadphase
        grid.clear();
        for (int i = 0; i < N; ++i) grid.insert(bodies, i);

        auto candidates = grid.getCandidatePairs();

        // narrowphase -- avoid global lock by using thread-local vectors then merge
        int numThreads = omp_get_max_threads();
        std::vector<std::vector<Contact>> localContacts(numThreads);

#pragma omp parallel
        {
            int tid = omp_get_thread_num();
#pragma omp for schedule(dynamic)
            for (int idx = 0; idx < (int)candidates.size(); ++idx) {
                auto p = candidates[idx];
                int i = p.first;
                int j = p.second;
                if (i == j) continue;
                if (bodies[i].isStatic && bodies[j].isStatic) continue;

                Contact c;
                if (testPair(i, j, c)) {
                    localContacts[tid].push_back(c);
                }
            }
        }

        // merge local contacts
        std::vector<Contact> contacts;
        size_t total = 0;
        for (const auto &lc : localContacts) total += lc.size();
        contacts.reserve(total);
        for (auto &lc : localContacts) {
            for (auto &c : lc) contacts.push_back(c);
        }

        // Also ensure dynamic bodies get tested against implicit world bounds
        for (int i = 0; i < N; ++i) {
            if (bodies[i].isStatic) continue;
            Body &b = bodies[i];
            float r = b.collider.radius;
            // left
            if (b.position.x - r < 0.0f) {
                Contact c; c.a = i; c.b = i; c.normal = Vec2{1,0}; c.penetration = (r - b.position.x); contacts.push_back(c);
            }
            // right
            if (b.position.x + r > worldWidth) {
                Contact c; c.a = i; c.b = i; c.normal = Vec2{-1,0}; c.penetration = (b.position.x + r - worldWidth); contacts.push_back(c);
            }
            // top
            if (b.position.y - r < 0.0f) {
                Contact c; c.a = i; c.b = i; c.normal = Vec2{0,1}; c.penetration = (r - b.position.y); contacts.push_back(c);
            }
            // bottom
            if (b.position.y + r > worldHeight) {
                Contact c; c.a = i; c.b = i; c.normal = Vec2{0,-1}; c.penetration = (b.position.y + r - worldHeight); contacts.push_back(c);
            }
        }

        // solve contacts iteratively
        int solverIterations = 8;
        for (int k = 0; k < solverIterations; ++k) {
            for (const auto &c : contacts) {
                // boundary contacts (a==b) are special: reflect velocity and correct position
                if (c.a == c.b) {
                    int i = c.a;
                    Body &b = bodies[i];
                    if (b.isStatic) continue;
                    Vec2 n = c.normal.normalized();
                    float velAlong = b.velocity.dot(n);
                    if (velAlong < 0.0f) b.velocity = b.velocity - n * (1.0f + b.restitution) * velAlong;
                    // positional correction
                    float invMass = b.invMass;
                    Vec2 corr = n * std::max(c.penetration - positionalCorrectionSlop, 0.0f) * positionalCorrectionPercent;
                    if (!b.isStatic) b.position += corr * invMass;
                } else {
                    resolveCollision(c);
                }
            }
        }

        // sanity clamps & invalid detection
        for (int i = 0; i < N; ++i) {
            Body &b = bodies[i];
            if (!std::isfinite(b.position.x) || !std::isfinite(b.position.y) ||
                !std::isfinite(b.velocity.x) || !std::isfinite(b.velocity.y)) {
                std::cerr << "[PhysicsWorld] Invalid body at " << i << " - resetting\n";
                b.position = b.prevPosition;
                b.velocity = Vec2::zero();
                b.force = Vec2::zero();
                continue;
            }
            b.clampSanity(1e6f, maxVelocity);
        }
    }
};