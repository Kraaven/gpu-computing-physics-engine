#pragma once
#include "Body.h"
#include "SpatialHash.h"
#include <algorithm>
#include <omp.h>
#include <vector>
#include <mutex>
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

    PhysicsWorld() = default;

    int addBody(const Body &b) {
        bodies.push_back(b);
        // ensure prevPosition is initialized for new body
        bodies.back().prevPosition = bodies.back().position;
        return (int)bodies.size() - 1;
    }

    // Narrowphase tests
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

    bool circleAABBTest(int ic, int ibox, Contact &out) {
        const Body &C = bodies[ic];
        const Body &B = bodies[ibox];

        Vec2 aMin = B.position - B.collider.halfExtents;
        Vec2 aMax = B.position + B.collider.halfExtents;

        Vec2 closest;
        closest.x = std::max(aMin.x, std::min(C.position.x, aMax.x));
        closest.y = std::max(aMin.y, std::min(C.position.y, aMax.y));

        Vec2 n = C.position - closest;
        float dist2 = n.length2();
        float r = C.collider.radius;

        if (dist2 >= r*r) return false;

        float dist = std::sqrt(dist2);
        if (dist > 1e-6f) {
            out.normal = n / dist;
        } else {
            float left = std::abs(C.position.x - aMin.x);
            float right = std::abs(aMax.x - C.position.x);
            float top = std::abs(C.position.y - aMin.y);
            float bottom = std::abs(aMax.y - C.position.y);

            float minDist = left;
            out.normal = Vec2{-1,0};
            if (right < minDist) { minDist = right; out.normal = Vec2{1,0}; }
            if (top < minDist)   { minDist = top;   out.normal = Vec2{0,-1}; }
            if (bottom < minDist){ minDist = bottom;out.normal = Vec2{0,1}; }
        }

        out.penetration = r - dist;
        out.normal = out.normal * -1.0f; // convert to circle -> box
        out.contactPoint = closest;
        out.a = ic; out.b = ibox;
        return true;
    }

    bool testPair(int i, int j, Contact &out) {
        const Body &A = bodies[i];
        const Body &B = bodies[j];

        if (A.collider.type == ShapeType::Circle && B.collider.type == ShapeType::Circle) {
            return circleCircleTest(i, j, out);
        }
        if (A.collider.type == ShapeType::Circle && B.collider.type == ShapeType::AABB) {
            return circleAABBTest(i, j, out);
        }
        if (A.collider.type == ShapeType::AABB && B.collider.type == ShapeType::Circle) {
            bool ok = circleAABBTest(j, i, out);
            if (ok) {
                // out.a is circle index (j), out.b is box index (i). we want A->B normal convention; keep as produced.
                // For solver we'll use out.a/out.b as-is.
                return true;
            }
            return false;
        }
        if (A.collider.type == ShapeType::AABB && B.collider.type == ShapeType::AABB) {
            Vec2 amin = A.position - A.collider.halfExtents;
            Vec2 amax = A.position + A.collider.halfExtents;
            Vec2 bmin = B.position - B.collider.halfExtents;
            Vec2 bmax = B.position + B.collider.halfExtents;

            bool overlapX = (amin.x <= bmax.x) && (amax.x >= bmin.x);
            bool overlapY = (amin.y <= bmax.y) && (amax.y >= bmin.y);

            if (!overlapX || !overlapY) return false;

            float px = std::min(amax.x - bmin.x, bmax.x - amin.x);
            float py = std::min(amax.y - bmin.y, bmax.y - amin.y);

            out.a = i; out.b = j;
            if (px < py) {
                if (A.position.x < B.position.x) out.normal = Vec2{-1,0};
                else out.normal = Vec2{1,0};
                out.penetration = px;
            } else {
                if (A.position.y < B.position.y) out.normal = Vec2{0,-1};
                else out.normal = Vec2{0,1};
                out.penetration = py;
            }
            out.contactPoint = (A.position + B.position) * 0.5f;
            return true;
        }
        return false;
    }

    void resolveCollision(const Contact &c) {
        Body &A = bodies[c.a];
        Body &B = bodies[c.b];

        if (A.isStatic && B.isStatic) return;

        Vec2 rv = B.velocity - A.velocity;
        Vec2 n = c.normal.normalized();
        float velAlongNormal = rv.dot(n);

        // Skip if separating and both dynamic? We still want static-dynamic correct response
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
        // apply gravity
#pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            Body &b = bodies[i];
            if (b.isStatic) continue;
            b.force += gravity * b.mass;
        }

        // integrate (semi-implicit Euler)
#pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
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
        SpatialHash grid(64.0f);
        grid.clear();
        for (int i = 0; i < (int)bodies.size(); ++i) {
            grid.insert(bodies, i);
        }

        auto candidates = grid.getCandidatePairs();

        // narrowphase (parallel-safe accumulation)
        std::vector<Contact> contacts;
        contacts.reserve(candidates.size());
        std::mutex contactsMutex;

#pragma omp parallel for schedule(dynamic)
        for (int idx = 0; idx < (int)candidates.size(); ++idx) {
            auto p = candidates[idx];
            int i = p.first;
            int j = p.second;
            if (i == j) continue;
            if (bodies[i].isStatic && bodies[j].isStatic) continue;

            Contact c;
            if (testPair(i, j, c)) {
                std::lock_guard<std::mutex> lock(contactsMutex);
                contacts.push_back(c);
            }
        }

        // dynamic vs all static AABBs (ensures walls always tested)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            if (bodies[i].isStatic) continue;
            for (int j = 0; j < (int)bodies.size(); ++j) {
                if (!bodies[j].isStatic || bodies[j].collider.type != ShapeType::AABB) continue;
                Contact c;
                if (testPair(i, j, c)) contacts.push_back(c);
            }
        }

        // solve contacts iteratively
        int solverIterations = 8;
        for (int k = 0; k < solverIterations; ++k) {
            for (const auto &c : contacts) resolveCollision(c);
        }

        // sanity clamps & invalid detection
        for (int i = 0; i < (int)bodies.size(); ++i) {
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
