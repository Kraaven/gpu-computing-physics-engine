#pragma once
#include <vector>
#include <algorithm>
#include <omp.h>
#include "Body.h"

// Contact (collision) info
struct Contact {
    int a;              // index of body A
    int b;              // index of body B
    Vec2 normal;        // from A -> B (points toward B)
    float penetration;  // penetration depth (positive)
};

class PhysicsWorld {
public:
    std::vector<Body> bodies;
    Vec2 gravity = {0.0f, 300.0f};
    float damping = 0.999f;      // slight damping
    float dt = 1.0f / 120.0f;

    // solver params
    float defaultRestitution = 0.6f;
    float positionalCorrectionPercent = 0.8f;
    float positionalCorrectionSlop = 0.01f;

    PhysicsWorld() = default;

    int addBody(const Body& b) {
        bodies.push_back(b);
        return (int)bodies.size() - 1;
    }

    // --------------------
    // Narrowphase tests
    // --------------------
    // circle-circle: returns true if overlapping and fills contact
    bool circleCircleTest(int ia, int ib, Contact &out) {
        const Body &A = bodies[ia];
        const Body &B = bodies[ib];

        float rA = A.collider.radius;
        float rB = B.collider.radius;

        Vec2 n = B.position - A.position;
        float dist2 = n.length2();
        float radii = rA + rB;
        float radii2 = radii * radii;

        if (dist2 >= radii2) return false; // no collision

        float dist = std::sqrt(dist2);

        // Avoid divide by zero
        if (dist > 1e-6f) {
            out.normal = n / dist; // normalized
        } else {
            // choose arbitrary (avoid NaNs)
            out.normal = Vec2{1.0f, 0.0f};
            dist = 0.0f;
        }

        out.penetration = radii - dist;
        out.a = ia;
        out.b = ib;
        return true;
    }

    // circle-AABB: returns true if overlapping and fills contact
    bool circleAABBTest(int ic, int ibox, Contact &out) {
        const Body &C = bodies[ic];
        const Body &B = bodies[ibox]; // treated as AABB (centered)

        // AABB center and half extents
        Vec2 aMin = B.position - B.collider.halfExtents;
        Vec2 aMax = B.position + B.collider.halfExtents;

        // Closest point on AABB to circle center
        Vec2 closest;
        closest.x = std::max(aMin.x, std::min(C.position.x, aMax.x));
        closest.y = std::max(aMin.y, std::min(C.position.y, aMax.y));

        Vec2 n = C.position - closest;
        float dist2 = n.length2();
        float r = C.collider.radius;

        if (dist2 >= r * r) return false; // no collision

        float dist = std::sqrt(dist2);
        if (dist > 1e-6f) {
            out.normal = n / dist; // points from box toward circle center
        } else {
            // circle center is exactly on the box (or inside center) -- pick axis of minimum penetration
            // compute distances to each face
            float left = std::abs(C.position.x - aMin.x);
            float right = std::abs(aMax.x - C.position.x);
            float top = std::abs(C.position.y - aMin.y);
            float bottom = std::abs(aMax.y - C.position.y);

            float minDist = left;
            out.normal = Vec2{-1, 0};

            if (right < minDist) { minDist = right; out.normal = Vec2{1, 0}; }
            if (top < minDist) { minDist = top; out.normal = Vec2{0, -1}; }
            if (bottom < minDist) { minDist = bottom; out.normal = Vec2{0, 1}; }
        }

        out.penetration = r - (dist); // positive
        // we want normal from circle A -> box B in our solver convention (A->B)
        // out.normal currently points from box -> circle center; so invert to point from circle -> box
        out.normal = out.normal * -1.0f;

        out.a = ic;    // circle
        out.b = ibox;  // box
        return true;
    }

    // wrapper to test pair (assumes both indices valid) and fill contact if collision
    bool testPair(int i, int j, Contact &out) {
        const Body &A = bodies[i];
        const Body &B = bodies[j];

        // circle - circle
        if (A.collider.type == ShapeType::Circle && B.collider.type == ShapeType::Circle) {
            return circleCircleTest(i, j, out);
        }

        // circle - AABB (either order)
        if (A.collider.type == ShapeType::Circle && B.collider.type == ShapeType::AABB) {
            return circleAABBTest(i, j, out);
        }
        if (A.collider.type == ShapeType::AABB && B.collider.type == ShapeType::Circle) {
            // swap so out.a is circle index (our circleAABBTest expects circle first)
            bool ok = circleAABBTest(j, i, out);
            if (ok) {
                // out currently has a = circleIndex, b = boxIndex
                // but the solver expects a = first, b = second (we can keep as-is)
                // we'll ensure solver uses out.a/out.b correctly
                // For consistency, we return as-is (where a is circle, b is box)
                return true;
            }
            return false;
        }

        // AABB - AABB: simple overlap check (axis-aligned)
        if (A.collider.type == ShapeType::AABB && B.collider.type == ShapeType::AABB) {
            Vec2 amin = A.position - A.collider.halfExtents;
            Vec2 amax = A.position + A.collider.halfExtents;
            Vec2 bmin = B.position - B.collider.halfExtents;
            Vec2 bmax = B.position + B.collider.halfExtents;

            bool overlapX = (amin.x <= bmax.x) && (amax.x >= bmin.x);
            bool overlapY = (amin.y <= bmax.y) && (amax.y >= bmin.y);

            if (!overlapX || !overlapY) return false;

            // Compute minimal penetration axis & normal
            float px = std::min(amax.x - bmin.x, bmax.x - amin.x);
            float py = std::min(amax.y - bmin.y, bmax.y - amin.y);

            out.a = i; out.b = j;
            if (px < py) {
                // x axis separation smaller
                if (A.position.x < B.position.x) out.normal = Vec2{-1, 0}; else out.normal = Vec2{1, 0};
                out.penetration = px;
            } else {
                if (A.position.y < B.position.y) out.normal = Vec2{0, -1}; else out.normal = Vec2{0, 1};
                out.penetration = py;
            }
            // Make normal point A -> B
            // We chose sign above accordingly.
            return true;
        }

        return false; // unsupported types
    }

    // --------------------
    // Collision resolution: impulse + positional correction
    // --------------------
    void resolveCollision(const Contact &c) {
        Body &A = bodies[c.a];
        Body &B = bodies[c.b];

        // Relative velocity (vB - vA)
        Vec2 rv = B.velocity - A.velocity;

        // normal should point from A -> B
        Vec2 n = c.normal.normalized();

        float velAlongNormal = rv.dot(n);

        // Do not resolve if velocities are separating
        if (velAlongNormal > 0.0f) return;

        // Restitution (bounciness): use min of the two
        float e = std::min(defaultRestitution, defaultRestitution);
        // (Optionally, bodies can hold their own restitution; using world default for now)

        float invMassA = A.invMass;
        float invMassB = B.invMass;
        float invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0f) return; // both static

        // Impulse scalar
        float j = -(1.0f + e) * velAlongNormal;
        j /= invMassSum;

        Vec2 impulse = n * j;

        if (!A.isStatic) A.velocity -= impulse * invMassA;
        if (!B.isStatic) B.velocity += impulse * invMassB;

        // Positional correction to avoid sinking (baumgarte-like)
        const float percent = positionalCorrectionPercent; // usually 20% - 80%
        const float slop = positionalCorrectionSlop;       // penetration allowance
        float penetration = c.penetration;
        float correctionMagnitude = std::max(penetration - slop, 0.0f) / invMassSum * percent;
        Vec2 correction = n * correctionMagnitude;
        if (!A.isStatic) A.position -= correction * invMassA;
        if (!B.isStatic) B.position += correction * invMassB;
    }

    // --------------------
    // Main step: apply forces, integrate, detect & solve collisions
    // --------------------
    void step() {
        // --- 1) Apply gravity (parallel)
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            Body &b = bodies[i];
            if (b.isStatic) continue;
            b.force += gravity * b.mass;
        }

        // --- 2) Integrate (parallel) : semi-implicit Euler
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            Body &b = bodies[i];
            if (b.isStatic) {
                // ensure velocities are zero for static bodies
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

        // --- 3) Broad/narrow phase (brute-force for now) and collect contacts
        std::vector<Contact> contacts;
        contacts.reserve(64);

        int N = (int)bodies.size();
        for (int i = 0; i < N; ++i) {
            for (int j = i + 1; j < N; ++j) {
                // Skip pairs where both are static
                if (bodies[i].isStatic && bodies[j].isStatic) continue;

                Contact c;
                if (testPair(i, j, c)) {
                    // Ensure contact's a,b map to the correct indices used in solver:
                    // our tests sometimes return (a=circle, b=box) etc. That's fine: solver uses c.a/c.b.
                    contacts.push_back(c);
                }
            }
        }

        // --- 4) Resolve contacts (single threaded)
        for (const auto &c : contacts) {
            resolveCollision(c);
        }
    }

};
