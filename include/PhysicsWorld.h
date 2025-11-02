#pragma once
#include "Body.h"
#include "SpatialHash.h"
#include <algorithm>
#include <omp.h>
#include <vector>
#include <mutex>
#include <iostream>


// Contact (collision) info
struct Contact {
  int a;             // index of body A
  int b;             // index of body B
  Vec2 normal;       // from A -> B (points toward B)
  float penetration; // penetration depth (positive)
  Vec2 contactPoint; // optional approximate point for debug
};

class PhysicsWorld {
public:
  std::vector<Body> bodies;
  Vec2 gravity = {0.0f, 300.0f};
  float damping = 0.999f; // slight damping
  float dt = 1.0f / 120.0f;

  // solver params
  float defaultRestitution = 0.6f;
  float positionalCorrectionPercent = 0.4f;
  float positionalCorrectionSlop = 0.02f;

    // safety limits
  float maxVelocity = 3000.0f;        // clamp velocity magnitude
  float maxImpulse = 20000.0f;        // clamp single impulse magnitude
  float maxPosCorrection = 200.0f;    // limit positional correction in pixels


  PhysicsWorld() = default;

  int addBody(const Body &b) {
    bodies.push_back(b);
    return (int)bodies.size() - 1;
  }

  // --------------------
  // Narrowphase tests
  // --------------------
  bool circleCircleTest(int ia, int ib, Contact &out) {
    const Body &A = bodies[ia];
    const Body &B = bodies[ib];

    float rA = A.collider.radius;
    float rB = B.collider.radius;

    Vec2 n = B.position - A.position;
    float dist2 = n.length2();
    float radii = rA + rB;
    float radii2 = radii * radii;

    if (dist2 >= radii2)
      return false; // no collision

    float dist = std::sqrt(dist2);

    if (dist > 1e-6f) {
      out.normal = n / dist; // normalized A->B
      out.contactPoint = A.position + out.normal * rA;
    } else {
      out.normal = Vec2{1.0f, 0.0f};
      out.contactPoint = A.position;
      dist = 0.0f;
    }

    out.penetration = radii - dist;
    out.a = ia;
    out.b = ib;
    return true;
  }

  bool circleAABBTest(int ic, int ibox, Contact &out) {
    const Body &C = bodies[ic];
    const Body &B = bodies[ibox]; // AABB

    Vec2 aMin = B.position - B.collider.halfExtents;
    Vec2 aMax = B.position + B.collider.halfExtents;

    Vec2 closest;
    closest.x = std::max(aMin.x, std::min(C.position.x, aMax.x));
    closest.y = std::max(aMin.y, std::min(C.position.y, aMax.y));

    Vec2 n = C.position - closest;
    float dist2 = n.length2();
    float r = C.collider.radius;

    if (dist2 >= r * r)
      return false; // no collision

    float dist = std::sqrt(dist2);
    if (dist > 1e-6f) {
      out.normal = n / dist;
    } else {
      float left = std::abs(C.position.x - aMin.x);
      float right = std::abs(aMax.x - C.position.x);
      float top = std::abs(C.position.y - aMin.y);
      float bottom = std::abs(aMax.y - C.position.y);

      float minDist = left;
      out.normal = Vec2{-1, 0};

      if (right < minDist) {
        minDist = right;
        out.normal = Vec2{1, 0};
      }
      if (top < minDist) {
        minDist = top;
        out.normal = Vec2{0, -1};
      }
      if (bottom < minDist) {
        minDist = bottom;
        out.normal = Vec2{0, 1};
      }
    }

    out.penetration = r - (dist); // positive
    // convert normal to point from circle -> box
    out.normal = out.normal * -1.0f;
    out.contactPoint = closest;
    out.a = ic;
    out.b = ibox;
    return true;
  }

  bool testPair(int i, int j, Contact &out) {
    const Body &A = bodies[i];
    const Body &B = bodies[j];

    if (A.collider.type == ShapeType::Circle &&
        B.collider.type == ShapeType::Circle) {
      return circleCircleTest(i, j, out);
    }

    if (A.collider.type == ShapeType::Circle &&
        B.collider.type == ShapeType::AABB) {
      return circleAABBTest(i, j, out);
    }
    if (A.collider.type == ShapeType::AABB &&
        B.collider.type == ShapeType::Circle) {
      bool ok = circleAABBTest(j, i, out);
      if (ok) {
        // out has a = circle, b = box; solver expects a->b normal convention used
        // We keep that and solver uses c.a / c.b as provided.
        return true;
      }
      return false;
    }

    if (A.collider.type == ShapeType::AABB &&
        B.collider.type == ShapeType::AABB) {
      Vec2 amin = A.position - A.collider.halfExtents;
      Vec2 amax = A.position + A.collider.halfExtents;
      Vec2 bmin = B.position - B.collider.halfExtents;
      Vec2 bmax = B.position + B.collider.halfExtents;

      bool overlapX = (amin.x <= bmax.x) && (amax.x >= bmin.x);
      bool overlapY = (amin.y <= bmax.y) && (amax.y >= bmin.y);

      if (!overlapX || !overlapY)
        return false;

      float px = std::min(amax.x - bmin.x, bmax.x - amin.x);
      float py = std::min(amax.y - bmin.y, bmax.y - amin.y);

      out.a = i;
      out.b = j;
      if (px < py) {
        if (A.position.x < B.position.x)
          out.normal = Vec2{-1, 0};
        else
          out.normal = Vec2{1, 0};
        out.penetration = px;
      } else {
        if (A.position.y < B.position.y)
          out.normal = Vec2{0, -1};
        else
          out.normal = Vec2{0, 1};
        out.penetration = py;
      }
      out.contactPoint = (A.position + B.position) * 0.5f;
      return true;
    }

    return false;
  }

  // --------------------
  // Collision resolution: impulse + positional correction
  // --------------------
    void resolveCollision(const Contact &c) {
    Body &A = bodies[c.a];
    Body &B = bodies[c.b];

    // Relative velocity (vB - vA)
    Vec2 rv = B.velocity - A.velocity;

    // Ensure normal points A -> B
    Vec2 n = c.normal.normalized();

    float velAlongNormal = rv.dot(n);

    // Do not resolve if velocities are separating
    if (velAlongNormal > 0.0f)
      return;

    float invMassA = A.invMass;
    float invMassB = B.invMass;
    float invMassSum = invMassA + invMassB;
    if (invMassSum <= 0.0f)
      return; // both static

    // Restitution: min of body or world defaults
    float e = std::min(A.restitution, B.restitution);
    e = std::min(e, defaultRestitution);

    // If contact is resting (small relative velocity), reduce bounce
    if (std::abs(velAlongNormal) < 0.5f && c.penetration < 0.5f) {
      e = 0.0f;
    }

    // Impulse scalar
    float j = -(1.0f + e) * velAlongNormal;
    j /= invMassSum;

    // clamp impulse to avoid explosion on deep penetrations
    if (j > maxImpulse) j = maxImpulse;
    if (j < -maxImpulse) j = -maxImpulse;

    Vec2 impulse = n * j;

    if (!A.isStatic)
      A.velocity -= impulse * invMassA;
    if (!B.isStatic)
      B.velocity += impulse * invMassB;

    // Positional correction to avoid sinking (baumgarte-like)
    const float percent = positionalCorrectionPercent; // usually 20% - 80%
    const float slop = positionalCorrectionSlop;       // penetration allowance
    float penetration = c.penetration;
    float correctionMagnitude =
        std::max(penetration - slop, 0.0f) / invMassSum * percent;

    // clamp correction magnitude to avoid teleporting
    if (correctionMagnitude > maxPosCorrection) correctionMagnitude = maxPosCorrection;
    Vec2 correction = n * correctionMagnitude;
    if (!A.isStatic)
      A.position -= correction * invMassA;
    if (!B.isStatic)
      B.position += correction * invMassB;
  }


  // --------------------
  // Main step: apply forces, integrate, detect & solve collisions
  // --------------------
  void step() {
    // --- 0) Ensure prevPosition set already by main loop (we also fallback here)
    for (auto &b : bodies) {
      if (b.prevPosition.x == 0 && b.prevPosition.y == 0) b.prevPosition = b.position;
    }

    // --- 1) Apply gravity (parallel)
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)bodies.size(); ++i) {
      Body &b = bodies[i];
      if (b.isStatic)
        continue;
      b.force += gravity * b.mass;
    }

    // --- 2) Integrate (parallel) : semi-implicit Euler
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

    // --- 3) Broadphase (spatial hash)
    SpatialHash grid(64.0f);
    grid.clear();

    for (int i = 0; i < (int)bodies.size(); ++i) {
      // still insert dynamic + non-huge statics (ground may be huge AABB)
      grid.insert(bodies, i);
    }

    auto candidates = grid.getCandidatePairs();

    // --- 4) Narrowphase on candidate pairs (parallel safe accumulation)
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

    // b) dynamic vs ALL static AABBs (support ground)
    for (int i = 0; i < (int)bodies.size(); ++i) {
      if (bodies[i].isStatic)
        continue;
      for (int j = 0; j < (int)bodies.size(); ++j) {
        if (!bodies[j].isStatic || bodies[j].collider.type != ShapeType::AABB)
          continue;

        Contact c;
        if (testPair(i, j, c)) contacts.push_back(c);
      }
    }

    // --- 5) Solve all contacts with multiple iterations
    int solverIterations = 8; // 5–10 recommended
    for (int k = 0; k < solverIterations; ++k) {
      for (const auto &c : contacts)
        resolveCollision(c);
    }

        // --- 6) Sanity clamp (prevent NaNs / runaway values)
    for (int i = 0; i < (int)bodies.size(); ++i) {
      Body &b = bodies[i];
      // If invalid numbers appear, reset to prevPosition
      if (!std::isfinite(b.position.x) || !std::isfinite(b.position.y) ||
          !std::isfinite(b.velocity.x) || !std::isfinite(b.velocity.y)) {
        std::cerr << "[PhysicsWorld] Invalid body detected at index " << i << " — resetting.\n";
        b.position = b.prevPosition;
        b.velocity = Vec2::zero();
        b.force = Vec2::zero();
        continue;
      }

      // clamp velocity and positions
      b.clampSanity(/*maxPos=*/1e6f, /*maxVel=*/maxVelocity);
    }

  }
};
