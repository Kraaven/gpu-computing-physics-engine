#pragma once
#include "Body.h"
#include <cmath>
#include <unordered_map>
#include <vector>

struct SpatialHash {
  float cellSize;
  std::unordered_map<long long, std::vector<int>> cells;

  SpatialHash(float cs = 64.0f) : cellSize(cs) {
    cells.reserve(1024);
  }

  void clear() { cells.clear(); }

  static inline long long hashCoords(int x, int y) {
    return ((long long)x << 32) ^ (y & 0xffffffff);
  }

  inline void worldToCell(const Vec2 &pos, int &cx, int &cy) const {
    cx = (int)std::floor(pos.x / cellSize);
    cy = (int)std::floor(pos.y / cellSize);
  }

  void insert(const std::vector<Body> &bodies, int index) {
    const Body &b = bodies[index];
    float r = b.collider.radius;
    Vec2 minPos = b.position - Vec2(r, r);
    Vec2 maxPos = b.position + Vec2(r, r);

    int minX = (int)std::floor(minPos.x / cellSize);
    int minY = (int)std::floor(minPos.y / cellSize);
    int maxX = (int)std::floor(maxPos.x / cellSize);
    int maxY = (int)std::floor(maxPos.y / cellSize);

    const int CLAMP_CELLS = 1024;
    if (minX < -CLAMP_CELLS) minX = -CLAMP_CELLS;
    if (minY < -CLAMP_CELLS) minY = -CLAMP_CELLS;
    if (maxX > CLAMP_CELLS) maxX = CLAMP_CELLS;
    if (maxY > CLAMP_CELLS) maxY = CLAMP_CELLS;

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        long long key = hashCoords(x, y);
        auto &vec = cells[key];
        vec.push_back(index);
      }
    }
  }

  std::vector<std::pair<int, int>> getCandidatePairs() const {
    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(256);

    const int offsets[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                               {1, 0},   {-1, 1}, {0, 1},  {1, 1}};

    for (const auto &kv : cells) {
      const long long key = kv.first;
      const auto &indices = kv.second;
      if (indices.empty()) continue;

      int cx = (int)(key >> 32);
      int cy = (int)(key & 0xffffffff);

      for (int n = 0; n < 9; ++n) {
        int nx = cx + offsets[n][0];
        int ny = cy + offsets[n][1];
        long long nkey = hashCoords(nx, ny);

        auto it = cells.find(nkey);
        if (it == cells.end()) continue;

        const auto &nindices = it->second;

        for (int i : indices) {
          for (int j : nindices) {
            if (i < j)
              pairs.emplace_back(i, j);
          }
        }
      }
    }

    return pairs;
  }

  // helper: remove indices of bodies outside rect area (inclusive)
  void removeOutsideArea(std::vector<Body> &bodies, float x, float y, float w, float h) {
    // naive implementation: iterate bodies and remove those outside. Caller must ensure safe erase semantics.
    // This is provided for convenience in the liquid scene.
    for (int i = (int)bodies.size() - 1; i >= 0; --i) {
      if (bodies[i].isStatic) continue;
      const Vec2 &p = bodies[i].position;
      if (p.x < x || p.x > x + w || p.y < y || p.y > y + h) {
        bodies.erase(bodies.begin() + i);
      }
    }
  }
};