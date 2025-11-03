#pragma once
#include "Body.h"
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

struct SpatialHash {
  float cellSize;
  std::unordered_map<long long, std::vector<int>> cells;

  SpatialHash(float cs = 64.0f) : cellSize(cs) {
    cells.reserve(2048);
  }

  void clear() { 
    // Reuse allocated memory instead of clearing
    for (auto &kv : cells) {
      kv.second.clear();
    }
  }

  static inline long long hashCoords(int x, int y) {
    return ((long long)x << 32) | ((long long)y & 0xFFFFFFFF);
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
    minX = std::max(minX, -CLAMP_CELLS);
    minY = std::max(minY, -CLAMP_CELLS);
    maxX = std::min(maxX, CLAMP_CELLS);
    maxY = std::min(maxY, CLAMP_CELLS);

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        long long key = hashCoords(x, y);
        cells[key].push_back(index);
      }
    }
  }

  // OPTIMIZED: Use set to avoid duplicate pairs
  std::vector<std::pair<int, int>> getCandidatePairs() const {
    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(cells.size() * 4); // Better initial capacity

    // Use unordered_set to track unique pairs
    std::unordered_set<long long> seenPairs;
    seenPairs.reserve(cells.size() * 4);

    // Helper to create unique pair key
    auto makePairKey = [](int a, int b) -> long long {
      if (a > b) std::swap(a, b);
      return ((long long)a << 32) | (long long)b;
    };

    for (const auto &kv : cells) {
      const auto &indices = kv.second;
      if (indices.size() < 2) continue;

      // Check pairs within the same cell
      for (size_t i = 0; i < indices.size(); ++i) {
        for (size_t j = i + 1; j < indices.size(); ++j) {
          int a = indices[i];
          int b = indices[j];
          long long pairKey = makePairKey(a, b);
          
          if (seenPairs.insert(pairKey).second) {
            pairs.emplace_back(a, b);
          }
        }
      }
    }

    return pairs;
  }

  void removeOutsideArea(std::vector<Body> &bodies, float x, float y, float w, float h) {
    bodies.erase(
      std::remove_if(bodies.begin(), bodies.end(), 
        [x, y, w, h](const Body &b) {
          if (b.isStatic) return false;
          const Vec2 &p = b.position;
          return p.x < x || p.x > x + w || p.y < y || p.y > y + h;
        }),
      bodies.end()
    );
  }
};