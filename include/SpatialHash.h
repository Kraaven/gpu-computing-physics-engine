#pragma once
#include "Body.h"
#include <cmath>
#include <unordered_map>
#include <vector>

struct SpatialHash {
  float cellSize;
  std::unordered_map<long long, std::vector<int>> cells;

  SpatialHash(float cs = 64.0f) : cellSize(cs) {}

  void clear() { cells.clear(); }

  // Hash function for (x, y) integer grid coords → 64-bit key
  long long hashCoords(int x, int y) const {
    return ((long long)x << 32) ^ (y & 0xffffffff);
  }

  // Converts world position → cell coordinate
  inline void worldToCell(const Vec2 &pos, int &cx, int &cy) const {
    cx = (int)std::floor(pos.x / cellSize);
    cy = (int)std::floor(pos.y / cellSize);
  }

  // Insert body index into all cells it overlaps
  void insert(const std::vector<Body> &bodies, int index) {
    const Body &b = bodies[index];
    Vec2 minPos, maxPos;

    if (b.collider.type == ShapeType::Circle) {
      float r = b.collider.radius;
      minPos = b.position - Vec2(r, r);
      maxPos = b.position + Vec2(r, r);
    } else {
      minPos = b.position - b.collider.halfExtents;
      maxPos = b.position + b.collider.halfExtents;
    }

    int minX = (int)std::floor(minPos.x / cellSize);
    int minY = (int)std::floor(minPos.y / cellSize);
    int maxX = (int)std::floor(maxPos.x / cellSize);
    int maxY = (int)std::floor(maxPos.y / cellSize);

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        long long key = hashCoords(x, y);
        cells[key].push_back(index);
      }
    }
  }

  // Generate all candidate pairs from neighboring cells
  std::vector<std::pair<int, int>> getCandidatePairs() const {
    std::vector<std::pair<int, int>> pairs;
    pairs.reserve(256);

    // Neighbor offsets (9 total including self)
    const int offsets[9][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                               {1, 0},   {-1, 1}, {0, 1},  {1, 1}};

    for (const auto &[key, indices] : cells) {
      if (indices.empty())
        continue;

      // Decode cell coords
      int cx = (int)(key >> 32);
      int cy = (int)(key & 0xffffffff);

      // For each neighbor
      for (int n = 0; n < 9; ++n) {
        int nx = cx + offsets[n][0];
        int ny = cy + offsets[n][1];
        long long nkey = hashCoords(nx, ny);

        auto it = cells.find(nkey);
        if (it == cells.end())
          continue;

        const auto &nindices = it->second;

        // Generate pairs
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
};
