#pragma once
#include "Body.h"
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <omp.h>

struct SpatialHash {
    float cellSize;
    std::unordered_map<long long, std::vector<int>> cells;
    
    // Thread-local storage for lock-free insertion
    std::vector<std::unordered_map<long long, std::vector<int>>> threadLocalCells;
    int numThreads;

    SpatialHash(float cs = 64.0f) : cellSize(cs) {
        cells.reserve(4096);
        numThreads = omp_get_max_threads();
        threadLocalCells.resize(numThreads);
        for (auto &tc : threadLocalCells) {
            tc.reserve(512);
        }
    }

    void clear() {
        for (auto &kv : cells) {
            kv.second.clear();
        }
        for (auto &tc : threadLocalCells) {
            for (auto &kv : tc) {
                kv.second.clear();
            }
        }
    }

    static inline long long hashCoords(int x, int y) {
        return ((long long)x << 32) | ((long long)y & 0xFFFFFFFF);
    }

    inline void worldToCell(const Vec2 &pos, int &cx, int &cy) const {
        cx = (int)std::floor(pos.x / cellSize);
        cy = (int)std::floor(pos.y / cellSize);
    }

    // Lock-free parallel insertion using thread-local storage
    void insertThreadSafe(const std::vector<Body> &bodies, int index) {
        int tid = omp_get_thread_num();
        auto &localCells = threadLocalCells[tid];
        
        const Body &b = bodies[index];
        float r = b.collider.radius;
        
        float minX = b.position.x - r;
        float minY = b.position.y - r;
        float maxX = b.position.x + r;
        float maxY = b.position.y + r;

        int cMinX = (int)std::floor(minX / cellSize);
        int cMinY = (int)std::floor(minY / cellSize);
        int cMaxX = (int)std::floor(maxX / cellSize);
        int cMaxY = (int)std::floor(maxY / cellSize);

        const int CLAMP = 1024;
        cMinX = std::max(cMinX, -CLAMP);
        cMinY = std::max(cMinY, -CLAMP);
        cMaxX = std::min(cMaxX, CLAMP);
        cMaxY = std::min(cMaxY, CLAMP);

        for (int cy = cMinY; cy <= cMaxY; ++cy) {
            for (int cx = cMinX; cx <= cMaxX; ++cx) {
                long long key = hashCoords(cx, cy);
                localCells[key].push_back(index);
            }
        }
    }

    // Merge thread-local cells into main cell map
    void mergeThreadLocalCells() {
        for (auto &localCells : threadLocalCells) {
            for (auto &kv : localCells) {
                long long key = kv.first;
                auto &indices = kv.second;
                
                cells[key].insert(cells[key].end(), indices.begin(), indices.end());
            }
        }
    }

    // Original single-threaded insert
    void insert(const std::vector<Body> &bodies, int index) {
        const Body &b = bodies[index];
        float r = b.collider.radius;
        
        float minX = b.position.x - r;
        float minY = b.position.y - r;
        float maxX = b.position.x + r;
        float maxY = b.position.y + r;

        int cMinX = (int)std::floor(minX / cellSize);
        int cMinY = (int)std::floor(minY / cellSize);
        int cMaxX = (int)std::floor(maxX / cellSize);
        int cMaxY = (int)std::floor(maxY / cellSize);

        const int CLAMP = 1024;
        cMinX = std::max(cMinX, -CLAMP);
        cMinY = std::max(cMinY, -CLAMP);
        cMaxX = std::min(cMaxX, CLAMP);
        cMaxY = std::min(cMaxY, CLAMP);

        for (int cy = cMinY; cy <= cMaxY; ++cy) {
            for (int cx = cMinX; cx <= cMaxX; ++cx) {
                long long key = hashCoords(cx, cy);
                cells[key].push_back(index);
            }
        }
    }

    // Optimized candidate pair generation
    std::vector<std::pair<int, int>> getCandidatePairs() const {
        std::vector<std::pair<int, int>> pairs;
        
        // Estimate capacity
        size_t estimatedPairs = 0;
        for (const auto &kv : cells) {
            size_t n = kv.second.size();
            if (n > 1) {
                estimatedPairs += (n * (n - 1)) / 2;
            }
        }
        pairs.reserve(estimatedPairs);

        std::unordered_set<long long> seenPairs;
        seenPairs.reserve(estimatedPairs);

        auto makePairKey = [](int a, int b) -> long long {
            if (a > b) std::swap(a, b);
            return ((long long)a << 32) | (long long)b;
        };

        for (const auto &kv : cells) {
            const auto &indices = kv.second;
            size_t n = indices.size();
            
            if (n < 2) continue;

            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
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
                    return b.position.x < x || b.position.x > x + w ||
                           b.position.y < y || b.position.y > y + h;
                }),
            bodies.end()
        );
    }
};