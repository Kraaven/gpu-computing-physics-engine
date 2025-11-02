#pragma once
#include <cmath>
#include <iostream>

struct Vec2 {
  float x = 0.0f;
  float y = 0.0f;

  Vec2() = default;
  Vec2(float _x, float _y) : x(_x), y(_y) {}

  Vec2 operator+(const Vec2 &o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2 &o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(float s) const { return {x * s, y * s}; }
  Vec2 operator/(float s) const { return {x / s, y / s}; }

  Vec2 &operator+=(const Vec2 &o) {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2 &operator-=(const Vec2 &o) {
    x -= o.x;
    y -= o.y;
    return *this;
  }

  float length() const { return std::sqrt(x * x + y * y); }
  float length2() const { return x * x + y * y; }

  Vec2 normalized() const {
    float len = length();
    if (len > 1e-6f)
      return {x / len, y / len};
    return {0, 0};
  }

  float dot(const Vec2 &o) const { return x * o.x + y * o.y; }

  static Vec2 zero() { return {0, 0}; }

  friend std::ostream &operator<<(std::ostream &os, const Vec2 &v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
  }
};
