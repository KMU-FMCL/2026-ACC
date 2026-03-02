#pragma once

#include <cmath>

// ──────────────────────────────────────────────────────────────────────────────
// Shared 2-D vector type, arithmetic, and math helpers
// Used by bezier.hpp (global namespace) and ph_runtime.hpp (qcar2_control::ph)
// ──────────────────────────────────────────────────────────────────────────────

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 operator+(const Vec2 & a, const Vec2 & b) {return {a.x + b.x, a.y + b.y};}
inline Vec2 operator-(const Vec2 & a, const Vec2 & b) {return {a.x - b.x, a.y - b.y};}
inline Vec2 operator*(double s, const Vec2 & v) {return {s * v.x, s * v.y};}

inline double dot(const Vec2 & a, const Vec2 & b) {return a.x * b.x + a.y * b.y;}

inline double norm(const Vec2 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y);
}

inline Vec2 normalize(const Vec2 & v)
{
  const double n = norm(v);
  if (n < 1e-9) {return {0.0, 0.0};}
  return {v.x / n, v.y / n};
}

inline double wrapToPi(double a)
{
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTwoPi = 2.0 * kPi;
  a = std::fmod(a + kPi, kTwoPi);
  if (a < 0.0) {a += kTwoPi;}
  return a - kPi;
}
