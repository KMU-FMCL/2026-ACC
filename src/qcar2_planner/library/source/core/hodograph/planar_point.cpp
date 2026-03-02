#include "qcar2_planner/core/hodograph/planar_point.hpp"

#include <cmath>

namespace qcar2_planner::core::hodograph {

auto PlanarPoint::operator+(PlanarPoint other) const -> PlanarPoint {
  return PlanarPoint{x + other.x, y + other.y};
}

auto PlanarPoint::operator-(PlanarPoint other) const -> PlanarPoint {
  return PlanarPoint{x - other.x, y - other.y};
}

auto PlanarPoint::operator*(double scalar) const -> PlanarPoint {
  return PlanarPoint{x * scalar, y * scalar};
}

auto PlanarPoint::operator/(double scalar) const -> PlanarPoint {
  return PlanarPoint{x / scalar, y / scalar};
}

auto PlanarPoint::operator==(PlanarPoint other) const -> bool {
  return x == other.x && y == other.y;
}

auto PlanarPoint::operator!=(PlanarPoint other) const -> bool {
  return !(*this == other);
}

auto PlanarPoint::Norm() const -> double {
  return std::sqrt(x * x + y * y);
}

auto PlanarPoint::DistanceTo(PlanarPoint other) const -> double {
  return (*this - other).Norm();
}

auto PlanarPoint::Dot(PlanarPoint other) const -> double {
  return x * other.x + y * other.y;
}

auto PlanarPoint::Cross(PlanarPoint other) const -> double {
  return x * other.y - y * other.x;
}

}  // namespace qcar2_planner::core::hodograph
