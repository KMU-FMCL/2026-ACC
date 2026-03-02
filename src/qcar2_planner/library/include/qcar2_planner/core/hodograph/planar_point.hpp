/**
 * @file planar_point.hpp
 * @brief Declares the PlanarPoint aggregate struct for 2D geometric computations.
 */

#ifndef QCAR2_PLANNER_CORE_HODOGRAPH_PLANAR_POINT_HPP_
#define QCAR2_PLANNER_CORE_HODOGRAPH_PLANAR_POINT_HPP_

namespace qcar2_planner::core::hodograph {

/**
 * @brief A trivially copyable 2D point with arithmetic and geometric operations.
 *
 * PlanarPoint is a 16-byte aggregate struct holding two double-precision
 * coordinates. It supports aggregate initialization (`PlanarPoint{x, y}`),
 * component-wise arithmetic, and standard geometric operations.
 */
struct PlanarPoint {
  double x;  // NOLINT(misc-non-private-member-variables-in-classes)
  double y;  // NOLINT(misc-non-private-member-variables-in-classes)

  /**
   * @brief Adds two points component-wise.
   * @param[in] other The point to add.
   * @return A new PlanarPoint whose components are the sum of this and other.
   */
  [[nodiscard]] auto operator+(PlanarPoint other) const -> PlanarPoint;

  /**
   * @brief Subtracts another point component-wise.
   * @param[in] other The point to subtract.
   * @return A new PlanarPoint whose components are the difference of this and other.
   */
  [[nodiscard]] auto operator-(PlanarPoint other) const -> PlanarPoint;

  /**
   * @brief Scales both components by a scalar.
   * @param[in] scalar The scalar multiplier.
   * @return A new PlanarPoint with each component multiplied by scalar.
   */
  [[nodiscard]] auto operator*(double scalar) const -> PlanarPoint;

  /**
   * @brief Divides both components by a scalar.
   * @param[in] scalar The scalar divisor.
   * @return A new PlanarPoint with each component divided by scalar.
   */
  [[nodiscard]] auto operator/(double scalar) const -> PlanarPoint;

  /**
   * @brief Checks exact equality of two points.
   * @param[in] other The point to compare against.
   * @return True if both x and y components are exactly equal.
   * @note Uses exact floating-point comparison. Do not use on points computed
   *       through arithmetic — use Catch::Matchers::WithinRel in tests instead.
   */
  [[nodiscard]] auto operator==(PlanarPoint other) const -> bool;

  /**
   * @brief Checks inequality of two points.
   * @param[in] other The point to compare against.
   * @return True if any component differs.
   */
  [[nodiscard]] auto operator!=(PlanarPoint other) const -> bool;

  /**
   * @brief Computes the Euclidean distance to another point.
   * @param[in] other The target point.
   * @return The straight-line distance between this point and other.
   */
  [[nodiscard]] auto DistanceTo(PlanarPoint other) const -> double;

  /**
   * @brief Computes the dot product with another point treated as a vector.
   * @param[in] other The other vector.
   * @return The scalar dot product x*other.x + y*other.y.
   */
  [[nodiscard]] auto Dot(PlanarPoint other) const -> double;

  /**
   * @brief Computes the 2D scalar cross product with another point.
   * @param[in] other The other vector.
   * @return The scalar value x*other.y - y*other.x.
   */
  [[nodiscard]] auto Cross(PlanarPoint other) const -> double;

  /**
   * @brief Computes the Euclidean norm (magnitude) of the vector.
   * @return The length of the vector from the origin to this point.
   */
  [[nodiscard]] auto Norm() const -> double;
};

}  // namespace qcar2_planner::core::hodograph

#endif  // QCAR2_PLANNER_CORE_HODOGRAPH_PLANAR_POINT_HPP_
