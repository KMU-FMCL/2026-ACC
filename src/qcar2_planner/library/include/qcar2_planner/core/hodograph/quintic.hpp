/**
 * @file quintic.hpp
 * @brief Declares the Quintic class for PH quintic curve construction from G1 Hermite data.
 */

#ifndef QCAR2_PLANNER_CORE_HODOGRAPH_QUINTIC_HPP_
#define QCAR2_PLANNER_CORE_HODOGRAPH_QUINTIC_HPP_

#include <cstdint>
#include <memory>

#include "playground/core/error/result.hpp"
#include "qcar2_planner/core/hodograph/planar_point.hpp"

namespace qcar2_planner::core::hodograph {

/**
 * @brief Quintic Pythagorean Hodograph curve constructed from G1 Hermite data.
 *
 * @details Provides mathematically exact arc length computation, rational unit
 * normals, and point evaluation across four solution branches. Hodograph
 * coefficients and the speed polynomial (sigma) are stored via Pimpl,
 * hiding all computation details from public headers.
 *
 * The class is move-only. Use the static Interpolate factories to construct
 * instances.
 */
class Quintic {
 public:
  /** @brief Error codes for Quintic operations. */
  enum class ErrorType : std::int8_t {
    kInvalidHermiteData,  ///< The Hermite data is invalid for other unspecified reasons.
    kCoincidentPoints,  ///< Start and end points are coincident.
    kZeroTangent,  ///< A tangent vector has zero length.
    kInvalidParameter,  ///< Evaluation parameter is outside [0, 1].
    kInvalidBranch  ///< Branch index is outside [0, 3].
  };

  /**
   * @brief Constructs a quintic PH curve using the minimum arc length branch.
   * @param[in] start The start point of the curve.
   * @param[in] end The end point of the curve.
   * @param[in] tangent_start Unit tangent direction at the start point.
   * @param[in] tangent_end Unit tangent direction at the end point.
   * @return Result<Quintic> — failure with kCoincidentPoints if start equals end,
   *         kZeroTangent if either tangent has zero length.
   */
  [[nodiscard]] static auto Interpolate(PlanarPoint start, PlanarPoint end,
                                        PlanarPoint tangent_start, PlanarPoint tangent_end)
      -> playground::core::Result<Quintic>;

  /**
   * @brief Constructs a quintic PH curve using an explicitly specified solution branch.
   * @param[in] start The start point of the curve.
   * @param[in] end The end point of the curve.
   * @param[in] tangent_start Unit tangent direction at the start point.
   * @param[in] tangent_end Unit tangent direction at the end point.
   * @param[in] branch Branch index in [0, 3] corresponding to sign combinations (++, +-, -+, --).
   * @return Result<Quintic> — failure with kCoincidentPoints if start equals end,
   *         kZeroTangent if either tangent has zero length,
   *         kInvalidBranch if branch is outside [0, 3].
   */
  [[nodiscard]] static auto Interpolate(PlanarPoint start, PlanarPoint end,
                                        PlanarPoint tangent_start, PlanarPoint tangent_end,
                                        int branch) -> playground::core::Result<Quintic>;

  /**
   * @brief Evaluates the curve position at the given parameter.
   * @param[in] param Evaluation parameter in [0, 1].
   * @return Result<PlanarPoint> — failure with kInvalidParameter if param is outside [0, 1].
   */
  [[nodiscard]] auto Evaluate(double param) const -> playground::core::Result<PlanarPoint>;

  /**
   * @brief Computes the unit tangent vector at the given parameter.
   * @param[in] param Evaluation parameter in [0, 1].
   * @return Result<PlanarPoint> — failure with kInvalidParameter if param is outside [0, 1].
   */
  [[nodiscard]] auto Tangent(double param) const -> playground::core::Result<PlanarPoint>;

  /**
   * @brief Computes the unit normal vector at the given parameter.
   * @details The normal points to the left of the curve direction (i.e., n = (-y', x') / sigma).
   * @param[in] param Evaluation parameter in [0, 1].
   * @return Result<PlanarPoint> — failure with kInvalidParameter if param is outside [0, 1].
   */
  [[nodiscard]] auto Normal(double param) const -> playground::core::Result<PlanarPoint>;

  /**
   * @brief Computes the exact arc length from param=0 to param.
   * @details Arc length is computed exactly via polynomial integration of the speed
   *          polynomial sigma — no numerical quadrature is required.
   * @param[in] param Evaluation parameter in [0, 1].
   * @return Result<double> — failure with kInvalidParameter if param is outside [0, 1].
   */
  [[nodiscard]] auto ArcLength(double param) const -> playground::core::Result<double>;

  /**
   * @brief Computes the total arc length of the curve.
   * @details Equivalent to ArcLength(1.0).
   * @return Result<double> containing the total arc length.
   */
  [[nodiscard]] auto TotalLength() const -> playground::core::Result<double>;

  Quintic(Quintic&&) noexcept;
  auto operator=(Quintic&&) noexcept -> Quintic&;
  ~Quintic();

  Quintic(const Quintic&) = delete;
  auto operator=(const Quintic&) -> Quintic& = delete;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  explicit Quintic(std::unique_ptr<Impl> impl);
};

}  // namespace qcar2_planner::core::hodograph

#endif  // QCAR2_PLANNER_CORE_HODOGRAPH_QUINTIC_HPP_
