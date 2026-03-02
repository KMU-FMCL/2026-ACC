/**
 * @file gaussian.hpp
 * @brief Declares the Gaussian class for a Gaussian radial basis function kernel.
 */

#ifndef QCAR2_PLANNER_CORE_RBF_GAUSSIAN_HPP_
#define QCAR2_PLANNER_CORE_RBF_GAUSSIAN_HPP_

#include <cstdint>

#include "playground/core/error/result.hpp"

namespace qcar2_planner::core::rbf {

/**
 * @brief Gaussian radial basis function kernel φ(ξ) = e^(-ε·(ξ-c)²).
 *
 * @details A lightweight value type holding the shape parameter ε and center c
 * for a Gaussian RBF kernel. The kernel evaluates to 1.0 at the center and
 * decays symmetrically toward zero. No heap allocation is performed; instances
 * are trivially copyable via compiler-generated copy and move operations.
 */
class Gaussian {
 public:
  /** @brief Error codes for Gaussian operations. */
  enum class ErrorType : std::int8_t {
    kInvalidEpsilon  ///< The shape parameter ε is not strictly positive.
  };

  /**
   * @brief Creates a Gaussian kernel with the given shape parameter and center.
   * @param[in] epsilon Shape parameter ε; must be strictly positive (ε > 0).
   * @param[in] center Kernel center c; any finite real value is valid.
   * @return Result<Gaussian> — failure with kInvalidEpsilon if epsilon ≤ 0.
   */
  [[nodiscard]] static auto Create(double epsilon,
                                   double center) -> playground::core::Result<Gaussian>;

  /**
   * @brief Evaluates the kernel at ξ: φ(ξ) = e^(-ε·(ξ-c)²).
   * @param[in] xi_val Evaluation point.
   * @return The kernel value in (0, 1].
   */
  [[nodiscard]] auto Evaluate(double xi_val) const -> double;

  /**
   * @brief Returns the shape parameter ε.
   * @return The epsilon value supplied at construction.
   */
  [[nodiscard]] auto Epsilon() const -> double;

  /**
   * @brief Returns the kernel center c.
   * @return The center value supplied at construction.
   */
  [[nodiscard]] auto Center() const -> double;

 private:
  explicit Gaussian(double epsilon, double center);
  double epsilon_;
  double center_;
};

}  // namespace qcar2_planner::core::rbf

#endif  // QCAR2_PLANNER_CORE_RBF_GAUSSIAN_HPP_
