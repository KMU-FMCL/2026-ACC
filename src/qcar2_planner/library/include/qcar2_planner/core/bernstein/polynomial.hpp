/**
 * @file polynomial.hpp
 * @brief Declares the Polynomial class for Bernstein-form polynomial arithmetic.
 */

#ifndef QCAR2_PLANNER_CORE_BERNSTEIN_POLYNOMIAL_HPP_
#define QCAR2_PLANNER_CORE_BERNSTEIN_POLYNOMIAL_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "playground/core/error/result.hpp"
#include "playground/core/memory/allocator.hpp"

namespace qcar2_planner::core::bernstein {

/**
 * @brief Bernstein-form polynomial with coefficient storage and arithmetic operations.
 *
 * Polynomial stores coefficients for a degree-n polynomial in Bernstein form
 * using an Eigen::VectorXd behind a Pimpl firewall. Construction, evaluation,
 * and degree-changing operations are all exposed through the public API.
 * All Eigen storage is hidden in the .cpp file.
 */
class Polynomial {
 public:
  /** @brief Error codes for Polynomial operations. */
  enum class ErrorType : std::int8_t {
    kInvalidCoefficients,  ///< Empty coefficient vector passed to Create.
    kDegreeZero,  ///< Differentiate called on a degree-0 polynomial.
    kDegreeMismatch,  ///< Operands have different degrees for + or -.
    kInvalidParameter  ///< Evaluation parameter is outside [0, 1].
  };

  /**
   * @brief Creates a Polynomial from a pool-backed coefficient vector.
   * @param[in] coefficients Bernstein coefficients b_0, ..., b_n.
   * @return Result<Polynomial> — failure with kInvalidCoefficients if empty.
   */
  [[nodiscard]] static auto Create(std::vector<double, playground::core::memory::Allocator<double>>
                                       coefficients) -> playground::core::Result<Polynomial>;

  /** @brief Move-only: move construction and assignment are enabled; copy is deleted. */
  Polynomial(Polynomial&&) = default;
  auto operator=(Polynomial&&) -> Polynomial& = default;
  Polynomial(const Polynomial&) = delete;
  auto operator=(const Polynomial&) -> Polynomial& = delete;
  ~Polynomial();

  /**
   * @brief Returns the polynomial degree (number of coefficients minus one).
   * @return The degree n, where the polynomial has n+1 coefficients.
   */
  [[nodiscard]] auto Degree() const -> std::size_t;

  /**
   * @brief Returns a pool-backed copy of the Bernstein coefficients.
   * @param[in] alloc Caller-supplied allocator for the returned vector.
   */
  [[nodiscard]] auto Coefficients(playground::core::memory::Allocator<double> alloc) const
      -> std::vector<double, playground::core::memory::Allocator<double>>;

  /**
   * @brief Evaluates the polynomial at param using the de Casteljau algorithm.
   * @param[in] param Evaluation parameter in [0, 1].
   * @return Result<double> — failure with kInvalidParameter if outside [0, 1].
   */
  [[nodiscard]] auto Evaluate(double param) const -> playground::core::Result<double>;

  /**
   * @brief Adds two same-degree polynomials component-wise.
   * @param[in] other The polynomial to add; must have the same degree.
   * @return Result<Polynomial> — failure with kDegreeMismatch if degrees differ.
   */
  [[nodiscard]] auto operator+(const Polynomial& other) const
      -> playground::core::Result<Polynomial>;

  /**
   * @brief Subtracts two same-degree polynomials component-wise.
   * @param[in] other The polynomial to subtract; must have the same degree.
   * @return Result<Polynomial> — failure with kDegreeMismatch if degrees differ.
   */
  [[nodiscard]] auto operator-(const Polynomial& other) const
      -> playground::core::Result<Polynomial>;

  /**
   * @brief Multiplies all coefficients by a scalar (infallible).
   * @param[in] scalar The scalar factor applied to every coefficient.
   * @return A new Polynomial whose coefficients are each multiplied by scalar.
   */
  [[nodiscard]] auto operator*(double scalar) const -> Polynomial;

  /**
   * @brief Elevates degree from n to n+1 preserving mathematical equivalence (infallible).
   * @param[in] poly The polynomial to elevate.
   * @return A new degree-(n+1) Polynomial that represents the same function as poly.
   */
  [[nodiscard]] static auto Elevate(const Polynomial& poly) -> Polynomial;

  /**
   * @brief Returns the derivative polynomial (degree n to n-1) using forward differences.
   * @param[in] poly The polynomial to differentiate.
   * @return Result<Polynomial> — failure with kDegreeZero if degree is 0.
   */
  [[nodiscard]] static auto Differentiate(const Polynomial& poly)
      -> playground::core::Result<Polynomial>;

  /**
   * @brief Returns the product polynomial of degree n+m (infallible).
   * @param[in] lhs Left-hand operand.
   * @param[in] rhs Right-hand operand.
   * @return A new Polynomial of degree n+m equal to the product of lhs and rhs.
   */
  [[nodiscard]] static auto Multiply(const Polynomial& lhs, const Polynomial& rhs) -> Polynomial;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  explicit Polynomial(std::unique_ptr<Impl> impl);
};

}  // namespace qcar2_planner::core::bernstein

#endif  // QCAR2_PLANNER_CORE_BERNSTEIN_POLYNOMIAL_HPP_
