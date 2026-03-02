#include "qcar2_planner/core/bernstein/polynomial.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "playground/core/error/error.hpp"
#include "playground/core/error/result.hpp"
#include "playground/core/memory/allocator.hpp"

namespace qcar2_planner::core::bernstein {

struct Polynomial::Impl {
  Eigen::VectorXd coefficients;
};

Polynomial::Polynomial(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Polynomial::~Polynomial() = default;

auto Polynomial::Create(std::vector<double, playground::core::memory::Allocator<double>>
                            coefficients) -> playground::core::Result<Polynomial> {
  if (coefficients.empty()) {
    return playground::core::Result<Polynomial>::Failure(
        playground::core::Error(ErrorType::kInvalidCoefficients));
  }
  auto impl = std::make_unique<Impl>();
  const auto kNumCoeffs = static_cast<Eigen::Index>(coefficients.size());
  impl->coefficients.resize(kNumCoeffs);
  for (Eigen::Index i = 0; i < kNumCoeffs; ++i) {
    impl->coefficients[i] = coefficients[static_cast<std::size_t>(i)];
  }
  return playground::core::Result<Polynomial>::Success(Polynomial(std::move(impl)));
}

auto Polynomial::Degree() const -> std::size_t {
  return static_cast<std::size_t>(impl_->coefficients.size()) - 1U;
}

auto Polynomial::Coefficients(playground::core::memory::Allocator<double> alloc) const
    -> std::vector<double, playground::core::memory::Allocator<double>> {
  const auto kCount = static_cast<std::size_t>(impl_->coefficients.size());
  std::vector<double, playground::core::memory::Allocator<double>> result(alloc);
  result.reserve(kCount);
  for (std::size_t i = 0; i < kCount; ++i) {
    result.push_back(impl_->coefficients[static_cast<Eigen::Index>(i)]);
  }
  return result;
}

auto Polynomial::Evaluate(double param) const -> playground::core::Result<double> {
  if (param < 0.0 || param > 1.0) {
    return playground::core::Result<double>::Failure(
        playground::core::Error(ErrorType::kInvalidParameter));
  }
  // de Casteljau algorithm: reduces degree-n to degree-0 in n passes
  Eigen::VectorXd work = impl_->coefficients;
  const auto kCoeffCount = static_cast<Eigen::Index>(work.size());
  for (Eigen::Index pass = 1; pass < kCoeffCount; ++pass) {
    for (Eigen::Index i = 0; i < kCoeffCount - pass; ++i) {
      work[i] = (1.0 - param) * work[i] + param * work[i + 1];
    }
  }
  return playground::core::Result<double>::Success(work[0]);
}

auto Polynomial::operator+(const Polynomial& other) const -> playground::core::Result<Polynomial> {
  if (Degree() != other.Degree()) {
    return playground::core::Result<Polynomial>::Failure(
        playground::core::Error(ErrorType::kDegreeMismatch));
  }
  auto impl = std::make_unique<Impl>();
  impl->coefficients = impl_->coefficients + other.impl_->coefficients;
  return playground::core::Result<Polynomial>::Success(Polynomial(std::move(impl)));
}

auto Polynomial::operator-(const Polynomial& other) const -> playground::core::Result<Polynomial> {
  if (Degree() != other.Degree()) {
    return playground::core::Result<Polynomial>::Failure(
        playground::core::Error(ErrorType::kDegreeMismatch));
  }
  auto impl = std::make_unique<Impl>();
  impl->coefficients = impl_->coefficients - other.impl_->coefficients;
  return playground::core::Result<Polynomial>::Success(Polynomial(std::move(impl)));
}

auto Polynomial::operator*(double scalar) const -> Polynomial {
  auto impl = std::make_unique<Impl>();
  impl->coefficients = impl_->coefficients * scalar;
  return Polynomial(std::move(impl));
}

auto Polynomial::Elevate(const Polynomial& poly) -> Polynomial {
  const auto kDegree = static_cast<Eigen::Index>(poly.Degree());
  Eigen::VectorXd elevated(kDegree + 2);
  elevated[0] = poly.impl_->coefficients[0];
  for (Eigen::Index i = 1; i <= kDegree; ++i) {
    const double kAlpha = static_cast<double>(i) / static_cast<double>(kDegree + 1);
    elevated[i] =
        kAlpha * poly.impl_->coefficients[i - 1] + (1.0 - kAlpha) * poly.impl_->coefficients[i];
  }
  elevated[kDegree + 1] = poly.impl_->coefficients[kDegree];
  auto impl = std::make_unique<Impl>();
  impl->coefficients = std::move(elevated);
  return Polynomial(std::move(impl));
}

auto Polynomial::Differentiate(const Polynomial& poly) -> playground::core::Result<Polynomial> {
  if (poly.Degree() == 0) {
    return playground::core::Result<Polynomial>::Failure(
        playground::core::Error(ErrorType::kDegreeZero));
  }
  const auto kDegree = static_cast<Eigen::Index>(poly.Degree());
  Eigen::VectorXd deriv(kDegree);
  for (Eigen::Index i = 0; i < kDegree; ++i) {
    deriv[i] = static_cast<double>(kDegree) *
               (poly.impl_->coefficients[i + 1] - poly.impl_->coefficients[i]);
  }
  auto impl = std::make_unique<Impl>();
  impl->coefficients = std::move(deriv);
  return playground::core::Result<Polynomial>::Success(Polynomial(std::move(impl)));
}

/**
 * @brief Computes the degree-(n+m) Bernstein product polynomial.
 * @details c[k] = sum_{i=max(0,k-m)}^{min(n,k)} C(n,i)*C(m,k-i)/C(n+m,k) * a[i]*b[k-i]
 */
auto Polynomial::Multiply(const Polynomial& lhs, const Polynomial& rhs) -> Polynomial {
  const auto kDegLhs = static_cast<Eigen::Index>(lhs.Degree());
  const auto kDegRhs = static_cast<Eigen::Index>(rhs.Degree());
  const auto kDeg = kDegLhs + kDegRhs;
  Eigen::VectorXd product(kDeg + 1);
  product.setZero();

  // Log-space binomial avoids overflow for large degrees
  const auto kLogBinom = [](Eigen::Index top, Eigen::Index bot) -> double {
    double val = 0.0;
    for (Eigen::Index j = 0; j < bot; ++j) {
      val += std::log(static_cast<double>(top - j)) - std::log(static_cast<double>(j + 1));
    }
    return val;
  };

  for (Eigen::Index k = 0; k <= kDeg; ++k) {
    const double kLogDenom = kLogBinom(kDeg, k);
    for (Eigen::Index i = std::max(Eigen::Index{0}, k - kDegRhs); i <= std::min(kDegLhs, k); ++i) {
      const Eigen::Index kColIndex = k - i;
      const double kWeight =
          std::exp(kLogBinom(kDegLhs, i) + kLogBinom(kDegRhs, kColIndex) - kLogDenom);
      product[k] += kWeight * lhs.impl_->coefficients[i] * rhs.impl_->coefficients[kColIndex];
    }
  }

  auto impl = std::make_unique<Impl>();
  impl->coefficients = std::move(product);
  return Polynomial(std::move(impl));
}

}  // namespace qcar2_planner::core::bernstein
