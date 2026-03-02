#include "qcar2_planner/core/rbf/gaussian.hpp"

#include <cmath>

#include "playground/core/error/error.hpp"
#include "playground/core/error/result.hpp"

namespace qcar2_planner::core::rbf {

Gaussian::Gaussian(double epsilon, double center) : epsilon_(epsilon), center_(center) {}

auto Gaussian::Create(double epsilon, double center) -> playground::core::Result<Gaussian> {
  if (epsilon <= 0.0) {
    return playground::core::Result<Gaussian>::Failure(
        playground::core::Error(ErrorType::kInvalidEpsilon));
  }
  return playground::core::Result<Gaussian>::Success(Gaussian(epsilon, center));
}

auto Gaussian::Evaluate(double xi_val) const -> double {
  const double kDiff = xi_val - center_;
  return std::exp(-epsilon_ * kDiff * kDiff);
}

auto Gaussian::Epsilon() const -> double {
  return epsilon_;
}

auto Gaussian::Center() const -> double {
  return center_;
}

}  // namespace qcar2_planner::core::rbf
