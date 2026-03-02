#include "qcar2_planner/core/hodograph/quintic.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>

#include "playground/core/error/error.hpp"
#include "playground/core/error/result.hpp"
#include "qcar2_planner/core/hodograph/planar_point.hpp"

namespace qcar2_planner::core::hodograph {

namespace {

// Number of solution branches for G1 PH quintic interpolation.
constexpr int kNumBranches = 4;

// Number of control points for a quintic (degree-5) curve.
constexpr std::size_t kNumControlPoints = 6U;

// Number of Bernstein coefficients for degree-4 speed / hodograph polynomials.
constexpr std::size_t kDeg4CoeffCount = 5U;

// Number of Bernstein coefficients for degree-5 arc length polynomial.
constexpr std::size_t kDeg5CoeffCount = 6U;

// Number of Bernstein coefficients for degree-2 preimage polynomials u, v.
constexpr std::size_t kDeg2CoeffCount = 3U;

// Bernstein product weights for degree-2 × degree-2 (n=m=2).
constexpr double kWeightK1 = 2.0;  // C(2,1) = 2
constexpr double kWeightK2 = 4.0;  // C(2,1)*C(2,1) = 4
constexpr double kDenomK1 = 4.0;  // C(4,1) = 4
constexpr double kDenomK2 = 6.0;  // C(4,2) = 6
constexpr double kDenomK3 = 4.0;  // C(4,3) = 4
constexpr double kHodographScale = 2.0;  // y' = 2*u*v
constexpr double kControlPointDenom = 5.0;  // Integration: Pi = Pi-1 + (1/5)*qi-1
constexpr double kSigmaIntegralDenom = 5.0;  // Total length = (1/5)*Σ sigma_k

// Tolerance for coincident-point and zero-tangent checks (both are geometric scale tests).
constexpr double kGeometricTol = 1e-10;

// Sign table for the four solution branches: {sign_s, sign_e}.
constexpr std::array<std::array<double, 2>, static_cast<std::size_t>(kNumBranches)> kBranchSigns{
    {{+1.0, +1.0}, {+1.0, -1.0}, {-1.0, +1.0}, {-1.0, -1.0}}};

/** @brief Evaluates a Bernstein polynomial of any degree using de Casteljau. */
template <std::size_t N>
auto EvalPoly(const std::array<double, N>& coeffs, double param) -> double {
  std::array<double, N> work = coeffs;
  const double kOneMinus = 1.0 - param;
  for (std::size_t pass = 1; pass < N; ++pass) {
    for (std::size_t idx = 0; idx < N - pass; ++idx) {
      work.at(idx) = kOneMinus * work.at(idx) + param * work.at(idx + 1U);
    }
  }
  return work.at(0);
}

/**
 * @brief Computes the degree-4 Bernstein product of two degree-2 Bernstein polynomials.
 * @details Uses the exact formula: c_k = Σ C(2,i)*C(2,k-i)/C(4,k) * a_i * b_{k-i}
 */
auto MultiplyDeg2(const std::array<double, kDeg2CoeffCount>& lhs,
                  const std::array<double, kDeg2CoeffCount>& rhs)
    -> std::array<double, kDeg4CoeffCount> {
  std::array<double, kDeg4CoeffCount> prod{};
  // k=0: C(2,0)*C(2,0)/C(4,0) = 1
  prod.at(0) = lhs.at(0) * rhs.at(0);
  // k=1: [C(2,0)*C(2,1)*a0*b1 + C(2,1)*C(2,0)*a1*b0] / C(4,1)
  prod.at(1) = (kWeightK1 * lhs.at(0) * rhs.at(1) + kWeightK1 * lhs.at(1) * rhs.at(0)) / kDenomK1;
  // k=2: [C(2,0)*C(2,2)*a0*b2 + C(2,1)*C(2,1)*a1*b1 + C(2,2)*C(2,0)*a2*b0] / C(4,2)
  prod.at(2) = (lhs.at(0) * rhs.at(2) + kWeightK2 * lhs.at(1) * rhs.at(1) + lhs.at(2) * rhs.at(0)) /
               kDenomK2;
  // k=3: [C(2,1)*C(2,2)*a1*b2 + C(2,2)*C(2,1)*a2*b1] / C(4,3)
  prod.at(3) = (kWeightK1 * lhs.at(1) * rhs.at(2) + kWeightK1 * lhs.at(2) * rhs.at(1)) / kDenomK3;
  // k=4: C(2,2)*C(2,2)/C(4,4) = 1
  prod.at(4) = lhs.at(2) * rhs.at(2);
  return prod;
}

/** @brief Computes x'_k = (u² - v²)_k (degree-4 Bernstein coefficients). */
auto ComputeHodographX(const std::array<double, kDeg2CoeffCount>& u_arr,
                       const std::array<double, kDeg2CoeffCount>& v_arr)
    -> std::array<double, kDeg4CoeffCount> {
  const auto kUSq = MultiplyDeg2(u_arr, u_arr);
  const auto kVSq = MultiplyDeg2(v_arr, v_arr);
  std::array<double, kDeg4CoeffCount> result{};
  for (std::size_t idx = 0; idx < kDeg4CoeffCount; ++idx) {
    result.at(idx) = kUSq.at(idx) - kVSq.at(idx);
  }
  return result;
}

/** @brief Computes y'_k = 2*(u*v)_k (degree-4 Bernstein coefficients). */
auto ComputeHodographY(const std::array<double, kDeg2CoeffCount>& u_arr,
                       const std::array<double, kDeg2CoeffCount>& v_arr)
    -> std::array<double, kDeg4CoeffCount> {
  const auto kUV = MultiplyDeg2(u_arr, v_arr);
  std::array<double, kDeg4CoeffCount> result{};
  for (std::size_t idx = 0; idx < kDeg4CoeffCount; ++idx) {
    result.at(idx) = kHodographScale * kUV.at(idx);
  }
  return result;
}

/** @brief Computes sigma_k = (u² + v²)_k (degree-4 Bernstein coefficients). */
auto ComputeSigma(const std::array<double, kDeg2CoeffCount>& u_arr,
                  const std::array<double, kDeg2CoeffCount>& v_arr)
    -> std::array<double, kDeg4CoeffCount> {
  const auto kUSq = MultiplyDeg2(u_arr, u_arr);
  const auto kVSq = MultiplyDeg2(v_arr, v_arr);
  std::array<double, kDeg4CoeffCount> result{};
  for (std::size_t idx = 0; idx < kDeg4CoeffCount; ++idx) {
    result.at(idx) = kUSq.at(idx) + kVSq.at(idx);
  }
  return result;
}

/**
 * @brief Integrates a degree-4 Bernstein polynomial over [0, param] via antiderivative.
 * @details Antiderivative: â_0 = 0, â_k = (1/5) * Σ_{j<k} c_j for k=1..5.
 */
auto IntegrateDeg4(const std::array<double, kDeg4CoeffCount>& coeffs, double param) -> double {
  constexpr double kScale = 1.0 / 5.0;
  std::array<double, kDeg5CoeffCount> anti{};
  anti.at(0) = 0.0;
  double running_sum = 0.0;
  for (std::size_t jdx = 0; jdx < kDeg4CoeffCount; ++jdx) {
    running_sum += coeffs.at(jdx);
    anti.at(jdx + 1U) = kScale * running_sum;
  }
  return EvalPoly(anti, param);
}

struct BranchData {
  std::array<double, kDeg4CoeffCount> hod_x_coeffs;
  std::array<double, kDeg4CoeffCount> hod_y_coeffs;
  std::array<double, kDeg4CoeffCount> sigma_coeffs;
  std::array<PlanarPoint, kNumControlPoints> control_points;
  double total_length;
};

/**
 * @brief Builds one of the four G1 PH quintic solution branches.
 * @details Uses complex quadratic formula to determine the interior preimage coefficient w1.
 */
auto BuildBranch(PlanarPoint start, PlanarPoint end, double theta_s, double theta_e, double sign_s,
                 double sign_e) -> BranchData {
  // Step 1: Endpoint preimage values from tangent angles (angle half-angle formula).
  const double kU0 = sign_s * std::cos(theta_s / 2.0);
  const double kV0 = sign_s * std::sin(theta_s / 2.0);
  const double kU2 = sign_e * std::cos(theta_e / 2.0);
  const double kV2 = sign_e * std::sin(theta_e / 2.0);

  // Step 2: Solve A*w1² + B*w1 + C = 0 (complex quadratic) for w1 = u1 + i*v1.
  // From ∫₀¹ [w(t)]² dt = Δ_complex, A=2/3, B=w0+w2, C=w0²+(w0*w2)/3+w2²-5*Δ.
  // Solution: w1 = (3/4)*(-B ± sqrt(D)) with D = B² - (8/3)*C.

  const double kDeltaX = end.x - start.x;
  const double kDeltaY = end.y - start.y;

  const double kW0SqRe = kU0 * kU0 - kV0 * kV0;
  const double kW0SqIm = 2.0 * kU0 * kV0;
  const double kW2SqRe = kU2 * kU2 - kV2 * kV2;
  const double kW2SqIm = 2.0 * kU2 * kV2;
  const double kW0W2Re = kU0 * kU2 - kV0 * kV2;
  const double kW0W2Im = kU0 * kV2 + kV0 * kU2;

  const double kBRe = kU0 + kU2;
  const double kBIm = kV0 + kV2;

  constexpr double kDisplScale = 5.0;
  constexpr double kW0W2Scale = 3.0;
  const double kConstRe = kW0SqRe + kW0W2Re / kW0W2Scale + kW2SqRe - kDisplScale * kDeltaX;
  const double kConstIm = kW0SqIm + kW0W2Im / kW0W2Scale + kW2SqIm - kDisplScale * kDeltaY;

  const double kBSqRe = kBRe * kBRe - kBIm * kBIm;
  const double kBSqIm = 2.0 * kBRe * kBIm;
  constexpr double kDiscScale = 8.0 / 3.0;
  const double kDiscRe = kBSqRe - kDiscScale * kConstRe;
  const double kDiscIm = kBSqIm - kDiscScale * kConstIm;

  const double kDiscMag = std::sqrt(kDiscRe * kDiscRe + kDiscIm * kDiscIm);
  const double kDiscArg = std::atan2(kDiscIm, kDiscRe);
  const double kSqrtDiscMag = std::sqrt(kDiscMag);
  const double kSqrtDiscRe = kSqrtDiscMag * std::cos(kDiscArg / 2.0);
  const double kSqrtDiscIm = kSqrtDiscMag * std::sin(kDiscArg / 2.0);

  constexpr double kW1Scale = 3.0 / 4.0;
  const double kU1 = kW1Scale * (-kBRe + kSqrtDiscRe);
  const double kV1 = kW1Scale * (-kBIm + kSqrtDiscIm);

  // Step 3: Assemble preimage coefficient arrays.
  const std::array<double, kDeg2CoeffCount> kUCoeffs{kU0, kU1, kU2};
  const std::array<double, kDeg2CoeffCount> kVCoeffs{kV0, kV1, kV2};

  // Step 4: Compute speed polynomial sigma = u² + v² (degree 4).
  const auto kSigmaCoeffs = ComputeSigma(kUCoeffs, kVCoeffs);

  // Step 5: Hodograph components and 6 Bezier control points.
  const auto kHodXCoeffs = ComputeHodographX(kUCoeffs, kVCoeffs);
  const auto kHodYCoeffs = ComputeHodographY(kUCoeffs, kVCoeffs);

  std::array<PlanarPoint, kNumControlPoints> ctrl{};
  ctrl.at(0) = start;
  for (std::size_t idx = 1; idx < kNumControlPoints; ++idx) {
    ctrl.at(idx) = ctrl.at(idx - 1U) + PlanarPoint{kHodXCoeffs.at(idx - 1U) / kControlPointDenom,
                                                   kHodYCoeffs.at(idx - 1U) / kControlPointDenom};
  }

  // Step 6: Total arc length = (1/5) * Σ sigma_k (integral formula).
  double total_length = 0.0;
  for (const double kCoeff : kSigmaCoeffs) {
    total_length += kCoeff;
  }
  total_length /= kSigmaIntegralDenom;

  return BranchData{kHodXCoeffs, kHodYCoeffs, kSigmaCoeffs, ctrl, total_length};
}

/** @brief Validates G1 Hermite input data. */
auto ValidateInput(PlanarPoint start, PlanarPoint end, PlanarPoint tangent_start,
                   PlanarPoint tangent_end) -> playground::core::Result<void> {
  if (start.DistanceTo(end) < kGeometricTol) {
    return playground::core::Result<void>::Failure(
        playground::core::Error(Quintic::ErrorType::kCoincidentPoints));
  }
  if (tangent_start.Norm() < kGeometricTol) {
    return playground::core::Result<void>::Failure(
        playground::core::Error(Quintic::ErrorType::kZeroTangent));
  }
  if (tangent_end.Norm() < kGeometricTol) {
    return playground::core::Result<void>::Failure(
        playground::core::Error(Quintic::ErrorType::kZeroTangent));
  }
  return playground::core::Result<void>::Success();
}

/** @brief Normalizes tangent vectors and returns their angles for preimage construction. */
struct TangentAngles {
  double theta_s;
  double theta_e;
};

auto ComputeTangentAngles(PlanarPoint tangent_start, PlanarPoint tangent_end) -> TangentAngles {
  const auto kTsUnit = tangent_start / tangent_start.Norm();
  const auto kTeUnit = tangent_end / tangent_end.Norm();
  return {std::atan2(kTsUnit.y, kTsUnit.x), std::atan2(kTeUnit.y, kTeUnit.x)};
}

/** @brief Evaluates the hodograph and speed polynomial at param. */
struct HodographSample {
  double x_prime;
  double y_prime;
  double sigma;
};

auto EvalHodographAt(const std::array<double, kDeg4CoeffCount>& hod_x,
                     const std::array<double, kDeg4CoeffCount>& hod_y,
                     const std::array<double, kDeg4CoeffCount>& sigma,
                     double param) -> HodographSample {
  return {EvalPoly(hod_x, param), EvalPoly(hod_y, param), EvalPoly(sigma, param)};
}

}  // namespace

/**
 * @brief Impl stores raw coefficient arrays to avoid depending on Polynomial::Impl completeness.
 */
struct Quintic::Impl {
  std::array<double, kDeg4CoeffCount> hod_x_coeffs;
  std::array<double, kDeg4CoeffCount> hod_y_coeffs;
  std::array<double, kDeg4CoeffCount> sigma_coeffs;
  std::array<PlanarPoint, kNumControlPoints> control_points;
};

Quintic::Quintic(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Quintic::Quintic(Quintic&&) noexcept = default;

auto Quintic::operator=(Quintic&&) noexcept -> Quintic& = default;

Quintic::~Quintic() = default;

auto Quintic::Interpolate(PlanarPoint start, PlanarPoint end, PlanarPoint tangent_start,
                          PlanarPoint tangent_end) -> playground::core::Result<Quintic> {
  const auto kValidation = ValidateInput(start, end, tangent_start, tangent_end);
  if (kValidation.IsFailure()) {
    return playground::core::Result<Quintic>::Failure(kValidation.GetError());
  }

  const auto [kThetaS, kThetaE] = ComputeTangentAngles(tangent_start, tangent_end);

  auto best_data =
      BuildBranch(start, end, kThetaS, kThetaE, kBranchSigns.at(0).at(0), kBranchSigns.at(0).at(1));
  double min_length =
      best_data.total_length > 0.0 ? best_data.total_length : std::numeric_limits<double>::max();

  for (int branch_idx = 1; branch_idx < kNumBranches; ++branch_idx) {
    const double kSignS = kBranchSigns.at(static_cast<std::size_t>(branch_idx)).at(0);
    const double kSignE = kBranchSigns.at(static_cast<std::size_t>(branch_idx)).at(1);
    auto candidate = BuildBranch(start, end, kThetaS, kThetaE, kSignS, kSignE);
    if (candidate.total_length > 0.0 && candidate.total_length < min_length) {
      min_length = candidate.total_length;
      best_data = candidate;
    }
  }

  auto impl = std::make_unique<Impl>(Impl{best_data.hod_x_coeffs, best_data.hod_y_coeffs,
                                          best_data.sigma_coeffs, best_data.control_points});
  return playground::core::Result<Quintic>::Success(Quintic(std::move(impl)));
}

auto Quintic::Interpolate(PlanarPoint start, PlanarPoint end, PlanarPoint tangent_start,
                          PlanarPoint tangent_end,
                          int branch) -> playground::core::Result<Quintic> {
  if (branch < 0 || branch >= kNumBranches) {
    return playground::core::Result<Quintic>::Failure(
        playground::core::Error(ErrorType::kInvalidBranch));
  }

  const auto kValidation = ValidateInput(start, end, tangent_start, tangent_end);
  if (kValidation.IsFailure()) {
    return playground::core::Result<Quintic>::Failure(kValidation.GetError());
  }

  const auto [kThetaS, kThetaE] = ComputeTangentAngles(tangent_start, tangent_end);

  const double kSignS = kBranchSigns.at(static_cast<std::size_t>(branch)).at(0);
  const double kSignE = kBranchSigns.at(static_cast<std::size_t>(branch)).at(1);
  const auto kData = BuildBranch(start, end, kThetaS, kThetaE, kSignS, kSignE);

  auto impl = std::make_unique<Impl>(
      Impl{kData.hod_x_coeffs, kData.hod_y_coeffs, kData.sigma_coeffs, kData.control_points});
  return playground::core::Result<Quintic>::Success(Quintic(std::move(impl)));
}

auto Quintic::Evaluate(double param) const -> playground::core::Result<PlanarPoint> {
  if (param < 0.0 || param > 1.0) {
    return playground::core::Result<PlanarPoint>::Failure(
        playground::core::Error(ErrorType::kInvalidParameter));
  }
  const double kIntX = IntegrateDeg4(impl_->hod_x_coeffs, param);
  const double kIntY = IntegrateDeg4(impl_->hod_y_coeffs, param);
  return playground::core::Result<PlanarPoint>::Success(
      PlanarPoint{impl_->control_points.at(0).x + kIntX, impl_->control_points.at(0).y + kIntY});
}

auto Quintic::Tangent(double param) const -> playground::core::Result<PlanarPoint> {
  if (param < 0.0 || param > 1.0) {
    return playground::core::Result<PlanarPoint>::Failure(
        playground::core::Error(ErrorType::kInvalidParameter));
  }
  const auto [kXPrime, kYPrime, kSigma] =
      EvalHodographAt(impl_->hod_x_coeffs, impl_->hod_y_coeffs, impl_->sigma_coeffs, param);
  return playground::core::Result<PlanarPoint>::Success(
      PlanarPoint{kXPrime / kSigma, kYPrime / kSigma});
}

auto Quintic::Normal(double param) const -> playground::core::Result<PlanarPoint> {
  if (param < 0.0 || param > 1.0) {
    return playground::core::Result<PlanarPoint>::Failure(
        playground::core::Error(ErrorType::kInvalidParameter));
  }
  const auto [kXPrime, kYPrime, kSigma] =
      EvalHodographAt(impl_->hod_x_coeffs, impl_->hod_y_coeffs, impl_->sigma_coeffs, param);
  return playground::core::Result<PlanarPoint>::Success(
      PlanarPoint{-kYPrime / kSigma, kXPrime / kSigma});
}

auto Quintic::ArcLength(double param) const -> playground::core::Result<double> {
  if (param < 0.0 || param > 1.0) {
    return playground::core::Result<double>::Failure(
        playground::core::Error(ErrorType::kInvalidParameter));
  }

  return playground::core::Result<double>::Success(IntegrateDeg4(impl_->sigma_coeffs, param));
}

auto Quintic::TotalLength() const -> playground::core::Result<double> {
  return playground::core::Result<double>::Success(IntegrateDeg4(impl_->sigma_coeffs, 1.0));
}

}  // namespace qcar2_planner::core::hodograph
