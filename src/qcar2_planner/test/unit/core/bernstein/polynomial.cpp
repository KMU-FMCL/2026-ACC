#include "qcar2_planner/core/bernstein/polynomial.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstddef>
#include <utility>
#include <vector>

#include "playground/core/memory/allocator.hpp"
#include "playground/core/memory/pool.hpp"
#include "playground/core/memory/size.hpp"

// NOLINTBEGIN(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)

namespace {
constexpr double kRelTol = 0.001;
constexpr double kAbsTol = 1e-9;
}  // namespace

TEST_CASE("Polynomial Create validates input and constructs a Bernstein polynomial",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("valid coefficients produce a Polynomial") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);

    // Act:
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));

    // Assert:
    REQUIRE(kResult.IsSuccess());
  }

  SECTION("empty coefficients return kInvalidCoefficients") {
    // Arrange:
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);

    // Act:
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));

    // Assert:
    REQUIRE_FALSE(kResult.IsSuccess());
    REQUIRE(kResult.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kResult.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kInvalidCoefficients);
  }
}

TEST_CASE("Polynomial Degree reports the number of Bernstein coefficients minus one",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("degree equals number of coefficients minus one") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kDegree = kResult.Value().Degree();

    // Assert:
    REQUIRE(kDegree == 2U);
  }

  SECTION("degree zero for single coefficient") {
    // Arrange:
    constexpr double kSingleCoeff = 5.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kSingleCoeff);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kDegree = kResult.Value().Degree();

    // Assert:
    REQUIRE(kDegree == 0U);
  }
}

TEST_CASE("Polynomial Evaluate computes de Casteljau value and rejects out-of-range parameters",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  // Degree-2 Bernstein: b0*(1-t)^2 + b1*2*(1-t)*t + b2*t^2
  // with b0=0, b1=0.5, b2=1 => t=0:0, t=0.5:0.5, t=1:1
  constexpr double kCoeffMid = 0.5;
  std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
  coeffs.push_back(0.0);
  coeffs.push_back(kCoeffMid);
  coeffs.push_back(1.0);
  const auto kPolyResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
  REQUIRE(kPolyResult.IsSuccess());
  const auto& poly = kPolyResult.Value();  // fixture re-created per SECTION by Catch2; no SECTION
                                           // transfers ownership of poly

  SECTION("evaluate at t=0 returns first coefficient") {
    // Arrange:

    // Act:
    const auto kEval = poly.Evaluate(0.0);

    // Assert:
    REQUIRE(kEval.IsSuccess());
    REQUIRE_THAT(kEval.Value(), Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }

  SECTION("evaluate at t=1 returns last coefficient") {
    // Arrange:

    // Act:
    const auto kEval = poly.Evaluate(1.0);

    // Assert:
    REQUIRE(kEval.IsSuccess());
    REQUIRE_THAT(kEval.Value(), Catch::Matchers::WithinRel(1.0, kRelTol));
  }

  SECTION("evaluate at t=0.5 returns midpoint value") {
    // Arrange:
    constexpr double kParamMid = 0.5;
    constexpr double kExpectedMid = 0.5;

    // Act:
    const auto kEval = poly.Evaluate(kParamMid);

    // Assert:
    REQUIRE(kEval.IsSuccess());
    REQUIRE_THAT(kEval.Value(), Catch::Matchers::WithinRel(kExpectedMid, kRelTol));
  }

  SECTION("parameter below 0 returns kInvalidParameter") {
    // Arrange:
    constexpr double kBelowRange = -0.1;

    // Act:
    const auto kEval = poly.Evaluate(kBelowRange);

    // Assert:
    REQUIRE_FALSE(kEval.IsSuccess());
    REQUIRE(kEval.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kEval.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kInvalidParameter);
  }

  SECTION("parameter above 1 returns kInvalidParameter") {
    // Arrange:
    constexpr double kAboveRange = 1.1;

    // Act:
    const auto kEval = poly.Evaluate(kAboveRange);

    // Assert:
    REQUIRE_FALSE(kEval.IsSuccess());
    REQUIRE(kEval.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kEval.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kInvalidParameter);
  }
}

TEST_CASE("Polynomial Coefficients returns the original Bernstein coefficients unchanged",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("Coefficients returns the original values") {
    // Arrange:
    constexpr double kCoeffA = 2.0;
    constexpr double kCoeffB = 5.0;
    constexpr double kCoeffC = 8.0;
    constexpr std::size_t kExpectedSize = 3U;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kCoeffA);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kReturned = kResult.Value().Coefficients(alloc);

    // Assert:
    REQUIRE(kReturned.size() == kExpectedSize);
    REQUIRE_THAT(kReturned[0], Catch::Matchers::WithinRel(kCoeffA, kRelTol));
    REQUIRE_THAT(kReturned[1], Catch::Matchers::WithinRel(kCoeffB, kRelTol));
    REQUIRE_THAT(kReturned[2], Catch::Matchers::WithinRel(kCoeffC, kRelTol));
  }
}

TEST_CASE(
    "Polynomial operator+ adds same-degree polynomials component-wise and rejects degree mismatch",
    "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("same-degree polynomials sum component-wise") {
    // Arrange:
    constexpr double kA1 = 2.0;
    constexpr double kA2 = 3.0;
    constexpr double kB0 = 4.0;
    constexpr double kB1 = 5.0;
    constexpr double kB2 = 6.0;
    constexpr std::size_t kExpectedSize = 3U;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(1.0);
    coeffs_a.push_back(kA1);
    coeffs_a.push_back(kA2);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(kB0);
    coeffs_b.push_back(kB1);
    coeffs_b.push_back(kB2);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kSum = kPolyA.Value() + kPolyB.Value();

    // Assert:
    REQUIRE(kSum.IsSuccess());
    const auto kCoeffs = kSum.Value().Coefficients(alloc);
    REQUIRE(kCoeffs.size() == kExpectedSize);
    REQUIRE_THAT(kCoeffs[0], Catch::Matchers::WithinRel(1.0 + kB0, kRelTol));
    REQUIRE_THAT(kCoeffs[1], Catch::Matchers::WithinRel(kA1 + kB1, kRelTol));
    REQUIRE_THAT(kCoeffs[2], Catch::Matchers::WithinRel(kA2 + kB2, kRelTol));
  }

  SECTION("different-degree polynomials return kDegreeMismatch") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(1.0);
    coeffs_a.push_back(kCoeffB);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(1.0);
    coeffs_b.push_back(kCoeffB);
    coeffs_b.push_back(kCoeffC);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kSum = kPolyA.Value() + kPolyB.Value();

    // Assert:
    REQUIRE_FALSE(kSum.IsSuccess());
    REQUIRE(kSum.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kSum.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kDegreeMismatch);
  }
}

TEST_CASE(
    "Polynomial operator- subtracts same-degree polynomials component-wise and rejects degree "
    "mismatch",
    "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("same-degree polynomials subtract component-wise") {
    // Arrange:
    constexpr double kA0 = 5.0;
    constexpr double kA1 = 7.0;
    constexpr double kA2 = 9.0;
    constexpr double kB1 = 2.0;
    constexpr double kB2 = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(kA0);
    coeffs_a.push_back(kA1);
    coeffs_a.push_back(kA2);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(1.0);
    coeffs_b.push_back(kB1);
    coeffs_b.push_back(kB2);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kDiff = kPolyA.Value() - kPolyB.Value();

    // Assert:
    REQUIRE(kDiff.IsSuccess());
    const auto kCoeffs = kDiff.Value().Coefficients(alloc);
    REQUIRE_THAT(kCoeffs[0], Catch::Matchers::WithinRel(kA0 - 1.0, kRelTol));
    REQUIRE_THAT(kCoeffs[1], Catch::Matchers::WithinRel(kA1 - kB1, kRelTol));
    REQUIRE_THAT(kCoeffs[2], Catch::Matchers::WithinRel(kA2 - kB2, kRelTol));
  }

  SECTION("different-degree polynomials return kDegreeMismatch") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(1.0);
    coeffs_a.push_back(kCoeffB);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(1.0);
    coeffs_b.push_back(kCoeffB);
    coeffs_b.push_back(kCoeffC);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kDiff = kPolyA.Value() - kPolyB.Value();

    // Assert:
    REQUIRE_FALSE(kDiff.IsSuccess());
    REQUIRE(kDiff.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kDiff.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kDegreeMismatch);
  }
}

TEST_CASE("Polynomial operator* scales all Bernstein coefficients by a scalar factor",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("scalar multiply scales all coefficients") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    constexpr double kScalar = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kScaled = kResult.Value() * kScalar;

    // Assert:
    const auto kReturned = kScaled.Coefficients(alloc);
    REQUIRE_THAT(kReturned[0], Catch::Matchers::WithinRel(1.0 * kScalar, kRelTol));
    REQUIRE_THAT(kReturned[1], Catch::Matchers::WithinRel(kCoeffB * kScalar, kRelTol));
    REQUIRE_THAT(kReturned[2], Catch::Matchers::WithinRel(kCoeffC * kScalar, kRelTol));
  }

  SECTION("multiply by zero yields zero coefficients") {
    // Arrange:
    constexpr double kCoeffA = 4.0;
    constexpr double kCoeffB = 5.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kCoeffA);
    coeffs.push_back(kCoeffB);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kScaled = kResult.Value() * 0.0;

    // Assert:
    const auto kReturned = kScaled.Coefficients(alloc);
    REQUIRE_THAT(kReturned[0], Catch::Matchers::WithinAbs(0.0, kAbsTol));
    REQUIRE_THAT(kReturned[1], Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }
}

TEST_CASE("Polynomial Elevate re-expresses a degree-n polynomial in degree-n+1 Bernstein basis",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("elevated degree is one greater than original") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    constexpr std::size_t kExpectedDegree = 3U;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kElevated = qcar2_planner::core::bernstein::Polynomial::Elevate(kResult.Value());

    // Assert:
    REQUIRE(kElevated.Degree() == kExpectedDegree);
  }

  SECTION("elevated polynomial evaluates identically to original at t=0.5") {
    // Arrange:
    constexpr double kCoeffMid = 0.5;
    constexpr double kParam = 0.5;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(0.0);
    coeffs.push_back(kCoeffMid);
    coeffs.push_back(1.0);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());
    const auto kOriginalVal = kResult.Value().Evaluate(kParam);
    REQUIRE(kOriginalVal.IsSuccess());
    const auto kElevated = qcar2_planner::core::bernstein::Polynomial::Elevate(kResult.Value());

    // Act:
    const auto kElevatedVal = kElevated.Evaluate(kParam);

    // Assert:
    REQUIRE(kElevatedVal.IsSuccess());
    REQUIRE_THAT(kElevatedVal.Value(), Catch::Matchers::WithinRel(kOriginalVal.Value(), kRelTol));
  }

  SECTION("elevated polynomial evaluates identically to original at t=0.25") {
    // Arrange:
    constexpr double kCoeffB = 3.0;
    constexpr double kCoeffC = 2.0;
    constexpr double kParam = 0.25;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffB);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());
    const auto kOriginalVal = kResult.Value().Evaluate(kParam);
    REQUIRE(kOriginalVal.IsSuccess());
    const auto kElevated = qcar2_planner::core::bernstein::Polynomial::Elevate(kResult.Value());

    // Act:
    const auto kElevatedVal = kElevated.Evaluate(kParam);

    // Assert:
    REQUIRE(kElevatedVal.IsSuccess());
    REQUIRE_THAT(kElevatedVal.Value(), Catch::Matchers::WithinRel(kOriginalVal.Value(), kRelTol));
  }

  SECTION("degree-0 polynomial elevates to degree-1 with both coefficients equal to original") {
    // p(t) = 3 * B_0^0(t) = 3
    // Elevation formula: b̂_i = (i/(n+1))*b_{i-1} + (1 - i/(n+1))*b_i, n=0
    // b̂_0 = (0/1)*b_{-1} + (1 - 0/1)*b_0 = b_0 = 3
    // b̂_1 = (1/1)*b_0  + (1 - 1/1)*b_1  = b_0 = 3
    // Elevated coefficients: [3, 3]

    // Arrange:
    constexpr double kConstCoeff = 3.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kConstCoeff);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kElevated = qcar2_planner::core::bernstein::Polynomial::Elevate(kResult.Value());

    // Assert:
    REQUIRE(kElevated.Degree() == 1U);
    const auto kCoeffs = kElevated.Coefficients(alloc);
    REQUIRE(kCoeffs.size() == 2U);
    REQUIRE_THAT(kCoeffs[0], Catch::Matchers::WithinRel(kConstCoeff, kRelTol));
    REQUIRE_THAT(kCoeffs[1], Catch::Matchers::WithinRel(kConstCoeff, kRelTol));
  }
}

TEST_CASE(
    "Polynomial Differentiate computes the derivative polynomial via Bernstein forward differences",
    "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("derivative degree is one less than original") {
    // Arrange:
    constexpr double kCoeffC = 4.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(0.0);
    coeffs.push_back(1.0);
    coeffs.push_back(kCoeffC);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kDeriv = qcar2_planner::core::bernstein::Polynomial::Differentiate(kResult.Value());

    // Assert:
    REQUIRE(kDeriv.IsSuccess());
    REQUIRE(kDeriv.Value().Degree() == 1U);
  }

  SECTION("degree-0 polynomial returns kDegreeZero") {
    // Arrange:
    constexpr double kConstCoeff = 5.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kConstCoeff);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kDeriv = qcar2_planner::core::bernstein::Polynomial::Differentiate(kResult.Value());

    // Assert:
    REQUIRE_FALSE(kDeriv.IsSuccess());
    REQUIRE(kDeriv.GetError().Is<qcar2_planner::core::bernstein::Polynomial::ErrorType>());
    REQUIRE(kDeriv.GetError().As<qcar2_planner::core::bernstein::Polynomial::ErrorType>() ==
            qcar2_planner::core::bernstein::Polynomial::ErrorType::kDegreeZero);
  }

  SECTION("derivative of linear polynomial b0*(1-t)+b1*t equals n*(b1-b0)") {
    // p(t) = 2*(1-t) + 4*t = 2 + 2t
    // p'(t) = 2  (constant) => Bernstein coeff = n*(b1-b0) = 1*(4-2) = 2

    // Arrange:
    constexpr double kCoeffA = 2.0;
    constexpr double kCoeffB = 4.0;
    constexpr double kExpectedDeriv = 2.0;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs(alloc);
    coeffs.push_back(kCoeffA);
    coeffs.push_back(kCoeffB);
    const auto kResult = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs));
    REQUIRE(kResult.IsSuccess());

    // Act:
    const auto kDeriv = qcar2_planner::core::bernstein::Polynomial::Differentiate(kResult.Value());

    // Assert:
    REQUIRE(kDeriv.IsSuccess());
    const auto kReturned = kDeriv.Value().Coefficients(alloc);
    REQUIRE(kReturned.size() == 1U);
    REQUIRE_THAT(kReturned[0], Catch::Matchers::WithinRel(kExpectedDeriv, kRelTol));
  }
}

TEST_CASE("Polynomial Multiply produces a degree-(n+m) product from two Bernstein polynomials",
          "[core][bernstein]") {
  constexpr std::size_t kPoolSizeKb = 4;
  playground::core::memory::Pool pool(playground::core::memory::Size::KB(kPoolSizeKb));
  playground::core::memory::Allocator<double> alloc(pool);

  SECTION("product degree equals sum of operand degrees") {
    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    constexpr std::size_t kExpectedDegree = 3U;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(1.0);
    coeffs_a.push_back(kCoeffB);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(1.0);
    coeffs_b.push_back(kCoeffB);
    coeffs_b.push_back(kCoeffC);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kProduct =
        qcar2_planner::core::bernstein::Polynomial::Multiply(kPolyA.Value(), kPolyB.Value());

    // Assert:
    REQUIRE(kProduct.Degree() == kExpectedDegree);
  }

  SECTION("product evaluates correctly at t=0.5") {
    // p(t) = t (linear: b0=0, b1=1)
    // q(t) = t (linear: b0=0, b1=1)
    // p*q = t^2, evaluated at t=0.5 should give 0.25

    // Arrange:
    constexpr double kParam = 0.5;
    constexpr double kExpected = 0.25;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(0.0);
    coeffs_a.push_back(1.0);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(0.0);
    coeffs_b.push_back(1.0);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());
    const auto kProduct =
        qcar2_planner::core::bernstein::Polynomial::Multiply(kPolyA.Value(), kPolyB.Value());

    // Act:
    const auto kVal = kProduct.Evaluate(kParam);

    // Assert:
    REQUIRE(kVal.IsSuccess());
    REQUIRE_THAT(kVal.Value(), Catch::Matchers::WithinRel(kExpected, kRelTol));
  }

  SECTION("degree-0 operand produces degree-n result with each coefficient scaled by the scalar") {
    // p(t) = 1*B_0^2(t) + 2*B_1^2(t) + 3*B_2^2(t)  (degree-2)
    // q(t) = 4*B_0^0(t) = 4                           (degree-0)
    // p*q has degree 2+0=2; c_k = a_k * 4 for each k
    // Expected coefficients: [4, 8, 12]

    // Arrange:
    constexpr double kCoeffB = 2.0;
    constexpr double kCoeffC = 3.0;
    constexpr double kScalar = 4.0;
    constexpr std::size_t kExpectedSize = 3U;
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_a(alloc);
    coeffs_a.push_back(1.0);
    coeffs_a.push_back(kCoeffB);
    coeffs_a.push_back(kCoeffC);
    std::vector<double, playground::core::memory::Allocator<double>> coeffs_b(alloc);
    coeffs_b.push_back(kScalar);
    const auto kPolyA = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_a));
    const auto kPolyB = qcar2_planner::core::bernstein::Polynomial::Create(std::move(coeffs_b));
    REQUIRE(kPolyA.IsSuccess());
    REQUIRE(kPolyB.IsSuccess());

    // Act:
    const auto kProduct =
        qcar2_planner::core::bernstein::Polynomial::Multiply(kPolyA.Value(), kPolyB.Value());

    // Assert:
    REQUIRE(kProduct.Degree() == 2U);
    const auto kCoeffs = kProduct.Coefficients(alloc);
    REQUIRE(kCoeffs.size() == kExpectedSize);
    REQUIRE_THAT(kCoeffs[0], Catch::Matchers::WithinRel(kScalar, kRelTol));
    REQUIRE_THAT(kCoeffs[1], Catch::Matchers::WithinRel(kCoeffB * kScalar, kRelTol));
    REQUIRE_THAT(kCoeffs[2], Catch::Matchers::WithinRel(kCoeffC * kScalar, kRelTol));
  }
}

// NOLINTEND(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)
