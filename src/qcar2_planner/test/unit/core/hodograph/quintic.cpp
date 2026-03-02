#include "qcar2_planner/core/hodograph/quintic.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

#include "qcar2_planner/core/hodograph/planar_point.hpp"

// NOLINTBEGIN(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)

namespace {
constexpr double kRelTol = 0.001;
constexpr double kAbsTol = 1e-9;
}  // namespace

using qcar2_planner::core::hodograph::PlanarPoint;
using qcar2_planner::core::hodograph::Quintic;

TEST_CASE("qcar2_planner::core::hodograph::Quintic", "[core][hodograph]") {
  // Standard G1 Hermite data: straight segment along x-axis with unit tangents
  constexpr double kEndX = 10.0;
  constexpr double kEndY = 5.0;
  const PlanarPoint kStart{0.0, 0.0};
  const PlanarPoint kEnd{kEndX, kEndY};
  const PlanarPoint kTangentStart{1.0, 0.0};
  const PlanarPoint kTangentEnd{0.0, 1.0};

  SECTION("Interpolate") {
    SECTION("default branch selects minimum length") {
      // Act:
      const auto kResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);

      // Assert:
      REQUIRE(kResult.IsSuccess());
    }

    SECTION("explicit branches 0-3 succeed") {
      // Act and Assert for each branch:
      for (int branch = 0; branch < 4; ++branch) {
        const auto kResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd, branch);
        REQUIRE(kResult.IsSuccess());
      }
    }

    SECTION("invalid branch returns kInvalidBranch") {
      // Arrange:
      constexpr int kInvalidBranch = 4;

      // Act:
      const auto kResult =
          Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd, kInvalidBranch);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidBranch);
    }

    SECTION("negative branch returns kInvalidBranch") {
      // Arrange:
      constexpr int kNegativeBranch = -1;

      // Act:
      const auto kResult =
          Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd, kNegativeBranch);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidBranch);
    }

    SECTION("valid branch with coincident points returns kCoincidentPoints") {
      // Arrange: valid branch index but invalid geometry — exercises the geometry
      // validation path inside the branch-explicit Interpolate overload.
      const PlanarPoint kSamePoint{1.0, 1.0};
      constexpr int kBranch = 0;

      // Act:
      // NOLINTNEXTLINE(readability-suspicious-call-argument)
      const auto kResult =
          Quintic::Interpolate(kSamePoint, kSamePoint, kTangentStart, kTangentEnd, kBranch);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kCoincidentPoints);
    }

    SECTION("coincident points returns kCoincidentPoints") {
      // Arrange:
      const PlanarPoint kSamePoint{1.0, 1.0};

      // Act:
      // NOLINTNEXTLINE(readability-suspicious-call-argument)
      const auto kResult = Quintic::Interpolate(kSamePoint, kSamePoint, kTangentStart, kTangentEnd);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kCoincidentPoints);
    }

    SECTION("zero tangent at start returns kZeroTangent") {
      // Arrange:
      const PlanarPoint kZeroTangent{0.0, 0.0};

      // Act:
      const auto kResult = Quintic::Interpolate(kStart, kEnd, kZeroTangent, kTangentEnd);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kZeroTangent);
    }

    SECTION("zero tangent at end returns kZeroTangent") {
      // Arrange:
      const PlanarPoint kZeroTangent{0.0, 0.0};

      // Act:
      const auto kResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kZeroTangent);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kResult.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kZeroTangent);
    }
  }

  SECTION("Evaluate") {
    const auto kCurveResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);
    REQUIRE(kCurveResult.IsSuccess());
    const auto& curve = kCurveResult.Value();

    SECTION("at t=0 returns start point") {
      // Act:
      const auto kEval = curve.Evaluate(0.0);

      // Assert:
      REQUIRE(kEval.IsSuccess());
      REQUIRE_THAT(kEval.Value().x, Catch::Matchers::WithinAbs(kStart.x, kAbsTol));
      REQUIRE_THAT(kEval.Value().y, Catch::Matchers::WithinAbs(kStart.y, kAbsTol));
    }

    SECTION("at t=1 returns end point") {
      // Act:
      const auto kEval = curve.Evaluate(1.0);

      // Assert:
      REQUIRE(kEval.IsSuccess());
      REQUIRE_THAT(kEval.Value().x, Catch::Matchers::WithinRel(kEnd.x, kRelTol));
      REQUIRE_THAT(kEval.Value().y, Catch::Matchers::WithinRel(kEnd.y, kRelTol));
    }

    SECTION("at t=0.5 returns valid point") {
      // Arrange:
      constexpr double kMidParam = 0.5;

      // Act:
      const auto kEval = curve.Evaluate(kMidParam);

      // Assert:
      REQUIRE(kEval.IsSuccess());
      // Point should be finite (basic sanity check)
      REQUIRE(std::isfinite(kEval.Value().x));
      REQUIRE(std::isfinite(kEval.Value().y));
    }

    SECTION("out of range below 0 returns kInvalidParameter") {
      // Arrange:
      constexpr double kBelowRange = -0.1;

      // Act:
      const auto kEval = curve.Evaluate(kBelowRange);

      // Assert:
      REQUIRE_FALSE(kEval.IsSuccess());
      REQUIRE(kEval.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kEval.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidParameter);
    }

    SECTION("out of range above 1 returns kInvalidParameter") {
      // Arrange:
      constexpr double kAboveRange = 1.1;

      // Act:
      const auto kEval = curve.Evaluate(kAboveRange);

      // Assert:
      REQUIRE_FALSE(kEval.IsSuccess());
      REQUIRE(kEval.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kEval.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidParameter);
    }
  }

  SECTION("Tangent") {
    const auto kCurveResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);
    REQUIRE(kCurveResult.IsSuccess());
    const auto& curve = kCurveResult.Value();

    SECTION("unit norm at t=0") {
      // Act:
      const auto kTan = curve.Tangent(0.0);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      const double kNorm = kTan.Value().Norm();
      REQUIRE_THAT(kNorm, Catch::Matchers::WithinRel(1.0, kRelTol));
    }

    SECTION("unit norm at t=1") {
      // Act:
      const auto kTan = curve.Tangent(1.0);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      const double kNorm = kTan.Value().Norm();
      REQUIRE_THAT(kNorm, Catch::Matchers::WithinRel(1.0, kRelTol));
    }

    SECTION("direction matches tangent_start at t=0") {
      // Act:
      const auto kTan = curve.Tangent(0.0);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      // Unit tangent at t=0 should match kTangentStart direction
      REQUIRE_THAT(kTan.Value().x, Catch::Matchers::WithinRel(kTangentStart.x, kRelTol));
      REQUIRE_THAT(kTan.Value().y, Catch::Matchers::WithinAbs(kTangentStart.y, kAbsTol));
    }

    SECTION("direction matches tangent_end at t=1") {
      // Act:
      const auto kTan = curve.Tangent(1.0);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      REQUIRE_THAT(kTan.Value().x, Catch::Matchers::WithinAbs(kTangentEnd.x, kAbsTol));
      REQUIRE_THAT(kTan.Value().y, Catch::Matchers::WithinRel(kTangentEnd.y, kRelTol));
    }

    SECTION("out of range returns kInvalidParameter") {
      // Arrange:
      constexpr double kAboveRange = 1.5;

      // Act:
      const auto kTan = curve.Tangent(kAboveRange);

      // Assert:
      REQUIRE_FALSE(kTan.IsSuccess());
      REQUIRE(kTan.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kTan.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidParameter);
    }
  }

  SECTION("Normal") {
    const auto kCurveResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);
    REQUIRE(kCurveResult.IsSuccess());
    const auto& curve = kCurveResult.Value();

    SECTION("perpendicular to tangent at t=0") {
      // Act:
      const auto kTan = curve.Tangent(0.0);
      const auto kNorm = curve.Normal(0.0);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      REQUIRE(kNorm.IsSuccess());
      const double kDot = kTan.Value().Dot(kNorm.Value());
      REQUIRE_THAT(kDot, Catch::Matchers::WithinAbs(0.0, kAbsTol));
    }

    SECTION("perpendicular to tangent at t=0.5") {
      // Arrange:
      constexpr double kMidParam = 0.5;

      // Act:
      const auto kTan = curve.Tangent(kMidParam);
      const auto kNorm = curve.Normal(kMidParam);

      // Assert:
      REQUIRE(kTan.IsSuccess());
      REQUIRE(kNorm.IsSuccess());
      const double kDot = kTan.Value().Dot(kNorm.Value());
      REQUIRE_THAT(kDot, Catch::Matchers::WithinAbs(0.0, kAbsTol));
    }

    SECTION("unit norm at t=0.5") {
      // Arrange:
      constexpr double kMidParam = 0.5;

      // Act:
      const auto kNorm = curve.Normal(kMidParam);

      // Assert:
      REQUIRE(kNorm.IsSuccess());
      REQUIRE_THAT(kNorm.Value().Norm(), Catch::Matchers::WithinRel(1.0, kRelTol));
    }

    SECTION("out of range returns kInvalidParameter") {
      // Arrange:
      constexpr double kAboveRange = 1.5;

      // Act:
      const auto kNorm = curve.Normal(kAboveRange);

      // Assert:
      REQUIRE_FALSE(kNorm.IsSuccess());
      REQUIRE(kNorm.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kNorm.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidParameter);
    }
  }

  SECTION("ArcLength") {
    const auto kCurveResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);
    REQUIRE(kCurveResult.IsSuccess());
    const auto& curve = kCurveResult.Value();

    SECTION("monotonically increasing") {
      // Arrange:
      constexpr double kT1 = 0.25;
      constexpr double kT2 = 0.5;
      constexpr double kT3 = 0.75;

      // Act:
      const auto kLen1 = curve.ArcLength(kT1);
      const auto kLen2 = curve.ArcLength(kT2);
      const auto kLen3 = curve.ArcLength(kT3);

      // Assert:
      REQUIRE(kLen1.IsSuccess());
      REQUIRE(kLen2.IsSuccess());
      REQUIRE(kLen3.IsSuccess());
      REQUIRE(kLen1.Value() < kLen2.Value());
      REQUIRE(kLen2.Value() < kLen3.Value());
    }

    SECTION("zero at t=0") {
      // Act:
      const auto kLen = curve.ArcLength(0.0);

      // Assert:
      REQUIRE(kLen.IsSuccess());
      REQUIRE_THAT(kLen.Value(), Catch::Matchers::WithinAbs(0.0, kAbsTol));
    }

    SECTION("out of range returns kInvalidParameter") {
      // Arrange:
      constexpr double kAboveRange = 1.5;

      // Act:
      const auto kLen = curve.ArcLength(kAboveRange);

      // Assert:
      REQUIRE_FALSE(kLen.IsSuccess());
      REQUIRE(kLen.GetError().Is<Quintic::ErrorType>());
      REQUIRE(kLen.GetError().As<Quintic::ErrorType>() == Quintic::ErrorType::kInvalidParameter);
    }
  }

  SECTION("TotalLength") {
    const auto kCurveResult = Quintic::Interpolate(kStart, kEnd, kTangentStart, kTangentEnd);
    REQUIRE(kCurveResult.IsSuccess());
    const auto& curve = kCurveResult.Value();

    SECTION("positive for valid curve") {
      // Act:
      const auto kLen = curve.TotalLength();

      // Assert:
      REQUIRE(kLen.IsSuccess());
      REQUIRE(kLen.Value() > 0.0);
    }

    SECTION("equals ArcLength at t=1") {
      // Act:
      const auto kTotal = curve.TotalLength();
      const auto kArcAtOne = curve.ArcLength(1.0);

      // Assert:
      REQUIRE(kTotal.IsSuccess());
      REQUIRE(kArcAtOne.IsSuccess());
      REQUIRE_THAT(kTotal.Value(), Catch::Matchers::WithinRel(kArcAtOne.Value(), kRelTol));
    }

    SECTION("greater than Euclidean distance between endpoints") {
      // The arc length must be at least the straight-line distance.
      // Arrange:
      const double kEuclideanDist = kStart.DistanceTo(kEnd);

      // Act:
      const auto kLen = curve.TotalLength();

      // Assert:
      REQUIRE(kLen.IsSuccess());
      REQUIRE(kLen.Value() >= kEuclideanDist);
    }
  }
}

// NOLINTEND(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)
