#include "qcar2_planner/core/hodograph/planar_point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

// NOLINTBEGIN(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)

namespace {
constexpr double kRelTol = 0.001;
constexpr double kAbsTol = 1e-9;
}  // namespace

TEST_CASE("PlanarPoint Arithmetic Operators", "[core][hodograph]") {
  SECTION("addition returns component-wise sum") {
    // Arrange:
    constexpr double kLhsX = 1.0;
    constexpr double kLhsY = 2.0;
    constexpr double kRhsX = 3.0;
    constexpr double kRhsY = 4.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{kLhsX, kLhsY};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{kRhsX, kRhsY};

    // Act:
    const auto kResult = kLhs + kRhs;

    // Assert:
    REQUIRE_THAT(kResult.x, Catch::Matchers::WithinRel(kLhsX + kRhsX, kRelTol));
    REQUIRE_THAT(kResult.y, Catch::Matchers::WithinRel(kLhsY + kRhsY, kRelTol));
  }

  SECTION("subtraction returns component-wise difference") {
    // Arrange:
    constexpr double kLhsX = 5.0;
    constexpr double kLhsY = 7.0;
    constexpr double kRhsX = 2.0;
    constexpr double kRhsY = 3.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{kLhsX, kLhsY};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{kRhsX, kRhsY};

    // Act:
    const auto kResult = kLhs - kRhs;

    // Assert:
    REQUIRE_THAT(kResult.x, Catch::Matchers::WithinRel(kLhsX - kRhsX, kRelTol));
    REQUIRE_THAT(kResult.y, Catch::Matchers::WithinRel(kLhsY - kRhsY, kRelTol));
  }

  SECTION("subtraction of equal points yields zero") {
    // Arrange:
    constexpr double kPointX = 3.0;
    constexpr double kPointY = 5.0;
    const qcar2_planner::core::hodograph::PlanarPoint kPoint{kPointX, kPointY};

    // Act:
    const auto kResult = kPoint - kPoint;

    // Assert:
    REQUIRE_THAT(kResult.x, Catch::Matchers::WithinAbs(0.0, kAbsTol));
    REQUIRE_THAT(kResult.y, Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }

  SECTION("scalar multiplication scales both components") {
    // Arrange:
    constexpr double kLhsX = 2.0;
    constexpr double kLhsY = 3.0;
    constexpr double kScalar = 4.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{kLhsX, kLhsY};

    // Act:
    const auto kResult = kLhs * kScalar;

    // Assert:
    REQUIRE_THAT(kResult.x, Catch::Matchers::WithinRel(kLhsX * kScalar, kRelTol));
    REQUIRE_THAT(kResult.y, Catch::Matchers::WithinRel(kLhsY * kScalar, kRelTol));
  }

  SECTION("scalar division divides both components") {
    // Arrange:
    constexpr double kLhsX = 6.0;
    constexpr double kLhsY = 9.0;
    constexpr double kDivisor = 3.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{kLhsX, kLhsY};

    // Act:
    const auto kResult = kLhs / kDivisor;

    // Assert:
    REQUIRE_THAT(kResult.x, Catch::Matchers::WithinRel(kLhsX / kDivisor, kRelTol));
    REQUIRE_THAT(kResult.y, Catch::Matchers::WithinRel(kLhsY / kDivisor, kRelTol));
  }
}

TEST_CASE("PlanarPoint Comparison Operators", "[core][hodograph]") {
  SECTION("equal points are detected") {
    // Arrange:
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{1.0, 2.0};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{1.0, 2.0};

    // Assert:
    REQUIRE(kLhs == kRhs);
    REQUIRE_FALSE(kLhs != kRhs);
  }

  SECTION("unequal points are detected") {
    // Arrange:
    constexpr double kRhsY = 3.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{1.0, 2.0};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{1.0, kRhsY};

    // Assert:
    REQUIRE(kLhs != kRhs);
    REQUIRE_FALSE(kLhs == kRhs);
  }
}

TEST_CASE("PlanarPoint Geometric Operations", "[core][hodograph]") {
  SECTION("Norm returns vector magnitude") {
    // Arrange:
    constexpr double kPointX = 3.0;
    constexpr double kPointY = 4.0;
    constexpr double kExpectedNorm = 5.0;
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{kPointX, kPointY};

    // Assert:
    REQUIRE_THAT(kLhs.Norm(), Catch::Matchers::WithinRel(kExpectedNorm, kRelTol));
  }

  SECTION("Norm of zero vector is zero") {
    // Arrange:
    const qcar2_planner::core::hodograph::PlanarPoint kZeroPoint{0.0, 0.0};

    // Assert:
    REQUIRE_THAT(kZeroPoint.Norm(), Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }

  SECTION("DistanceTo returns Euclidean distance") {
    // Arrange:
    constexpr double kRhsX = 3.0;
    constexpr double kRhsY = 4.0;
    constexpr double kExpectedDistance = 5.0;
    const qcar2_planner::core::hodograph::PlanarPoint kOrigin{0.0, 0.0};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{kRhsX, kRhsY};

    // Assert:
    REQUIRE_THAT(kOrigin.DistanceTo(kRhs), Catch::Matchers::WithinRel(kExpectedDistance, kRelTol));
  }

  SECTION("DistanceTo identical points is zero") {
    // Arrange:
    const qcar2_planner::core::hodograph::PlanarPoint kPoint{1.0, 2.0};

    // Assert:
    REQUIRE_THAT(kPoint.DistanceTo(kPoint), Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }

  SECTION("Dot returns dot product") {
    // Arrange:
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{1.0, 0.0};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{0.0, 1.0};

    // Assert:
    REQUIRE_THAT(kLhs.Dot(kRhs), Catch::Matchers::WithinAbs(0.0, kAbsTol));
  }

  SECTION("Cross returns 2D scalar cross product") {
    // Arrange:
    const qcar2_planner::core::hodograph::PlanarPoint kLhs{1.0, 0.0};
    const qcar2_planner::core::hodograph::PlanarPoint kRhs{0.0, 1.0};

    // Assert:
    REQUIRE_THAT(kLhs.Cross(kRhs), Catch::Matchers::WithinRel(1.0, kRelTol));
  }
}

// NOLINTEND(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)
