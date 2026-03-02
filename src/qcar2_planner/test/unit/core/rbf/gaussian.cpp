#include "qcar2_planner/core/rbf/gaussian.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

// NOLINTBEGIN(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)

namespace {
constexpr double kRelTol = 0.001;
constexpr double kAbsTol = 1e-9;
}  // namespace

using qcar2_planner::core::rbf::Gaussian;

TEST_CASE("Gaussian", "[core][rbf]") {
  SECTION("Create") {
    SECTION("valid epsilon succeeds") {
      // Arrange:
      constexpr double kEpsilon = 1.0;
      constexpr double kCenter = 0.0;

      // Act:
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);

      // Assert:
      REQUIRE(kResult.IsSuccess());
    }

    SECTION("zero epsilon returns kInvalidEpsilon") {
      // Arrange:
      constexpr double kEpsilon = 0.0;
      constexpr double kCenter = 0.0;

      // Act:
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Gaussian::ErrorType>());
      REQUIRE(kResult.GetError().As<Gaussian::ErrorType>() == Gaussian::ErrorType::kInvalidEpsilon);
    }

    SECTION("negative epsilon returns kInvalidEpsilon") {
      // Arrange:
      constexpr double kEpsilon = -0.5;
      constexpr double kCenter = 0.0;

      // Act:
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);

      // Assert:
      REQUIRE_FALSE(kResult.IsSuccess());
      REQUIRE(kResult.GetError().Is<Gaussian::ErrorType>());
      REQUIRE(kResult.GetError().As<Gaussian::ErrorType>() == Gaussian::ErrorType::kInvalidEpsilon);
    }
  }

  SECTION("Evaluate") {
    SECTION("at center returns 1.0") {
      // Arrange:
      constexpr double kEpsilon = 2.0;
      constexpr double kCenter = 3.0;
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);
      REQUIRE(kResult.IsSuccess());
      const auto& gaussian = kResult.Value();

      // Act:
      const auto kVal = gaussian.Evaluate(kCenter);

      // Assert:
      REQUIRE_THAT(kVal, Catch::Matchers::WithinAbs(1.0, kAbsTol));  // e^0 = 1.0 exactly
    }

    SECTION("away from center decays") {
      // Arrange:
      constexpr double kEpsilon = 1.0;
      constexpr double kCenter = 0.0;
      constexpr double kOffset = 1.0;
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);
      REQUIRE(kResult.IsSuccess());
      const auto& gaussian = kResult.Value();

      // Act:
      const auto kVal = gaussian.Evaluate(kCenter + kOffset);

      // Assert:
      REQUIRE(kVal < 1.0);
    }

    SECTION("symmetric around center") {
      // Arrange:
      constexpr double kEpsilon = 1.5;
      constexpr double kCenter = 2.0;
      constexpr double kOffset = 0.5;
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);
      REQUIRE(kResult.IsSuccess());
      const auto& gaussian = kResult.Value();

      // Act:
      const auto kValPlus = gaussian.Evaluate(kCenter + kOffset);
      const auto kValMinus = gaussian.Evaluate(kCenter - kOffset);

      // Assert:
      REQUIRE_THAT(kValPlus, Catch::Matchers::WithinRel(kValMinus, kRelTol));
    }
  }

  SECTION("Epsilon") {
    SECTION("returns constructed value") {
      // Arrange:
      constexpr double kEpsilon = 3.5;
      constexpr double kCenter = 1.0;
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);
      REQUIRE(kResult.IsSuccess());

      // Act:
      const auto kReturned = kResult.Value().Epsilon();

      // Assert:
      REQUIRE_THAT(kReturned, Catch::Matchers::WithinRel(kEpsilon, kRelTol));
    }
  }

  SECTION("Center") {
    SECTION("returns constructed value") {
      // Arrange:
      constexpr double kEpsilon = 1.0;
      constexpr double kCenter = -2.5;
      const auto kResult = Gaussian::Create(kEpsilon, kCenter);
      REQUIRE(kResult.IsSuccess());

      // Act:
      const auto kReturned = kResult.Value().Center();

      // Assert:
      REQUIRE_THAT(kReturned, Catch::Matchers::WithinAbs(kCenter, kAbsTol));  // exact accessor
    }
  }
}

// NOLINTEND(cert-err58-cpp, cppcoreguidelines-avoid-do-while, misc-use-anonymous-namespace)
