// Copyright 2026 FMCL (Future Mobility Control Lab), Kookmin University
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "qcar2_ransac/ransac_localizer.hpp"

#include <catch2/catch_test_macros.hpp>

// NOLINTBEGIN(cppcoreguidelines-avoid-do-while)
TEST_CASE("Lowercase Input Is Returned Unchanged", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "stop";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "stop");
}

TEST_CASE("Uppercase Letters Are Converted To Lowercase", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "Stop";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "stop");
}

TEST_CASE("Mixed Case Is Fully Lowercased", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "TrafficLight";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "trafficlight");
}

TEST_CASE("Space Is Replaced With Underscore", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "traffic light";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "traffic_light");
}

TEST_CASE("Hyphen Is Replaced With Underscore", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "no-entry";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "no_entry");
}

TEST_CASE("Mixed Case With Space Is Lowercased And Space Replaced", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "Red Light";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "red_light");
}

TEST_CASE("Empty String Is Returned Unchanged", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "");
}

TEST_CASE("All Spaces Are Replaced With Underscores", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "  ";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "__");
}

TEST_CASE("Multiple Spaces Are Each Replaced With An Underscore", "[ransac-localizer][normalize-class-name]")
{
  const std::string kInput = "a b";

  const std::string kResult = qcar2_ransac::detail::NormalizeClassName(kInput);

  REQUIRE(kResult == "a_b");
}
// NOLINTEND(cppcoreguidelines-avoid-do-while)
