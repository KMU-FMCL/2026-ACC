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

#include <rclcpp/rclcpp.hpp>

#include "qcar2_ransac/ransac_localizer.hpp"

auto main(int argc, char** argv) -> int {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<qcar2_ransac::RansacLocalizer>();

  RCLCPP_INFO(node->get_logger(), "RANSAC Localizer node started");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
