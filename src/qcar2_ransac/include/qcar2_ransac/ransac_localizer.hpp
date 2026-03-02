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

#ifndef QCAR2_RANSAC__RANSAC_LOCALIZER_HPP_
#define QCAR2_RANSAC__RANSAC_LOCALIZER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <qcar2_msgs/msg/detection2_d.hpp>
#include <qcar2_msgs/msg/detection2_d_array.hpp>
#include <qcar2_msgs/msg/object3_d.hpp>
#include <qcar2_msgs/msg/object3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace qcar2_ransac {

namespace detail {
auto NormalizeClassName(const std::string& class_name) -> std::string;
}  // namespace detail

/**
 * @brief Structure for tracking object measurements over time
 */
struct ObjectTracker {
  std::string class_name;
  int class_id = -1;
  std::deque<Eigen::Vector3d> measurements;  // Recent position measurements in map frame
  Eigen::Vector3d confirmed_position;  // Final confirmed position
  bool is_confirmed = false;
  int confirmed_id = -1;  // Unique ID for confirmed objects of same class (e.g., Stop_1, Stop_2)
  double last_confidence = 0.0;
  rclcpp::Time last_seen;
  rclcpp::Time hold_until;  // For sample-and-hold update scheduling
};

/**
 * @brief Color definition for visualization
 */
struct MarkerColor {
  float r, g, b, a;
};

/**
 * @brief RANSAC-based object localizer node
 *
 * This node receives 3D positions from YOLO detections (already transformed in
 * qcar2_od), applies RANSAC for robust position estimation, transforms to
 * base_link/map frames, and visualizes in RViz2.
 */
class RansacLocalizer : public rclcpp::Node {
 public:
  explicit RansacLocalizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RansacLocalizer() override = default;

  RansacLocalizer(const RansacLocalizer&) = delete;
  RansacLocalizer(RansacLocalizer&&) = delete;
  auto operator=(const RansacLocalizer&) -> RansacLocalizer& = delete;
  auto operator=(RansacLocalizer&&) -> RansacLocalizer& = delete;

 private:
  // Callback functions
  auto DetectionCallback(const qcar2_msgs::msg::Detection2DArray::SharedPtr& msg) -> void;
  auto ProcessDetections() -> void;

  // Core processing functions
  auto DepthCameraToBaseLink(const Eigen::Vector3d& camera_point) const -> Eigen::Vector3d;

  auto BaseLinkToMap(const Eigen::Vector3d& base_link_pos,
                     const rclcpp::Time& stamp) const -> std::optional<Eigen::Vector3d>;

  // Object tracking functions
  auto UpdateTracker(const std::string& class_name, int class_id,
                     const Eigen::Vector3d& map_position, double confidence) -> void;

  auto UpdateConfirmedTracker(ObjectTracker& tracker, const Eigen::Vector3d& map_position,
                              double confidence, bool is_hold_class, bool is_dynamic_class,
                              const rclcpp::Time& now) const -> void;

  auto FindClosestTracker(std::vector<ObjectTracker>& class_trackers,
                          const Eigen::Vector3d& map_position) const -> ObjectTracker*;

  auto TryConfirmTracker(ObjectTracker& tracker, const std::string& class_name,
                         const std::vector<ObjectTracker>& class_trackers, bool is_hold_class,
                         const rclcpp::Time& now) -> void;

  auto UpdateExistingTracker(ObjectTracker& tracker, const std::string& class_name,
                             const std::vector<ObjectTracker>& class_trackers,
                             const Eigen::Vector3d& map_position, double confidence,
                             bool is_hold_class, const rclcpp::Time& now) -> void;

  auto IsTooCloseToExisting(const std::vector<ObjectTracker>& class_trackers,
                            const Eigen::Vector3d& map_position) const -> bool;

  auto CheckConfirmation(ObjectTracker& tracker) const -> bool;
  auto GetNextConfirmedId(const std::string& class_name) -> int;
  auto PruneTrackers(const rclcpp::Time& now) -> void;

  // Publishing functions
  auto PublishObjectPositions(const std::vector<qcar2_msgs::msg::Object3D>& objects,
                              const rclcpp::Time& stamp) -> void;

  auto PublishConfirmedObjects(const rclcpp::Time& stamp) -> void;
  auto PublishVisualization(const rclcpp::Time& stamp) -> void;

  // Utility functions
  auto GetColorForClass(const std::string& class_name) const -> MarkerColor;
  static auto CalculateStdDev(const std::deque<Eigen::Vector3d>& measurements) -> double;
  auto IsDynamicClass(const std::string& class_name) const -> bool;
  auto IsIgnoredClass(const std::string& class_name) const -> bool;
  auto IsSignalColorClass(const std::string& class_name) const -> bool;
  auto ShouldHoldPosition(const std::string& class_name) const -> bool;

  // Declare parameters
  auto DeclareParameters() -> void;

  // Subscribers
  rclcpp::Subscription<qcar2_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // Publishers
  rclcpp::Publisher<qcar2_msgs::msg::Object3DArray>::SharedPtr object_positions_pub_;
  rclcpp::Publisher<qcar2_msgs::msg::Object3DArray>::SharedPtr confirmed_objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr confirmed_markers_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data storage
  qcar2_msgs::msg::Detection2DArray::SharedPtr current_detections_;
  std::mutex detection_mutex_;

  // Object tracking
  std::map<std::string, std::vector<ObjectTracker>> trackers_;  // Key: class_name
  std::map<std::string, int> confirmed_counts_;  // Next ID for each class
  std::mutex tracker_mutex_;

  // Depth camera to base_link extrinsic
  Eigen::Matrix4d depth_to_baselink_extrinsic_;

  // Depth parameters
  double min_depth_;
  double max_depth_;
  double signal_color_max_depth_;

  // Tracking parameters
  double tracking_distance_;
  int min_measurements_;
  double std_threshold_;
  int moving_avg_window_;
  double duplicate_distance_;
  double dynamic_update_alpha_;
  double dynamic_timeout_sec_;
  double hold_position_sec_;

  // Dynamic classes (e.g., person, cone) keep updating after confirmation
  std::unordered_set<std::string> dynamic_classes_;
  std::unordered_set<std::string> ignored_classes_;
  std::unordered_set<std::string> signal_color_classes_;
  std::unordered_set<std::string> hold_excluded_classes_;

  // Timer for processing
  rclcpp::TimerBase::SharedPtr process_timer_;

  // Color map for visualization
  std::map<std::string, MarkerColor> color_map_;
};

}  // namespace qcar2_ransac

#endif  // QCAR2_RANSAC__RANSAC_LOCALIZER_HPP_
