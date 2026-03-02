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

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace qcar2_ransac {

namespace detail {

auto NormalizeClassName(const std::string& class_name) -> std::string {
  std::string normalized = class_name;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
  std::replace(normalized.begin(), normalized.end(), ' ', '_');
  std::replace(normalized.begin(), normalized.end(), '-', '_');
  return normalized;
}

}  // namespace detail

namespace {

constexpr double kDepthCamTx = 0.095;  // depth camera X translation to base_link [m]
constexpr double kDepthCamTy = 0.032;  // depth camera Y translation to base_link [m]
constexpr double kDepthCamTz = 0.172;  // depth camera Z translation to base_link [m]

constexpr float kHalfIntensity = 0.5F;  // 50% color channel intensity
constexpr float kGrayIntensity = 0.7F;  // 70% color channel intensity (default gray)

constexpr int kQueueDepth = 10;  // ROS 2 subscription/publisher queue depth
constexpr double kMsPerSecond = 1000.0;  // milliseconds per second
constexpr double kDefaultMinDepth = 0.1;  // default minimum valid depth [m]
constexpr double kDefaultMaxDepth = 5.0;  // default maximum valid depth [m]
constexpr double kDefaultSignalColorMaxDepth =
    2.0;  // default max depth for signal color labels [m]
constexpr int kDefaultMinMeasurements = 5;  // min measurements before confirmation
constexpr double kDefaultStdThreshold = 0.3;  // max std deviation for confirmation [m]
constexpr int kDefaultMovingAvgWindow = 10;  // window size for moving average
constexpr double kDefaultDuplicateDistance = 0.5;  // duplicate detection distance threshold [m]
constexpr double kDefaultDynamicUpdateAlpha = 0.4;  // EMA update ratio for dynamic classes
constexpr double kDefaultHoldPositionSec = 10.0;  // sample-and-hold interval [s]
constexpr double kDefaultProcessRate = 10.0;  // processing rate [Hz]
constexpr double kDefaultTrackingDistance = 1.0;  // max distance to associate with tracker [m]
constexpr double kDefaultDynamicTimeoutSec = 1.0;  // remove dynamic tracker if unseen this long [s]
constexpr double kTfLookupTimeoutSec = 0.1;  // TF lookup timeout [s]
constexpr double kMarkerSphereSize = 0.1;  // sphere marker diameter [m]
constexpr double kMarkerTextOffset = 0.15;  // text marker Z offset above sphere [m]
constexpr double kMarkerTextScale = 0.08;  // text marker scale [m]
constexpr float kColorFull = 1.0F;  // full color channel intensity
constexpr float kColorNone = 0.0F;  // zero color channel intensity

}  // namespace

RansacLocalizer::RansacLocalizer(const rclcpp::NodeOptions& options)
    : Node("ransac_localizer", options) {
  // Declare and get parameters
  DeclareParameters();

  // Depth camera -> base_link extrinsic matrix
  depth_to_baselink_extrinsic_ << 0.0, 0.0, 1.0, kDepthCamTx, -1.0, 0.0, 0.0, kDepthCamTy, 0.0,
      -1.0, 0.0, kDepthCamTz, 0.0, 0.0, 0.0, 1.0;

  // Depth parameters
  min_depth_ = this->get_parameter("min_depth").as_double();
  max_depth_ = this->get_parameter("max_depth").as_double();
  signal_color_max_depth_ = this->get_parameter("signal_color_max_depth").as_double();

  // Tracking parameters
  tracking_distance_ = this->get_parameter("tracking_distance").as_double();
  min_measurements_ = static_cast<int>(this->get_parameter("min_measurements").as_int());
  std_threshold_ = this->get_parameter("std_threshold").as_double();
  moving_avg_window_ = static_cast<int>(this->get_parameter("moving_avg_window").as_int());
  duplicate_distance_ = this->get_parameter("duplicate_distance").as_double();
  dynamic_update_alpha_ = this->get_parameter("dynamic_update_alpha").as_double();
  dynamic_timeout_sec_ = this->get_parameter("dynamic_timeout_sec").as_double();
  hold_position_sec_ = this->get_parameter("hold_position_sec").as_double();

  const auto kDynamicClasses = this->get_parameter("dynamic_classes").as_string_array();
  for (const auto& class_name : kDynamicClasses) {
    dynamic_classes_.insert(detail::NormalizeClassName(class_name));
  }
  const auto kIgnoredClasses = this->get_parameter("ignored_classes").as_string_array();
  for (const auto& class_name : kIgnoredClasses) {
    ignored_classes_.insert(detail::NormalizeClassName(class_name));
  }
  const auto kSignalColorClasses = this->get_parameter("signal_color_classes").as_string_array();
  for (const auto& class_name : kSignalColorClasses) {
    signal_color_classes_.insert(detail::NormalizeClassName(class_name));
  }
  const auto kHoldExcludedClasses = this->get_parameter("hold_excluded_classes").as_string_array();
  for (const auto& class_name : kHoldExcludedClasses) {
    hold_excluded_classes_.insert(detail::NormalizeClassName(class_name));
  }
  std::ostringstream dynamic_classes_stream;
  for (auto it = dynamic_classes_.begin(); it != dynamic_classes_.end(); ++it) {
    if (it != dynamic_classes_.begin()) {
      dynamic_classes_stream << ", ";
    }
    dynamic_classes_stream << *it;
  }
  std::ostringstream hold_excluded_classes_stream;
  for (auto it = hold_excluded_classes_.begin(); it != hold_excluded_classes_.end(); ++it) {
    if (it != hold_excluded_classes_.begin()) {
      hold_excluded_classes_stream << ", ";
    }
    hold_excluded_classes_stream << *it;
  }

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize color map for visualization
  color_map_["stop"] = {kColorFull, kColorNone, kColorNone, kColorFull};  // Red
  color_map_["yield"] = {kColorFull, kColorFull, kColorNone, kColorFull};  // Yellow
  color_map_["round"] = {kColorNone, kHalfIntensity, kHalfIntensity, kColorFull};  // Teal
  color_map_["speed"] = {kColorNone, kColorNone, kColorFull, kColorFull};  // Blue
  color_map_["traffic"] = {kColorNone, kColorFull, kColorNone, kColorFull};  // Green
  color_map_["cross"] = {kColorFull, kHalfIntensity, kColorNone, kColorFull};  // Orange
  color_map_["no_entry"] = {kHalfIntensity, kColorNone, kHalfIntensity, kColorFull};  // Purple
  color_map_["parking"] = {kColorNone, kColorFull, kColorFull, kColorFull};  // Cyan
  color_map_["one_way"] = {kColorFull, kColorNone, kColorFull, kColorFull};  // Magenta
  color_map_["pedestrian"] = {kHalfIntensity, kHalfIntensity, kColorNone, kColorFull};  // Olive
  color_map_["default"] = {kGrayIntensity, kGrayIntensity, kGrayIntensity, kColorFull};  // Gray

  // Subscribe to yolo_detections (already has 3D position from qcar2_od)
  detection_sub_ = this->create_subscription<qcar2_msgs::msg::Detection2DArray>(
      "/yolo_detections", kQueueDepth,
      [this](const qcar2_msgs::msg::Detection2DArray::SharedPtr msg) {  // NOLINT(performance-unnecessary-value-param)
        DetectionCallback(msg);
      });

  // Create publishers
  object_positions_pub_ =
      this->create_publisher<qcar2_msgs::msg::Object3DArray>("/object_positions", kQueueDepth);

  confirmed_objects_pub_ =
      this->create_publisher<qcar2_msgs::msg::Object3DArray>("/confirmed_objects", kQueueDepth);

  confirmed_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/confirmed_markers", kQueueDepth);

  // Create processing timer (10 Hz)
  double process_rate = this->get_parameter("process_rate").as_double();
  process_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(kMsPerSecond / process_rate)),
      [this]() { ProcessDetections(); });

  RCLCPP_INFO(this->get_logger(), "RANSAC Localizer initialized (simplified)");
  RCLCPP_INFO(this->get_logger(), "  - Using 3D positions from /yolo_detections");
  RCLCPP_INFO(this->get_logger(), "  - Depth range: %.2f - %.2f m", min_depth_, max_depth_);
  RCLCPP_INFO(this->get_logger(), "  - Signal color max depth: %.2f m", signal_color_max_depth_);
  RCLCPP_INFO(this->get_logger(), "  - Tracking distance: %.2f m, min measurements: %d",
              tracking_distance_, min_measurements_);
  RCLCPP_INFO(this->get_logger(), "  - Dynamic classes: [%s], alpha: %.2f, timeout: %.2fs",
              dynamic_classes_stream.str().c_str(), dynamic_update_alpha_, dynamic_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  - Hold update: %.2fs (excluded: [%s])", hold_position_sec_,
              hold_excluded_classes_stream.str().c_str());
}

auto RansacLocalizer::DeclareParameters() -> void {
  // Depth processing parameters
  // Minimum valid depth in meters
  this->declare_parameter("min_depth", kDefaultMinDepth);
  // Maximum valid depth in meters
  this->declare_parameter("max_depth", kDefaultMaxDepth);
  // Max depth for red/yellow/green labels
  this->declare_parameter("signal_color_max_depth", kDefaultSignalColorMaxDepth);
  this->declare_parameter("signal_color_classes",
                          std::vector<std::string>{"red", "yellow", "green"});

  // Object tracking parameters
  // Max distance to associate with existing tracker (m)
  this->declare_parameter("tracking_distance", kDefaultTrackingDistance);
  // Min measurements before confirmation
  this->declare_parameter("min_measurements", kDefaultMinMeasurements);
  // Max std deviation for confirmation (m)
  this->declare_parameter("std_threshold", kDefaultStdThreshold);
  // Window size for moving average
  this->declare_parameter("moving_avg_window", kDefaultMovingAvgWindow);
  // Distance threshold for duplicate detection (m)
  this->declare_parameter("duplicate_distance", kDefaultDuplicateDistance);
  this->declare_parameter("dynamic_classes", std::vector<std::string>{"*"});
  // EMA update ratio for dynamic classes
  this->declare_parameter("dynamic_update_alpha", kDefaultDynamicUpdateAlpha);
  // Remove dynamic tracker if unseen for this long
  this->declare_parameter("dynamic_timeout_sec", kDefaultDynamicTimeoutSec);
  // Sample-and-hold interval for non-excluded classes
  this->declare_parameter("hold_position_sec", kDefaultHoldPositionSec);
  this->declare_parameter("hold_excluded_classes",
                          std::vector<std::string>{"person", "red", "green", "yellow"});
  this->declare_parameter(
      "ignored_classes",
      std::vector<std::string>{"traffic_light", "traffic light", "red_traffic_light",
                               "yellow_traffic_light", "green_traffic_light", "red light",
                               "yellow light", "green light"});

  // Processing rate
  // Processing rate in Hz
  this->declare_parameter("process_rate", kDefaultProcessRate);
}

auto RansacLocalizer::DetectionCallback(const qcar2_msgs::msg::Detection2DArray::SharedPtr& msg)
    -> void {
  std::lock_guard<std::mutex> lock(detection_mutex_);
  current_detections_ = msg;
}

auto RansacLocalizer::ProcessDetections() -> void {
  // Get current detections
  qcar2_msgs::msg::Detection2DArray::SharedPtr detections;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (!current_detections_) {
      return;
    }
    detections = current_detections_;
  }

  std::vector<qcar2_msgs::msg::Object3D> object_positions;
  rclcpp::Time stamp = this->now();

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received %zu detections",
                       detections->detections.size());

  for (const auto& detection : detections->detections) {
    if (IsIgnoredClass(detection.class_name)) {
      continue;
    }

    const double kSampledDepth =
        detection.depth_sample > 0.0 ? detection.depth_sample : detection.depth;

    const double kClassMaxDepth = IsSignalColorClass(detection.class_name)
                                      ? std::max(max_depth_, signal_color_max_depth_)
                                      : max_depth_;

    // Skip if no valid depth (qcar2_od sets depth=0 if invalid)
    if (kSampledDepth <= 0.0 || kSampledDepth < min_depth_ || kSampledDepth > kClassMaxDepth) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[%s] Invalid depth: %.3f (range: %.1f-%.1f)",
                           detection.class_name.c_str(), kSampledDepth, min_depth_, kClassMaxDepth);
      continue;
    }

    if (detection.depth_sample_u < 0 || detection.depth_sample_v < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[%s] Missing aligned depth sample pixel in /yolo_detections",
                           detection.class_name.c_str());
    }

    // Get 3D position in depth camera frame (already computed in qcar2_od)
    Eigen::Vector3d camera_point(detection.position_x, detection.position_y, detection.position_z);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[%s] depth=%.2fm, sample_px=(%d,%d), camera=(%.2f, %.2f, %.2f)",
                         detection.class_name.c_str(), kSampledDepth, detection.depth_sample_u,
                         detection.depth_sample_v, camera_point.x(), camera_point.y(),
                         camera_point.z());

    // Transform to base_link frame
    Eigen::Vector3d base_link_pos = DepthCameraToBaseLink(camera_point);

    // Create Object3D for base_link frame
    qcar2_msgs::msg::Object3D obj;
    obj.class_name = detection.class_name;
    obj.class_id = detection.class_id;
    obj.confidence = detection.confidence;
    obj.position.x = base_link_pos.x();
    obj.position.y = base_link_pos.y();
    obj.position.z = base_link_pos.z();
    obj.distance = static_cast<float>(base_link_pos.norm());
    object_positions.push_back(obj);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[%s] base_link=(%.2f, %.2f, %.2f)", detection.class_name.c_str(),
                         base_link_pos.x(), base_link_pos.y(), base_link_pos.z());

    // Transform to map frame for tracking
    auto map_pos = BaseLinkToMap(base_link_pos, stamp);
    if (map_pos) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[%s] map=(%.2f, %.2f, %.2f)", detection.class_name.c_str(),
                           map_pos->x(), map_pos->y(), map_pos->z());

      std::lock_guard<std::mutex> lock(tracker_mutex_);
      UpdateTracker(detection.class_name, detection.class_id, *map_pos, detection.confidence);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[%s] TF base_link->map failed!", detection.class_name.c_str());
    }
  }

  PruneTrackers(stamp);

  // Publish results
  PublishObjectPositions(object_positions, stamp);
  PublishConfirmedObjects(stamp);
  PublishVisualization(stamp);
}

auto RansacLocalizer::DepthCameraToBaseLink(const Eigen::Vector3d& camera_point) const
    -> Eigen::Vector3d {
  // Create homogeneous point
  Eigen::Vector4d cam_point(camera_point.x(), camera_point.y(), camera_point.z(), 1.0);

  // Transform to base_link frame
  Eigen::Vector4d base_point = depth_to_baselink_extrinsic_ * cam_point;

  return {base_point.x(), base_point.y(), base_point.z()};
}

auto RansacLocalizer::BaseLinkToMap(const Eigen::Vector3d& base_link_pos, const rclcpp::Time& stamp)
    const -> std::optional<Eigen::Vector3d> {
  geometry_msgs::msg::PointStamped base_point;
  base_point.header.frame_id = "base_link";
  base_point.header.stamp = stamp;
  base_point.point.x = base_link_pos.x();
  base_point.point.y = base_link_pos.y();
  base_point.point.z = base_link_pos.z();

  try {
    geometry_msgs::msg::PointStamped map_point;
    map_point = tf_buffer_->transform(base_point, "map", tf2::durationFromSec(kTfLookupTimeoutSec));
    return Eigen::Vector3d(map_point.point.x, map_point.point.y, map_point.point.z);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(this->get_logger(), "TF transform failed: %s", ex.what());
    return std::nullopt;
  }
}

auto RansacLocalizer::UpdateConfirmedTracker(ObjectTracker& tracker,
                                             const Eigen::Vector3d& map_position, double confidence,
                                             bool is_hold_class, bool is_dynamic_class,
                                             const rclcpp::Time& now) const -> void {
  if (is_hold_class) {
    if (tracker.hold_until.nanoseconds() == 0 ||
        now.nanoseconds() >= tracker.hold_until.nanoseconds()) {
      tracker.confirmed_position = map_position;
      tracker.hold_until = now + rclcpp::Duration::from_seconds(hold_position_sec_);
    }
    tracker.measurements.push_back(map_position);
    if (static_cast<int>(tracker.measurements.size()) > moving_avg_window_) {
      tracker.measurements.pop_front();
    }
    tracker.last_confidence = confidence;
  } else if (is_dynamic_class) {
    // Dynamic objects (e.g., person/cone): keep confirmed position moving
    // over time.
    tracker.confirmed_position = (1.0 - dynamic_update_alpha_) * tracker.confirmed_position +
                                 dynamic_update_alpha_ * map_position;
    tracker.measurements.push_back(map_position);
    if (static_cast<int>(tracker.measurements.size()) > moving_avg_window_) {
      tracker.measurements.pop_front();
    }
    tracker.last_confidence = confidence;
  }
  tracker.last_seen = now;
}

auto RansacLocalizer::FindClosestTracker(std::vector<ObjectTracker>& class_trackers,
                                         const Eigen::Vector3d& map_position) const
    -> ObjectTracker* {
  ObjectTracker* closest_tracker = nullptr;
  double min_distance = std::numeric_limits<double>::max();

  for (auto& tracker : class_trackers) {
    if (tracker.is_confirmed || tracker.measurements.empty()) {
      continue;
    }

    Eigen::Vector3d tracker_pos = Eigen::Vector3d::Zero();
    for (const auto& meas : tracker.measurements) {
      tracker_pos += meas;
    }
    tracker_pos /= static_cast<double>(tracker.measurements.size());

    double dist = (tracker_pos.head<2>() - map_position.head<2>()).norm();
    if (dist < min_distance && dist < tracking_distance_) {
      min_distance = dist;
      closest_tracker = &tracker;
    }
  }

  return closest_tracker;
}

auto RansacLocalizer::TryConfirmTracker(ObjectTracker& tracker, const std::string& class_name,
                                        const std::vector<ObjectTracker>& class_trackers,
                                        bool is_hold_class, const rclcpp::Time& now) -> void {
  if (!CheckConfirmation(tracker)) {
    return;
  }

  Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
  for (const auto& meas : tracker.measurements) {
    avg_pos += meas;
  }
  avg_pos /= static_cast<double>(tracker.measurements.size());

  // Check for duplicates before confirming
  for (const auto& existing : class_trackers) {
    if (!existing.is_confirmed) {
      continue;
    }
    double dist = (existing.confirmed_position.head<2>() - avg_pos.head<2>()).norm();
    if (dist < duplicate_distance_) {
      return;
    }
  }

  tracker.confirmed_position = avg_pos;
  tracker.is_confirmed = true;
  tracker.confirmed_id = GetNextConfirmedId(class_name);
  if (is_hold_class) {
    tracker.hold_until = now + rclcpp::Duration::from_seconds(hold_position_sec_);
  }

  RCLCPP_INFO(this->get_logger(), "Confirmed: %s_%d at map (%.2f, %.2f)", class_name.c_str(),
              tracker.confirmed_id, avg_pos.x(), avg_pos.y());
}

auto RansacLocalizer::UpdateExistingTracker(ObjectTracker& tracker, const std::string& class_name,
                                            const std::vector<ObjectTracker>& class_trackers,
                                            const Eigen::Vector3d& map_position, double confidence,
                                            bool is_hold_class, const rclcpp::Time& now) -> void {
  tracker.measurements.push_back(map_position);
  if (static_cast<int>(tracker.measurements.size()) > moving_avg_window_) {
    tracker.measurements.pop_front();
  }

  double std_dev = CalculateStdDev(tracker.measurements);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Tracker [%s]: measurements=%zu/%d, std=%.3f/%.3f", class_name.c_str(),
                       tracker.measurements.size(), min_measurements_, std_dev, std_threshold_);

  TryConfirmTracker(tracker, class_name, class_trackers, is_hold_class, now);

  tracker.last_confidence = confidence;
  tracker.last_seen = now;
}

auto RansacLocalizer::IsTooCloseToExisting(const std::vector<ObjectTracker>& class_trackers,
                                           const Eigen::Vector3d& map_position) const -> bool {
  for (const auto& tracker : class_trackers) {
    Eigen::Vector3d tracker_pos;
    if (tracker.is_confirmed) {
      tracker_pos = tracker.confirmed_position;
    } else if (!tracker.measurements.empty()) {
      tracker_pos = Eigen::Vector3d::Zero();
      for (const auto& meas : tracker.measurements) {
        tracker_pos += meas;
      }
      tracker_pos /= static_cast<double>(tracker.measurements.size());
    } else {
      continue;
    }

    double dist = (tracker_pos.head<2>() - map_position.head<2>()).norm();
    if (dist < tracking_distance_) {
      return true;
    }
  }
  return false;
}

auto RansacLocalizer::UpdateTracker(const std::string& class_name, int class_id,
                                    const Eigen::Vector3d& map_position,
                                    double confidence) -> void {
  auto& class_trackers = trackers_[class_name];
  const bool kDynamicClass = IsDynamicClass(class_name);
  const bool kHoldClass = ShouldHoldPosition(class_name);
  const auto kNow = this->now();

  // Check against confirmed objects to avoid duplicates
  for (auto& tracker : class_trackers) {
    if (!tracker.is_confirmed) {
      continue;
    }
    double dist = (tracker.confirmed_position.head<2>() - map_position.head<2>()).norm();
    const double kAssociationThreshold =
        (kDynamicClass || kHoldClass) ? tracking_distance_ : duplicate_distance_;
    if (dist < kAssociationThreshold) {
      UpdateConfirmedTracker(tracker, map_position, confidence, kHoldClass, kDynamicClass, kNow);
      return;
    }
  }

  // Find closest non-confirmed tracker
  ObjectTracker* closest_tracker = FindClosestTracker(class_trackers, map_position);

  if (closest_tracker) {
    UpdateExistingTracker(*closest_tracker, class_name, class_trackers, map_position, confidence,
                          kHoldClass, kNow);
  } else if (!IsTooCloseToExisting(class_trackers, map_position)) {
    // Create new tracker
    ObjectTracker new_tracker;
    new_tracker.class_name = class_name;
    new_tracker.class_id = class_id;
    new_tracker.measurements.push_back(map_position);
    new_tracker.last_confidence = confidence;
    new_tracker.last_seen = kNow;
    class_trackers.push_back(new_tracker);
  }
}

auto RansacLocalizer::CheckConfirmation(ObjectTracker& tracker) const -> bool {
  if (static_cast<int>(tracker.measurements.size()) < min_measurements_) {
    return false;
  }

  double std_dev = CalculateStdDev(tracker.measurements);
  return std_dev < std_threshold_;
}

auto RansacLocalizer::CalculateStdDev(const std::deque<Eigen::Vector3d>& measurements) -> double {
  if (measurements.size() < 2) {
    return std::numeric_limits<double>::max();
  }

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& meas : measurements) {
    mean += meas.head<2>();
  }
  mean /= static_cast<double>(measurements.size());

  double variance = 0.0;
  for (const auto& meas : measurements) {
    variance += (meas.head<2>() - mean).squaredNorm();
  }
  variance /= static_cast<double>(measurements.size() - 1);

  return std::sqrt(variance);
}

auto RansacLocalizer::GetNextConfirmedId(const std::string& class_name) -> int {
  if (confirmed_counts_.find(class_name) == confirmed_counts_.end()) {
    confirmed_counts_[class_name] = 0;
  }
  return ++confirmed_counts_[class_name];
}

auto RansacLocalizer::PruneTrackers(const rclcpp::Time& now) -> void {
  std::lock_guard<std::mutex> lock(tracker_mutex_);

  std::size_t removed_count = 0;
  for (auto class_it = trackers_.begin(); class_it != trackers_.end();) {
    const std::string& class_name = class_it->first;
    auto& class_trackers = class_it->second;

    auto erase_begin = std::remove_if(
        class_trackers.begin(), class_trackers.end(), [&](const ObjectTracker& tracker) {
          if (!IsDynamicClass(class_name)) {
            return false;
          }
          if (tracker.last_seen.nanoseconds() == 0) {
            return false;
          }
          const double kUnseenSec = (now - tracker.last_seen).seconds();
          return kUnseenSec > dynamic_timeout_sec_;
        });

    removed_count += static_cast<std::size_t>(std::distance(erase_begin, class_trackers.end()));
    class_trackers.erase(erase_begin, class_trackers.end());

    if (class_trackers.empty()) {
      class_it = trackers_.erase(class_it);
    } else {
      ++class_it;
    }
  }

  if (removed_count > 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Removed %zu stale dynamic tracker(s)", removed_count);
  }
}

auto RansacLocalizer::PublishObjectPositions(const std::vector<qcar2_msgs::msg::Object3D>& objects,
                                             const rclcpp::Time& stamp) -> void {
  qcar2_msgs::msg::Object3DArray msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;
  msg.objects = objects;
  object_positions_pub_->publish(msg);
}

auto RansacLocalizer::PublishConfirmedObjects(const rclcpp::Time& stamp) -> void {
  std::lock_guard<std::mutex> lock(tracker_mutex_);

  qcar2_msgs::msg::Object3DArray msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;

  for (const auto& [class_name, class_trackers] : trackers_) {
    if (IsIgnoredClass(class_name)) {
      continue;
    }

    for (const auto& tracker : class_trackers) {
      if (!tracker.is_confirmed) {
        continue;
      }

      qcar2_msgs::msg::Object3D obj;
      obj.class_name = class_name + "_" + std::to_string(tracker.confirmed_id);
      obj.class_id = tracker.class_id;
      obj.confidence = static_cast<float>(tracker.last_confidence);
      obj.position.x = tracker.confirmed_position.x();
      obj.position.y = tracker.confirmed_position.y();
      obj.position.z = tracker.confirmed_position.z();
      obj.distance = static_cast<float>(tracker.confirmed_position.norm());
      msg.objects.push_back(obj);
    }
  }

  confirmed_objects_pub_->publish(msg);
}

auto RansacLocalizer::PublishVisualization(const rclcpp::Time& stamp) -> void {
  std::lock_guard<std::mutex> lock(tracker_mutex_);

  visualization_msgs::msg::MarkerArray confirmed_markers;

  // Delete all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.header.stamp = stamp;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  confirmed_markers.markers.push_back(delete_marker);

  int marker_id = 0;

  for (const auto& [class_name, class_trackers] : trackers_) {
    if (IsIgnoredClass(class_name)) {
      continue;
    }

    MarkerColor color = GetColorForClass(class_name);

    for (const auto& tracker : class_trackers) {
      if (!tracker.is_confirmed) {
        continue;
      }

      // Sphere marker for position
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = stamp;
      sphere.ns = "signs";
      sphere.id = marker_id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = tracker.confirmed_position.x();
      sphere.pose.position.y = tracker.confirmed_position.y();
      sphere.pose.position.z = tracker.confirmed_position.z();
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = kMarkerSphereSize;
      sphere.scale.y = kMarkerSphereSize;
      sphere.scale.z = kMarkerSphereSize;
      sphere.color.r = color.r;
      sphere.color.g = color.g;
      sphere.color.b = color.b;
      sphere.color.a = color.a;
      confirmed_markers.markers.push_back(sphere);

      // Text marker for label
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "map";
      text.header.stamp = stamp;
      text.ns = "labels";
      text.id = marker_id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = tracker.confirmed_position.x();
      text.pose.position.y = tracker.confirmed_position.y();
      text.pose.position.z = tracker.confirmed_position.z() + kMarkerTextOffset;
      text.pose.orientation.w = 1.0;
      text.scale.z = kMarkerTextScale;
      text.color.r = kColorFull;
      text.color.g = kColorFull;
      text.color.b = kColorFull;
      text.color.a = kColorFull;

      std::string label = class_name;
      if (!label.empty()) {
        label[0] = static_cast<char>(std::toupper(label[0]));
      }
      label += "_" + std::to_string(tracker.confirmed_id);
      text.text = label;

      confirmed_markers.markers.push_back(text);
    }
  }

  confirmed_markers_pub_->publish(confirmed_markers);
}

auto RansacLocalizer::GetColorForClass(const std::string& class_name) const -> MarkerColor {
  auto color_iter = color_map_.find(class_name);
  if (color_iter != color_map_.end()) {
    return color_iter->second;
  }

  std::string lower_name = class_name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

  for (const auto& [key, color] : color_map_) {
    if (lower_name.find(key) != std::string::npos) {
      return color;
    }
  }

  return color_map_.at("default");
}

auto RansacLocalizer::IsDynamicClass(const std::string& class_name) const -> bool {
  if (dynamic_classes_.find("*") != dynamic_classes_.end()) {
    return true;
  }

  const std::string kNormalizedName = detail::NormalizeClassName(class_name);

  return std::any_of(dynamic_classes_.begin(), dynamic_classes_.end(),
                     [&kNormalizedName](const std::string& dynamic_name) {
                       return kNormalizedName == dynamic_name ||
                              kNormalizedName.find(dynamic_name) != std::string::npos;
                     });
}

auto RansacLocalizer::IsIgnoredClass(const std::string& class_name) const -> bool {
  const std::string kNormalizedName = detail::NormalizeClassName(class_name);

  return std::any_of(ignored_classes_.begin(), ignored_classes_.end(),
                     [&kNormalizedName](const std::string& ignored_name) {
                       return kNormalizedName == ignored_name ||
                              kNormalizedName.find(ignored_name) != std::string::npos;
                     });
}

auto RansacLocalizer::ShouldHoldPosition(const std::string& class_name) const -> bool {
  const std::string kNormalizedName = detail::NormalizeClassName(class_name);

  return std::all_of(hold_excluded_classes_.begin(), hold_excluded_classes_.end(),
                     [&kNormalizedName](const std::string& excluded_name) {
                       return kNormalizedName != excluded_name &&
                              kNormalizedName.find(excluded_name) == std::string::npos;
                     });
}

auto RansacLocalizer::IsSignalColorClass(const std::string& class_name) const -> bool {
  const std::string kNormalizedName = detail::NormalizeClassName(class_name);

  return std::any_of(signal_color_classes_.begin(), signal_color_classes_.end(),
                     [&kNormalizedName](const std::string& signal_color_name) {
                       return kNormalizedName == signal_color_name;
                     });
}

}  // namespace qcar2_ransac
