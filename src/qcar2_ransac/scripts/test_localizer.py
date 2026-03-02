#!/usr/bin/env python3
# Copyright 2026 FMCL (Future Mobility Control Lab), Kookmin University
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
Test Localizer Node (Python).

Computes and visualizes global sign coordinates using the depth value at the YOLO bbox center.
Handles QLabs depth-camera noise with enhanced outlier rejection.

Subscribe:
  - /yolo_detections (qcar2_msgs/Detection2DArray)
  - /camera/depth_image (sensor_msgs/Image, mono16)

Publish:
  - /test/object_positions (qcar2_msgs/Object3DArray, in base_link frame)
  - /test/confirmed_objects (qcar2_msgs/Object3DArray, in map frame)
  - /test/markers (visualization_msgs/MarkerArray)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from qcar2_msgs.msg import Detection2DArray, Object3D, Object3DArray

import tf2_ros

from dataclasses import dataclass, field
from typing import Optional, Dict, List, Deque
from collections import deque


@dataclass
class CameraIntrinsics:
    """Camera intrinsics."""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class ObjectTracker:
    """Object tracking state."""

    class_name: str
    class_id: int
    measurements: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=30))
    confirmed_position: Optional[np.ndarray] = None
    is_confirmed: bool = False
    confirmed_id: int = -1
    last_confidence: float = 0.0


def _median_pos(measurements: Deque[np.ndarray]) -> np.ndarray:
    return np.median(np.array(measurements), axis=0)


class TestLocalizer(Node):
    """Python-based test localizer."""

    # Per-sign color map (RGBA)
    COLOR_MAP = {
        'stop': (1.0, 0.0, 0.0, 1.0),      # Red
        'yield': (1.0, 1.0, 0.0, 1.0),     # Yellow
        'round': (0.0, 0.5, 0.5, 1.0),     # Teal
        'speed': (0.0, 0.0, 1.0, 1.0),     # Blue
        'traffic': (0.0, 1.0, 0.0, 1.0),   # Green
        'default': (0.7, 0.7, 0.7, 1.0),   # Gray
    }

    def __init__(self):
        super().__init__('test_localizer')

        # ========================================
        # Parameters
        # ========================================
        # RGB camera intrinsics (640x480)
        self.declare_parameter('rgb_fx', 455.2)
        self.declare_parameter('rgb_fy', 459.43)
        self.declare_parameter('rgb_cx', 308.53)
        self.declare_parameter('rgb_cy', 213.56)

        # Depth camera intrinsics (640x480)
        self.declare_parameter('depth_fx', 385.6)
        self.declare_parameter('depth_fy', 385.6)
        self.declare_parameter('depth_cx', 321.9)
        self.declare_parameter('depth_cy', 237.3)

        # Depth processing parameters
        self.declare_parameter('min_depth', 0.2)      # Minimum valid range (m)
        self.declare_parameter('max_depth', 5.0)      # Maximum valid range (m)
        self.declare_parameter('depth_scale', 0.001)  # mm -> m conversion

        # ROI parameter
        self.declare_parameter('bbox_scale', 0.4)     # Use the center 40% of bbox

        # Tracking parameters (enhanced noise robustness)
        self.declare_parameter('tracking_distance', 1.5)  # Association distance to tracker (m)
        self.declare_parameter('min_measurements', 15)      # Minimum samples for confirmation
        self.declare_parameter('std_threshold', 0.5)  # Max std deviation for confirmation (m)
        self.declare_parameter('outlier_threshold', 1.5)    # Outlier rejection distance (m)
        self.declare_parameter('duplicate_distance', 0.5)   # Duplicate suppression distance (m)

        # Load parameters
        self.rgb_intrinsics = CameraIntrinsics(
            fx=self.get_parameter('rgb_fx').value,
            fy=self.get_parameter('rgb_fy').value,
            cx=self.get_parameter('rgb_cx').value,
            cy=self.get_parameter('rgb_cy').value,
        )
        self.depth_intrinsics = CameraIntrinsics(
            fx=self.get_parameter('depth_fx').value,
            fy=self.get_parameter('depth_fy').value,
            cx=self.get_parameter('depth_cx').value,
            cy=self.get_parameter('depth_cy').value,
        )

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.bbox_scale = self.get_parameter('bbox_scale').value

        self.tracking_distance = self.get_parameter('tracking_distance').value
        self.min_measurements = self.get_parameter('min_measurements').value
        self.std_threshold = self.get_parameter('std_threshold').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.duplicate_distance = self.get_parameter('duplicate_distance').value

        # ========================================
        # Depth -> base_link transform matrix
        # ========================================
        self.depth_to_baselink = np.array([
            [0.0,  0.0,  1.0,  0.095],
            [-1.0, 0.0,  0.0,  0.032],
            [0.0, -1.0,  0.0,  0.172],
            [0.0,  0.0,  0.0,  1.0]
        ])

        # ========================================
        # State variables
        # ========================================
        self.cv_bridge = CvBridge()
        self.current_depth_image: Optional[np.ndarray] = None
        self.current_detections: Optional[Detection2DArray] = None

        # Object trackers: {class_name: [ObjectTracker, ...]}
        self.trackers: Dict[str, List[ObjectTracker]] = {}
        self.confirmed_counts: Dict[str, int] = {}

        # ========================================
        # TF2
        # ========================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ========================================
        # Subscribers
        # ========================================
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo_detections',
            self.detection_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )

        # ========================================
        # Publishers
        # ========================================
        self.object_pub = self.create_publisher(
            Object3DArray, '/test/object_positions', 10)
        self.confirmed_pub = self.create_publisher(
            Object3DArray, '/test/confirmed_objects', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/test/markers', 10)
        self.object_marker_pub = self.create_publisher(
            MarkerArray, '/test/object_markers', 10)

        # ========================================
        # Timer (10 Hz)
        # ========================================
        self.timer = self.create_timer(0.1, self.process_callback)

        self.get_logger().info('Test Localizer initialized')
        self.get_logger().info(f'  - Depth range: {self.min_depth:.1f} - {self.max_depth:.1f} m')
        self.get_logger().info(
            f'  - Min measurements: {self.min_measurements}, '
            f'std threshold: {self.std_threshold:.2f} m'
        )
        self.get_logger().info(f'  - Outlier threshold: {self.outlier_threshold:.2f} m')

    def detection_callback(self, msg: Detection2DArray):
        """Receive YOLO detection results."""
        self.current_detections = msg

    def depth_callback(self, msg: Image):
        """Receive depth image."""
        try:
            self.current_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def process_callback(self):
        """Process detections and publish object positions."""
        if self.current_depth_image is None or self.current_detections is None:
            return

        stamp = self.get_clock().now()
        object_positions = []

        for detection in self.current_detections.detections:
            # 1. RGB -> depth coordinate transform (simple scaling)
            depth_u, depth_v = self.rgb_to_depth_pixel(
                detection.center_x, detection.center_y)

            # 2. Extract depth from ROI
            depth_value = self.extract_depth_at_center(
                depth_u, depth_v,
                detection.width * (self.depth_intrinsics.fx / self.rgb_intrinsics.fx),
                detection.height * (self.depth_intrinsics.fy / self.rgb_intrinsics.fy)
            )

            if depth_value is None:
                continue

            # 3. Camera -> base_link transform
            base_link_pos = self.camera_to_baselink(depth_u, depth_v, depth_value)

            # Create Object3D
            obj = Object3D()
            obj.class_name = detection.class_name
            obj.class_id = detection.class_id
            obj.confidence = detection.confidence
            obj.position.x = float(base_link_pos[0])
            obj.position.y = float(base_link_pos[1])
            obj.position.z = float(base_link_pos[2])
            obj.distance = float(np.linalg.norm(base_link_pos))
            object_positions.append(obj)

            # 4. base_link -> map transform and tracking update
            map_pos = self.baselink_to_map(base_link_pos, stamp)
            if map_pos is not None:
                self.update_tracker(
                    detection.class_name,
                    detection.class_id,
                    map_pos,
                    detection.confidence
                )

        # Publish
        self.publish_object_positions(object_positions, stamp)
        self.publish_object_markers(object_positions, stamp)
        self.publish_confirmed_objects(stamp)
        self.publish_markers(stamp)

    def rgb_to_depth_pixel(self, rgb_x: float, rgb_y: float) -> tuple:
        """Convert RGB pixel coordinates to depth pixel coordinates."""
        # Convert to normalized coordinates and reproject with depth intrinsics
        norm_x = (rgb_x - self.rgb_intrinsics.cx) / self.rgb_intrinsics.fx
        norm_y = (rgb_y - self.rgb_intrinsics.cy) / self.rgb_intrinsics.fy

        depth_x = norm_x * self.depth_intrinsics.fx + self.depth_intrinsics.cx
        depth_y = norm_y * self.depth_intrinsics.fy + self.depth_intrinsics.cy

        return depth_x, depth_y

    def extract_depth_at_center(self, center_x: float, center_y: float,
                                bbox_width: float, bbox_height: float) -> Optional[float]:
        """Extract median depth from the center ROI region."""
        if self.current_depth_image is None:
            return None

        h, w = self.current_depth_image.shape

        # Compute ROI
        roi_w = int(bbox_width * self.bbox_scale)
        roi_h = int(bbox_height * self.bbox_scale)
        roi_w = max(roi_w, 5)
        roi_h = max(roi_h, 5)

        x_start = int(center_x - roi_w / 2)
        y_start = int(center_y - roi_h / 2)
        x_end = x_start + roi_w
        y_end = y_start + roi_h

        # Clamp to image bounds
        x_start = max(0, x_start)
        y_start = max(0, y_start)
        x_end = min(w, x_end)
        y_end = min(h, y_end)

        if x_start >= x_end or y_start >= y_end:
            return None

        # Extract ROI
        roi = self.current_depth_image[y_start:y_end, x_start:x_end]

        # Filter valid depth values
        valid_mask = roi > 0
        if not np.any(valid_mask):
            return None

        valid_depths = roi[valid_mask].astype(np.float64) * self.depth_scale

        # Apply range filtering
        valid_depths = valid_depths[(valid_depths >= self.min_depth) &
                                    (valid_depths <= self.max_depth)]

        if len(valid_depths) < 5:
            return None

        # Use median (robust to outliers)
        return float(np.median(valid_depths))

    def camera_to_baselink(self, u: float, v: float, depth: float) -> np.ndarray:
        """Convert depth camera coordinates to base_link coordinates."""
        # Depth image coordinates -> 3D camera coordinates
        x_cam = (u - self.depth_intrinsics.cx) * depth / self.depth_intrinsics.fx
        y_cam = (v - self.depth_intrinsics.cy) * depth / self.depth_intrinsics.fy
        z_cam = depth

        # Homogeneous coordinates
        cam_point = np.array([x_cam, y_cam, z_cam, 1.0])

        # Transform to base_link
        base_point = self.depth_to_baselink @ cam_point

        return base_point[:3]

    def baselink_to_map(self, base_link_pos: np.ndarray, stamp) -> Optional[np.ndarray]:
        """Convert base_link coordinates to map coordinates (TF2)."""
        try:
            point = PointStamped()
            point.header.frame_id = 'base_link'
            point.header.stamp = stamp.to_msg()
            point.point.x = float(base_link_pos[0])
            point.point.y = float(base_link_pos[1])
            point.point.z = float(base_link_pos[2])

            transformed = self.tf_buffer.transform(
                point, 'map', timeout=Duration(seconds=0.1))

            return np.array([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            ])
        except Exception as e:
            self.get_logger().debug(f'TF transform failed: {e}')
            return None

    def update_tracker(self, class_name: str, class_id: int,
                       map_pos: np.ndarray, confidence: float):
        """Update object tracking (enhanced outlier rejection)."""
        if class_name not in self.trackers:
            self.trackers[class_name] = []

        trackers = self.trackers[class_name]

        # 1. Check if it is near an already confirmed object
        for tracker in trackers:
            if tracker.is_confirmed:
                dist = np.linalg.norm(tracker.confirmed_position[:2] - map_pos[:2])
                if dist < self.duplicate_distance:
                    # Near an already confirmed object -> ignore
                    return

        # 2. Find the nearest unconfirmed tracker
        closest_tracker: Optional[ObjectTracker] = None
        min_dist = float('inf')

        for tracker in trackers:
            if tracker.is_confirmed:
                continue
            if len(tracker.measurements) == 0:
                continue

            # Median position of existing measurements
            avg_pos = _median_pos(tracker.measurements)

            dist = np.linalg.norm(avg_pos[:2] - map_pos[:2])
            if dist < min_dist and dist < self.tracking_distance:
                min_dist = dist
                closest_tracker = tracker

        if closest_tracker is not None:
            # Update existing tracker
            # Outlier check: ignore if too far from previous median
            if len(closest_tracker.measurements) >= 3:
                median_pos = _median_pos(closest_tracker.measurements)
                dist_from_median = np.linalg.norm(median_pos[:2] - map_pos[:2])

                if dist_from_median > self.outlier_threshold:
                    # Classified as outlier -> ignore
                    self.get_logger().debug(
                        f'Outlier rejected for {class_name}: '
                        f'dist={dist_from_median:.2f}m > {self.outlier_threshold:.2f}m'
                    )
                    return

            closest_tracker.measurements.append(map_pos)
            closest_tracker.last_confidence = confidence

            # Confirmation check
            if self.check_confirmation(closest_tracker):
                # Confirm using median position
                confirmed_pos = _median_pos(closest_tracker.measurements)

                # Duplicate check
                is_duplicate = False
                for t in trackers:
                    if t.is_confirmed:
                        d = np.linalg.norm(t.confirmed_position[:2] - confirmed_pos[:2])
                        if d < self.duplicate_distance:
                            is_duplicate = True
                            break

                if not is_duplicate:
                    closest_tracker.confirmed_position = confirmed_pos
                    closest_tracker.is_confirmed = True
                    closest_tracker.confirmed_id = self.get_next_id(class_name)

                    self.get_logger().info(
                        f'Confirmed: {class_name.capitalize()}_{closest_tracker.confirmed_id} '
                        f'at ({confirmed_pos[0]:.2f}, {confirmed_pos[1]:.2f})'
                    )
        else:
            # Create new tracker (only if sufficiently far from existing trackers)
            too_close = False
            for tracker in trackers:
                if tracker.is_confirmed:
                    ref_pos = tracker.confirmed_position
                elif len(tracker.measurements) > 0:
                    ref_pos = _median_pos(tracker.measurements)
                else:
                    continue

                if np.linalg.norm(ref_pos[:2] - map_pos[:2]) < self.tracking_distance:
                    too_close = True
                    break

            if not too_close:
                new_tracker = ObjectTracker(
                    class_name=class_name,
                    class_id=class_id,
                )
                new_tracker.measurements.append(map_pos)
                new_tracker.last_confidence = confidence
                trackers.append(new_tracker)

    def check_confirmation(self, tracker: ObjectTracker) -> bool:
        """Check confirmation criteria."""
        if len(tracker.measurements) < self.min_measurements:
            return False

        # Compute 2D standard deviation (x and y only)
        positions = np.array(tracker.measurements)
        std_x = np.std(positions[:, 0])
        std_y = np.std(positions[:, 1])
        std_2d = np.sqrt(std_x**2 + std_y**2)

        return std_2d < self.std_threshold

    def get_next_id(self, class_name: str) -> int:
        """Return the next ID for each class."""
        if class_name not in self.confirmed_counts:
            self.confirmed_counts[class_name] = 0
        self.confirmed_counts[class_name] += 1
        return self.confirmed_counts[class_name]

    def get_color_for_class(self, class_name: str) -> tuple:
        """Return class-specific color."""
        lower_name = class_name.lower()
        for key, color in self.COLOR_MAP.items():
            if key in lower_name:
                return color
        return self.COLOR_MAP['default']

    def publish_object_positions(self, objects: List[Object3D], stamp):
        """Publish current detections (base_link frame)."""
        msg = Object3DArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = stamp.to_msg()
        msg.objects = objects
        self.object_pub.publish(msg)

    def publish_object_markers(self, objects: List[Object3D], stamp):
        """Publish marker visualization for current detections (base_link frame)."""
        marker_array = MarkerArray()

        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'base_link'
        delete_marker.header.stamp = stamp.to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, obj in enumerate(objects):
            color = self.get_color_for_class(obj.class_name)

            # Cube marker (current detections are shown as cubes)
            cube = Marker()
            cube.header.frame_id = 'base_link'
            cube.header.stamp = stamp.to_msg()
            cube.ns = 'detections'
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = obj.position.x
            cube.pose.position.y = obj.position.y
            cube.pose.position.z = obj.position.z
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.08
            cube.scale.y = 0.08
            cube.scale.z = 0.08
            cube.color.r = color[0]
            cube.color.g = color[1]
            cube.color.b = color[2]
            cube.color.a = 0.7  # Slightly transparent
            cube.lifetime.sec = 0
            cube.lifetime.nanosec = 200000000  # 0.2 s
            marker_array.markers.append(cube)

        self.object_marker_pub.publish(marker_array)

    def publish_confirmed_objects(self, stamp):
        """Publish confirmed objects (map frame)."""
        msg = Object3DArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp.to_msg()

        for class_name, trackers in self.trackers.items():
            for tracker in trackers:
                if not tracker.is_confirmed:
                    continue

                obj = Object3D()
                obj.class_name = f'{class_name}_{tracker.confirmed_id}'
                obj.class_id = tracker.class_id
                obj.confidence = tracker.last_confidence
                obj.position.x = float(tracker.confirmed_position[0])
                obj.position.y = float(tracker.confirmed_position[1])
                obj.position.z = float(tracker.confirmed_position[2])
                obj.distance = float(np.linalg.norm(tracker.confirmed_position))
                msg.objects.append(obj)

        self.confirmed_pub.publish(msg)

    def publish_markers(self, stamp):
        """Publish RViz visualization markers."""
        marker_array = MarkerArray()

        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = stamp.to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0

        for class_name, trackers in self.trackers.items():
            color = self.get_color_for_class(class_name)

            for tracker in trackers:
                if not tracker.is_confirmed:
                    continue

                pos = tracker.confirmed_position

                # Sphere marker
                sphere = Marker()
                sphere.header.frame_id = 'map'
                sphere.header.stamp = stamp.to_msg()
                sphere.ns = 'signs'
                sphere.id = marker_id
                marker_id += 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = float(pos[0])
                sphere.pose.position.y = float(pos[1])
                sphere.pose.position.z = float(pos[2])
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.1
                sphere.scale.y = 0.1
                sphere.scale.z = 0.1
                sphere.color.r = color[0]
                sphere.color.g = color[1]
                sphere.color.b = color[2]
                sphere.color.a = color[3]
                marker_array.markers.append(sphere)

                # Text marker (e.g., Stop_1)
                text = Marker()
                text.header.frame_id = 'map'
                text.header.stamp = stamp.to_msg()
                text.ns = 'labels'
                text.id = marker_id
                marker_id += 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = float(pos[0])
                text.pose.position.y = float(pos[1])
                text.pose.position.z = float(pos[2]) + 0.15
                text.pose.orientation.w = 1.0
                text.scale.z = 0.08
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = f'{class_name.capitalize()}_{tracker.confirmed_id}'
                marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TestLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
