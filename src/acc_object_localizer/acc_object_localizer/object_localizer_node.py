#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

from acc_msgs.msg import Detection2DArray, Object3D, Object3DArray
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformException

import message_filters


class ObjectLocalizerNode(Node):
    def __init__(self):
        super().__init__('object_localizer_node')

        # ========== Parameters (all values loaded from YAML) ==========
        self.declare_parameter('debug', True)

        # Camera intrinsics
        self.declare_parameter('rgb_intrinsic.fx', 0.0)
        self.declare_parameter('rgb_intrinsic.fy', 0.0)
        self.declare_parameter('rgb_intrinsic.cx', 0.0)
        self.declare_parameter('rgb_intrinsic.cy', 0.0)

        self.declare_parameter('depth_intrinsic.fx', 0.0)
        self.declare_parameter('depth_intrinsic.fy', 0.0)
        self.declare_parameter('depth_intrinsic.cx', 0.0)
        self.declare_parameter('depth_intrinsic.cy', 0.0)

        # RGB to Depth extrinsic (4x4 transformation matrix)
        self.declare_parameter('rgb_to_depth_extrinsic', [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ])

        # RealSense to body extrinsic (4x4 transformation matrix)
        self.declare_parameter('realsense_to_body_extrinsic', [
            0.0, 0.0, 1.0, 0.095,
            -1.0, 0.0, 0.0, 0.032,
            0.0, -1.0, 0.0, 0.172,
            0.0, 0.0, 0.0, 1.0
        ])

        # Depth filtering
        self.declare_parameter('depth_scale', 1.0)  # Depth scale factor (virtual-environment compensation)
        self.declare_parameter('depth_min', 0.1)  # meters
        self.declare_parameter('depth_max', 5.0)  # meters
        self.declare_parameter('use_median_filter', False)  # Whether to use median filter
        self.declare_parameter('depth_median_kernel_size', 5)  # pixels (median filter)

        # Marker settings
        self.declare_parameter('marker_scale', 0.1)  # marker size (meters)
        self.declare_parameter('marker_lifetime_sec', 1.0)

        # Topics
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('use_aligned_depth', False)  # Whether aligned depth is enabled

        # Frame IDs
        self.declare_parameter('base_frame', 'base_link')

        # Load parameters
        self.debug = self.get_parameter('debug').value

        # RGB intrinsic matrix
        self.rgb_K = np.array([
            [self.get_parameter('rgb_intrinsic.fx').value, 0.0, self.get_parameter('rgb_intrinsic.cx').value],
            [0.0, self.get_parameter('rgb_intrinsic.fy').value, self.get_parameter('rgb_intrinsic.cy').value],
            [0.0, 0.0, 1.0]
        ])

        # Depth intrinsic matrix
        self.depth_K = np.array([
            [self.get_parameter('depth_intrinsic.fx').value, 0.0, self.get_parameter('depth_intrinsic.cx').value],
            [0.0, self.get_parameter('depth_intrinsic.fy').value, self.get_parameter('depth_intrinsic.cy').value],
            [0.0, 0.0, 1.0]
        ])

        # RGB to Depth extrinsic (4x4)
        rgb_to_depth_flat = self.get_parameter('rgb_to_depth_extrinsic').value
        self.T_rgb_to_depth = np.array(rgb_to_depth_flat).reshape(4, 4)

        # RealSense to body extrinsic (4x4)
        realsense_to_body_flat = self.get_parameter('realsense_to_body_extrinsic').value
        self.T_cam_to_body = np.array(realsense_to_body_flat).reshape(4, 4)

        # Depth filtering params
        self.depth_scale = self.get_parameter('depth_scale').value
        self.depth_min = self.get_parameter('depth_min').value
        self.depth_max = self.get_parameter('depth_max').value
        self.use_median_filter = self.get_parameter('use_median_filter').value
        self.depth_median_kernel = self.get_parameter('depth_median_kernel_size').value

        # Marker params
        self.marker_scale = self.get_parameter('marker_scale').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value

        # Topics
        self.depth_topic = self.get_parameter('depth_topic').value
        self.use_aligned_depth = self.get_parameter('use_aligned_depth').value

        # Frame IDs
        self.base_frame = self.get_parameter('base_frame').value

        # ========== CV Bridge ==========
        self.bridge = CvBridge()

        # ========== Subscribers (synchronized with Message Filter) ==========
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.detection_sub = message_filters.Subscriber(
            self, Detection2DArray, '/yolo_detections', qos_profile=qos_profile
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile
        )

        # ApproximateTimeSynchronizer (allows +/-100ms)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.depth_sub],
            queue_size=10,
            slop=0.1  # 100ms
        )
        self.sync.registerCallback(self.sync_callback)

        # ========== Publishers ==========
        self.object_pub = self.create_publisher(Object3DArray, '/object_localizer/objects', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_localizer/markers', 10)

        self.get_logger().info('Object Localizer Node initialized')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info(f'Aligned depth mode: {self.use_aligned_depth}')
        self.get_logger().info(f'RGB Intrinsic:\n{self.rgb_K}')
        self.get_logger().info(f'Depth Intrinsic:\n{self.depth_K}')
        if not self.use_aligned_depth:
            self.get_logger().info(f'RGB→Depth Extrinsic:\n{self.T_rgb_to_depth}')
        self.get_logger().info(f'Camera→Body Extrinsic:\n{self.T_cam_to_body}')
        self.get_logger().info(f'Depth scale: {self.depth_scale}x (raw depth will be multiplied)')
        filter_mode = f'Median filter (kernel={self.depth_median_kernel})' if self.use_median_filter else 'Raw depth (no filter)'
        self.get_logger().info(f'Depth mode: {filter_mode}')

    def sync_callback(self, detections_msg, depth_msg):
        """Called when detection and depth images are synchronized."""
        try:
            # Convert depth image to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            if self.debug:
                h, w = depth_image.shape
                self.get_logger().info(
                    f'Depth image: {w}×{h}, dtype={depth_image.dtype}, '
                    f'min={np.min(depth_image):.2f}, max={np.max(depth_image):.2f}'
                )

            # Convert to meters if the depth image is in millimeters
            if depth_image.dtype == np.uint16:
                depth_image = depth_image.astype(np.float32) / 1000.0  # mm → m

            # Apply depth scaling (virtual-environment compensation)
            depth_image = depth_image * self.depth_scale

            if self.debug:
                self.get_logger().info(
                    f'After scale ({self.depth_scale}x): min={np.min(depth_image):.2f}m, max={np.max(depth_image):.2f}m'
                )

            # Process detections
            objects = []
            markers = MarkerArray()

            for idx, detection in enumerate(detections_msg.detections):
                obj = self.process_detection(detection, depth_image, idx)
                if obj is not None:
                    objects.append(obj)
                    marker = self.create_marker(obj, idx, detections_msg.header.stamp)
                    markers.markers.append(marker)

            # Publish results
            if len(objects) > 0:
                obj_array_msg = Object3DArray()
                obj_array_msg.header.stamp = detections_msg.header.stamp
                obj_array_msg.header.frame_id = self.base_frame
                obj_array_msg.objects = objects
                self.object_pub.publish(obj_array_msg)

                self.marker_pub.publish(markers)

                if self.debug:
                    self.get_logger().info(f'Localized {len(objects)} objects in base_link frame')

        except Exception as e:
            self.get_logger().error(f'Error in sync_callback: {e}')

    def process_detection(self, detection, depth_image, idx):
        """Process a single detection and create an Object3D in base_link coordinates."""
        try:
            # 1. RGB pixel coordinates
            u_rgb = detection.center_x
            v_rgb = detection.center_y

            if self.debug:
                self.get_logger().info(f'[{detection.class_name}] RGB pixel: ({u_rgb:.1f}, {v_rgb:.1f})')

            if self.use_aligned_depth:
                # Aligned depth: use RGB pixel directly
                u_depth, v_depth = u_rgb, v_rgb

                if self.debug:
                    self.get_logger().info(f'[{detection.class_name}] Using aligned depth (same pixel)')
            else:
                # Non-aligned depth: RGB -> Depth transform is required
                # 2. RGB pixel -> RGB camera 3D ray (temporary depth=1m)
                rgb_ray = self.pixel_to_camera_ray(u_rgb, v_rgb, self.rgb_K)

                # 3. RGB 3D -> Depth camera 3D (apply extrinsic)
                depth_ray_homogeneous = self.T_rgb_to_depth @ np.append(rgb_ray, 1.0)
                depth_ray = depth_ray_homogeneous[:3] / depth_ray_homogeneous[3]

                # 4. Depth 3D -> Depth pixel
                u_depth, v_depth = self.camera_to_pixel(depth_ray, self.depth_K)

                if self.debug:
                    h, w = depth_image.shape
                    in_bounds = (0 <= int(u_depth) < w) and (0 <= int(v_depth) < h)
                    self.get_logger().info(
                        f'[{detection.class_name}] Depth pixel: ({u_depth:.1f}, {v_depth:.1f}), '
                        f'Image size: ({w}, {h}), In bounds: {in_bounds}'
                    )

            # 5. Query actual depth value from depth image
            depth_value = self.get_depth_at_pixel(depth_image, u_depth, v_depth)

            if self.debug:
                if depth_value is not None:
                    self.get_logger().info(f'[{detection.class_name}] Raw depth value: {depth_value:.3f}m')
                else:
                    self.get_logger().warn(f'[{detection.class_name}] Depth value is None (out of bounds or no data)')

            if depth_value is None or depth_value < self.depth_min or depth_value > self.depth_max:
                if self.debug:
                    self.get_logger().warn(
                        f'[{detection.class_name}] Invalid depth: {depth_value} '
                        f'(valid range: {self.depth_min}m ~ {self.depth_max}m)'
                    )
                return None

            # 6. Depth pixel + actual depth -> camera 3D coordinate
            if self.use_aligned_depth:
                # Aligned depth: use RGB intrinsics
                point_cam = self.pixel_to_camera_3d(u_depth, v_depth, depth_value, self.rgb_K)
            else:
                # Non-aligned depth: use depth intrinsics
                point_cam = self.pixel_to_camera_3d(u_depth, v_depth, depth_value, self.depth_K)

            # 7. Camera -> base_link transform (realsense_to_body)
            point_cam_homogeneous = np.append(point_cam, 1.0)
            point_base_link = self.T_cam_to_body @ point_cam_homogeneous
            point_base_link = point_base_link[:3] / point_base_link[3]

            # 8. Build Object3D message
            obj = Object3D()
            obj.class_name = detection.class_name
            obj.class_id = detection.class_id
            obj.confidence = detection.confidence
            obj.position.x = float(point_base_link[0])
            obj.position.y = float(point_base_link[1])
            obj.position.z = float(point_base_link[2])

            if self.debug:
                self.get_logger().info(
                    f'{detection.class_name}: RGB({u_rgb:.1f},{v_rgb:.1f}) → '
                    f'Depth({u_depth:.1f},{v_depth:.1f}) → '
                    f'depth={depth_value:.2f}m → '
                    f'base_link=({point_base_link[0]:.2f}, {point_base_link[1]:.2f}, {point_base_link[2]:.2f})'
                )

            return obj

        except Exception as e:
            self.get_logger().error(f'Error processing detection {idx}: {e}')
            return None

    def pixel_to_camera_ray(self, u, v, K):
        """Pixel coordinate -> camera-coordinate ray (depth=1)."""
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        x = (u - cx) / fx
        y = (v - cy) / fy
        z = 1.0
        return np.array([x, y, z])

    def camera_to_pixel(self, point_3d, K):
        """Camera 3D coordinate -> pixel coordinate."""
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        u = (point_3d[0] / point_3d[2]) * fx + cx
        v = (point_3d[1] / point_3d[2]) * fy + cy
        return u, v

    def pixel_to_camera_3d(self, u, v, depth, K):
        """Pixel coordinate + depth -> camera 3D coordinate."""
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return np.array([x, y, z])

    def get_depth_at_pixel(self, depth_image, u, v):
        """Get depth value at pixel (u, v) from depth image (median filter optional)."""
        h, w = depth_image.shape
        u_int = int(round(u))
        v_int = int(round(v))

        # Bounds check
        if u_int < 0 or u_int >= w or v_int < 0 or v_int >= h:
            if self.debug:
                self.get_logger().warn(f'Pixel ({u_int}, {v_int}) out of bounds (image size: {w}×{h})')
            return None

        if self.use_median_filter:
            # Use median filter (noise reduction)
            kernel_size = self.depth_median_kernel
            half_k = kernel_size // 2

            v_min = max(0, v_int - half_k)
            v_max = min(h, v_int + half_k + 1)
            u_min = max(0, u_int - half_k)
            u_max = min(w, u_int + half_k + 1)

            depth_patch = depth_image[v_min:v_max, u_min:u_max]
            valid_depths = depth_patch[(depth_patch > self.depth_min) & (depth_patch < self.depth_max)]

            if len(valid_depths) == 0:
                return None

            return float(np.median(valid_depths))
        else:
            # Use raw depth value directly
            depth_value = float(depth_image[v_int, u_int])

            if self.debug:
                self.get_logger().info(f'  → Raw depth at ({u_int}, {v_int}): {depth_value:.3f}m')

            # Only perform validity check
            if depth_value < self.depth_min or depth_value > self.depth_max:
                if self.debug:
                    self.get_logger().warn(f'  → Depth {depth_value:.3f}m out of valid range [{self.depth_min}, {self.depth_max}]')
                return None

            return depth_value

    def create_marker(self, obj, idx, stamp):
        """Create an RViz marker from Object3D (color by class)."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'object_localizer'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = obj.position.x
        marker.pose.position.y = obj.position.y
        marker.pose.position.z = obj.position.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale

        # Color mapping (stop=red, yield=yellow, round=blue)
        class_lower = obj.class_name.lower()
        if 'stop' in class_lower:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif 'yield' in class_lower:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif 'round' in class_lower:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            # Default: white
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

        marker.color.a = 1.0
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e8)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
