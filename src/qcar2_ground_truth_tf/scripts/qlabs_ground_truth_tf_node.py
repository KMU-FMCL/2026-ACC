#!/usr/bin/env python3

import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll-pitch-yaw [rad] to quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class QLabsGroundTruthTFNode(Node):
    def __init__(self) -> None:
        super().__init__("qlabs_ground_truth_tf_node")

        self.declare_parameter("qlabs_host", "localhost")
        self.declare_parameter("actor_number", 0)
        self.declare_parameter("auto_discover_actor", True)
        self.declare_parameter("actor_search_min", 0)
        self.declare_parameter("actor_search_max", 255)
        self.declare_parameter("actor_search_period_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("apply_qlabs_to_map_transform", True)
        self.declare_parameter("qlabs_to_map_scale", 0.975)
        self.declare_parameter("qlabs_to_map_theta", 0.60)
        self.declare_parameter("qlabs_to_map_tx", 0.40687896)
        self.declare_parameter("qlabs_to_map_ty", 1.39737787)
        self.declare_parameter("qlabs_to_map_tz", 0.0)
        self.declare_parameter("publish_pose_topic", True)
        self.declare_parameter("publish_path_topic", True)
        self.declare_parameter("pose_topic", "/ground_truth/pose")
        self.declare_parameter("path_topic", "/ground_truth/path")
        self.declare_parameter("max_path_points", 5000)
        self.declare_parameter("spawn_qcar", False)
        self.declare_parameter("spawn_location", [0.0, 0.0, 0.0])
        self.declare_parameter("spawn_rotation", [0.0, 0.0, 0.0])

        self.qlabs_host = str(self.get_parameter("qlabs_host").value)
        self.actor_number = int(self.get_parameter("actor_number").value)
        self.auto_discover_actor = bool(self.get_parameter("auto_discover_actor").value)
        self.actor_search_min = int(self.get_parameter("actor_search_min").value)
        self.actor_search_max = int(self.get_parameter("actor_search_max").value)
        self.actor_search_period_sec = float(self.get_parameter("actor_search_period_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.apply_qlabs_to_map_transform = bool(self.get_parameter("apply_qlabs_to_map_transform").value)
        self.qlabs_to_map_scale = float(self.get_parameter("qlabs_to_map_scale").value)
        self.qlabs_to_map_theta = float(self.get_parameter("qlabs_to_map_theta").value)
        self.qlabs_to_map_tx = float(self.get_parameter("qlabs_to_map_tx").value)
        self.qlabs_to_map_ty = float(self.get_parameter("qlabs_to_map_ty").value)
        self.qlabs_to_map_tz = float(self.get_parameter("qlabs_to_map_tz").value)
        self.publish_pose_topic = bool(self.get_parameter("publish_pose_topic").value)
        self.publish_path_topic = bool(self.get_parameter("publish_path_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.spawn_qcar = bool(self.get_parameter("spawn_qcar").value)
        self.spawn_location = list(self.get_parameter("spawn_location").value)
        self.spawn_rotation = list(self.get_parameter("spawn_rotation").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.qlabs: Optional[QuanserInteractiveLabs] = None
        self.hqcar: Optional[QLabsQCar2] = None
        self.connected = False
        self.last_actor_search_sec = 0.0
        self.last_read_fail_log_sec = 0.0

        self.publish_static_map_to_odom()
        self.ensure_connection()

        period = max(1e-3, 1.0 / max(1e-3, self.publish_rate_hz))
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"QLabs GT TF node started: {self.map_frame}->{self.odom_frame}->{self.base_frame} @ "
            f"{self.publish_rate_hz:.1f} Hz"
        )
        if self.apply_qlabs_to_map_transform:
            self.get_logger().info(
                "Applying QLabs->map transform: "
                f"scale={self.qlabs_to_map_scale}, theta={self.qlabs_to_map_theta}, "
                f"tx={self.qlabs_to_map_tx}, ty={self.qlabs_to_map_ty}, tz={self.qlabs_to_map_tz}"
            )
        else:
            self.get_logger().info("QLabs->map transform disabled (identity).")

    def publish_static_map_to_odom(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static TF: {self.map_frame} -> {self.odom_frame} (identity)")

    def ensure_connection(self) -> None:
        try:
            if self.qlabs is None:
                self.qlabs = QuanserInteractiveLabs()
            if not self.qlabs.open(self.qlabs_host):
                self.connected = False
                self.get_logger().error(f"Failed to connect to QLabs host: {self.qlabs_host}")
                return

            self.hqcar = QLabsQCar2(self.qlabs)
            if self.spawn_qcar:
                if len(self.spawn_location) != 3 or len(self.spawn_rotation) != 3:
                    self.get_logger().error("spawn_location and spawn_rotation must be length 3.")
                else:
                    ok = self.hqcar.spawn_id(
                        actorNumber=self.actor_number,
                        location=[float(v) for v in self.spawn_location],
                        rotation=[float(v) for v in self.spawn_rotation],
                        waitForConfirmation=True,
                    )
                    if not ok:
                        self.get_logger().warn("spawn_qcar=true but spawn_id failed; continuing with actor lookup.")

            self.connected = True
            self.get_logger().info("Connected to QLabs.")
        except Exception as exc:
            self.connected = False
            self.get_logger().error(f"QLabs setup exception: {exc}")

    def _try_get_world_pose(self, actor_number: Optional[int]) -> Optional[Tuple[List[float], List[float]]]:
        if self.hqcar is None:
            return None
        try:
            success = False
            location = None
            rotation = None

            # Preferred: explicit actor number.
            if actor_number is not None:
                try:
                    success, location, rotation, _scale = self.hqcar.get_world_transform(actorNumber=actor_number)
                except TypeError:
                    try:
                        success, location, rotation, _scale = self.hqcar.get_world_transform(actor_number)
                    except TypeError:
                        # Some QVL variants use internal actor index only.
                        if hasattr(self.hqcar, "actorNumber"):
                            setattr(self.hqcar, "actorNumber", actor_number)
                        if hasattr(self.hqcar, "actor_number"):
                            setattr(self.hqcar, "actor_number", actor_number)
                        success, location, rotation, _scale = self.hqcar.get_world_transform()
            else:
                success, location, rotation, _scale = self.hqcar.get_world_transform()

            if not success:
                return None
            if len(location) != 3 or len(rotation) != 3:
                return None
            return location, rotation
        except Exception as exc:
            self.get_logger().warn(f"get_world_transform exception: {exc}")
            return None

    def _discover_actor_number(self) -> bool:
        if self.hqcar is None:
            return False
        now_sec = time.monotonic()
        if (now_sec - self.last_actor_search_sec) < max(0.1, self.actor_search_period_sec):
            return False
        self.last_actor_search_sec = now_sec

        start = min(self.actor_search_min, self.actor_search_max)
        end = max(self.actor_search_min, self.actor_search_max)
        for candidate in range(start, end + 1):
            pose = self._try_get_world_pose(candidate)
            if pose is not None:
                if candidate != self.actor_number:
                    self.get_logger().info(f"Discovered QCar2 actor_number={candidate}")
                self.actor_number = candidate
                return True
        return False

    def _apply_map_transform(
        self, location: List[float], rotation: List[float]
    ) -> Tuple[List[float], List[float]]:
        if not self.apply_qlabs_to_map_transform:
            return location, rotation

        x, y, z = float(location[0]), float(location[1]), float(location[2])
        roll, pitch, yaw = float(rotation[0]), float(rotation[1]), float(rotation[2])

        xs = x * self.qlabs_to_map_scale
        ys = y * self.qlabs_to_map_scale
        zs = z * self.qlabs_to_map_scale

        c = math.cos(self.qlabs_to_map_theta)
        s = math.sin(self.qlabs_to_map_theta)
        xm = c * xs - s * ys + self.qlabs_to_map_tx
        ym = s * xs + c * ys + self.qlabs_to_map_ty
        zm = zs + self.qlabs_to_map_tz

        yaw_m = normalize_angle(yaw + self.qlabs_to_map_theta)
        return [xm, ym, zm], [roll, pitch, yaw_m]

    def timer_callback(self) -> None:
        if not self.connected:
            self.ensure_connection()
            return

        pose = self._try_get_world_pose(self.actor_number)
        if pose is None and self.auto_discover_actor:
            if self._discover_actor_number():
                pose = self._try_get_world_pose(self.actor_number)

        if pose is None:
            now_sec = time.monotonic()
            if (now_sec - self.last_read_fail_log_sec) > 2.0:
                self.last_read_fail_log_sec = now_sec
                self.get_logger().warn(
                    "Ground-truth read failed (QLabs connected). "
                    f"Current actor_number={self.actor_number}. "
                    "Check actor id or enable auto_discover_actor."
                )
            return

        location, rotation = pose
        location, rotation = self._apply_map_transform(location, rotation)
        roll, pitch, yaw = float(rotation[0]), float(rotation[1]), float(rotation[2])
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        now_msg = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now_msg
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = float(location[0])
        tf_msg.transform.translation.y = float(location[1])
        tf_msg.transform.translation.z = float(location[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        if self.publish_pose_topic:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now_msg
            pose_msg.header.frame_id = self.map_frame
            pose_msg.pose.position.x = float(location[0])
            pose_msg.pose.position.y = float(location[1])
            pose_msg.pose.position.z = float(location[2])
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self.pose_pub.publish(pose_msg)

            if self.publish_path_topic:
                self.path_msg.header.stamp = now_msg
                self.path_msg.poses.append(pose_msg)
                if len(self.path_msg.poses) > self.max_path_points:
                    overflow = len(self.path_msg.poses) - self.max_path_points
                    del self.path_msg.poses[0:overflow]
                self.path_pub.publish(self.path_msg)


def main() -> None:
    rclpy.init()
    node = QLabsGroundTruthTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
