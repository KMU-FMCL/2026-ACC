#!/usr/bin/env python3

import importlib
import builtins
import math
import os
import shutil
import sys
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
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


class QCarGPSTFNode(Node):
    def __init__(self) -> None:
        super().__init__("qcar_gps_tf_node")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("read_retry_period_sec", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("gps_initial_x", 0.0)
        self.declare_parameter("gps_initial_y", 0.0)
        self.declare_parameter("gps_initial_yaw", 0.0)
        self.declare_parameter("gps_calibrate", False)
        self.declare_parameter("pal_python_path", "/home/Quanser/0_libraries/python")
        self.declare_parameter("pal_runtime_path", "/tmp/qcar_pal_runtime")
        self.declare_parameter("virtual_qcar_type", 2)

        self.declare_parameter("apply_gps_to_map_transform", True)
        self.declare_parameter("gps_to_map_scale", 0.975)
        self.declare_parameter("gps_to_map_theta", 0.717702849743)
        self.declare_parameter("gps_to_map_tx", 0.352846958100)
        self.declare_parameter("gps_to_map_ty", 1.382286458114)
        self.declare_parameter("gps_to_map_tz", 0.0)

        self.declare_parameter("publish_pose_topic", True)
        self.declare_parameter("publish_odom_topic", True)
        self.declare_parameter("publish_path_topic", True)
        self.declare_parameter("pose_topic", "/gps_pose")
        self.declare_parameter("odom_topic", "/gps_odom")
        self.declare_parameter("path_topic", "/gps_path")
        self.declare_parameter("max_path_points", 5000)
        self.declare_parameter("pose_cov_xy", 0.01)
        self.declare_parameter("pose_cov_yaw", 0.02)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.read_retry_period_sec = float(self.get_parameter("read_retry_period_sec").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.gps_initial_x = float(self.get_parameter("gps_initial_x").value)
        self.gps_initial_y = float(self.get_parameter("gps_initial_y").value)
        self.gps_initial_yaw = float(self.get_parameter("gps_initial_yaw").value)
        self.gps_calibrate = bool(self.get_parameter("gps_calibrate").value)
        self.pal_python_path = str(self.get_parameter("pal_python_path").value)
        self.pal_runtime_path = str(self.get_parameter("pal_runtime_path").value)
        self.virtual_qcar_type = str(int(self.get_parameter("virtual_qcar_type").value))

        self.apply_gps_to_map_transform = bool(self.get_parameter("apply_gps_to_map_transform").value)
        self.gps_to_map_scale = float(self.get_parameter("gps_to_map_scale").value)
        self.gps_to_map_theta = float(self.get_parameter("gps_to_map_theta").value)
        self.gps_to_map_tx = float(self.get_parameter("gps_to_map_tx").value)
        self.gps_to_map_ty = float(self.get_parameter("gps_to_map_ty").value)
        self.gps_to_map_tz = float(self.get_parameter("gps_to_map_tz").value)

        self.publish_pose_topic = bool(self.get_parameter("publish_pose_topic").value)
        self.publish_odom_topic = bool(self.get_parameter("publish_odom_topic").value)
        self.publish_path_topic = bool(self.get_parameter("publish_path_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.pose_cov_xy = float(self.get_parameter("pose_cov_xy").value)
        self.pose_cov_yaw = float(self.get_parameter("pose_cov_yaw").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 20)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.gps = None
        self.qcar_gps_cls = None
        self.connected = False
        self.last_retry_sec = 0.0
        self.last_read_fail_log_sec = 0.0

        self._publish_static_map_to_odom()
        self._connect_gps()

        period = max(1e-3, 1.0 / max(1e-3, self.publish_rate_hz))
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"QCarGPS TF node started: {self.map_frame}->{self.odom_frame}->{self.base_frame} @ "
            f"{self.publish_rate_hz:.1f} Hz"
        )
        if self.apply_gps_to_map_transform:
            self.get_logger().info(
                "Applying GPS->map transform: "
                f"scale={self.gps_to_map_scale}, theta={self.gps_to_map_theta}, "
                f"tx={self.gps_to_map_tx}, ty={self.gps_to_map_ty}, tz={self.gps_to_map_tz}"
            )

    def _publish_static_map_to_odom(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static TF: {self.map_frame} -> {self.odom_frame} (identity)")

    def _close_gps(self) -> None:
        if self.gps is None:
            return
        try:
            if hasattr(self.gps, "__exit__"):
                self.gps.__exit__(None, None, None)
        except Exception:
            pass
        self.gps = None
        self.connected = False

    def _connect_gps(self) -> None:
        qcar_gps_cls = self._load_qcar_gps_cls()
        if qcar_gps_cls is None:
            self.connected = False
            return
        try:
            self._close_gps()
            self.gps = qcar_gps_cls(
                initialPose=[self.gps_initial_x, self.gps_initial_y, self.gps_initial_yaw],
                calibrate=self.gps_calibrate,
            )
            if hasattr(self.gps, "__enter__"):
                self.gps.__enter__()
            self.connected = True
            self.get_logger().info("Connected to QCarGPS.")
        except Exception as exc:
            self.connected = False
            self.gps = None
            self.get_logger().error(f"QCarGPS connection failed: {exc}")

    def _load_qcar_gps_cls(self):
        if self.qcar_gps_cls is not None:
            return self.qcar_gps_cls

        candidates = []
        if self.pal_python_path:
            candidates.append(self.pal_python_path)
        env_pal = os.environ.get("PAL_PYTHON_PATH", "")
        if env_pal:
            candidates.append(env_pal)
        env_py = os.environ.get("PYTHONPATH", "")
        if env_py:
            candidates.extend([p for p in env_py.split(":") if p])
        # Also inspect currently active python paths.
        candidates.extend(list(sys.path))
        candidates.extend(
            [
                "/home/Quanser/0_libraries/python",
                "/workspaces/isaac_ros-dev/docker/0_libraries/python",
                "/workspaces/isaac_ros-dev/docker/development_docker/quanser_dev_docker_files/0_libraries/python",
                "/home/jsm/Documents/ACC_Development/docker/0_libraries/python",
            ]
        )

        source_pal_root = self._find_pal_root(candidates)
        if source_pal_root is None:
            self.get_logger().error(
                "Could not find PAL root containing pal/products/qcar.py. "
                f"candidates={candidates}"
            )
            return None

        runtime_pal_root = self._prepare_writable_pal_root(source_pal_root)
        if runtime_pal_root is not None and runtime_pal_root not in sys.path:
            sys.path.insert(0, runtime_pal_root)
        if source_pal_root not in sys.path:
            sys.path.append(source_pal_root)

        # PAL qcar.py calls os.getlogin() and may prompt for virtual car type
        # at import time. In ROS/docker non-interactive environments this can
        # fail or block, so we provide safe defaults only during import.
        original_getlogin = os.getlogin
        original_input = builtins.input

        def _safe_getlogin():
            try:
                return original_getlogin()
            except Exception:
                return os.environ.get("USER", "admin")

        def _safe_input(prompt=""):
            _ = prompt
            return self.virtual_qcar_type

        try:
            # Clear possibly failed imports before retry.
            sys.modules.pop("pal.products.qcar", None)
            sys.modules.pop("pal.products.qcar_config", None)
            os.getlogin = _safe_getlogin  # type: ignore[assignment]
            builtins.input = _safe_input  # type: ignore[assignment]
            qcar_module = importlib.import_module("pal.products.qcar")
            self.qcar_gps_cls = getattr(qcar_module, "QCarGPS", None)
        except Exception as exc:
            self.qcar_gps_cls = None
            self.get_logger().error(
                "Failed to import pal.products.qcar.QCarGPS. "
                f"pal_python_path='{self.pal_python_path}', PAL_PYTHON_PATH='{env_pal}', "
                f"pal_runtime_path='{self.pal_runtime_path}', virtual_qcar_type='{self.virtual_qcar_type}', "
                f"error={exc}"
            )
            return None
        finally:
            os.getlogin = original_getlogin  # type: ignore[assignment]
            builtins.input = original_input  # type: ignore[assignment]

        if self.qcar_gps_cls is None:
            self.get_logger().error("Imported pal.products.qcar but QCarGPS symbol was not found.")
            return None
        return self.qcar_gps_cls

    def _find_pal_root(self, candidates):
        for path in candidates:
            if not path:
                continue
            if os.path.isfile(os.path.join(path, "pal", "products", "qcar.py")):
                return path
        return None

    def _prepare_writable_pal_root(self, source_root):
        # qcar.py writes qcar_config.json into its own package directory at import time.
        # Use a writable runtime mirror under /tmp so launch can run without extra options.
        runtime_root = self.pal_runtime_path.strip()
        if not runtime_root:
            return None
        try:
            src_pal = os.path.join(source_root, "pal")
            dst_pal = os.path.join(runtime_root, "pal")
            if not os.path.isdir(src_pal):
                return None
            os.makedirs(runtime_root, exist_ok=True)
            shutil.copytree(src_pal, dst_pal, dirs_exist_ok=True)
            return runtime_root
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to prepare writable PAL runtime path '{runtime_root}' from '{source_root}': {exc}"
            )
            return None

    def _apply_map_transform(self, x: float, y: float, z: float, yaw: float) -> Tuple[float, float, float, float]:
        if not self.apply_gps_to_map_transform:
            return x, y, z, yaw

        xs = x * self.gps_to_map_scale
        ys = y * self.gps_to_map_scale
        zs = z * self.gps_to_map_scale
        c = math.cos(self.gps_to_map_theta)
        s = math.sin(self.gps_to_map_theta)
        xm = c * xs - s * ys + self.gps_to_map_tx
        ym = s * xs + c * ys + self.gps_to_map_ty
        zm = zs + self.gps_to_map_tz
        yaw_m = normalize_angle(yaw + self.gps_to_map_theta)
        return xm, ym, zm, yaw_m

    def _timer_callback(self) -> None:
        now_sec = time.monotonic()
        if not self.connected:
            if (now_sec - self.last_retry_sec) >= max(0.1, self.read_retry_period_sec):
                self.last_retry_sec = now_sec
                self._connect_gps()
            return

        try:
            assert self.gps is not None
            has_new = self.gps.readGPS()
            if not has_new:
                return

            pos = list(self.gps.position)
            ori = list(self.gps.orientation)
            if len(pos) < 2 or len(ori) < 3:
                if (now_sec - self.last_read_fail_log_sec) > 2.0:
                    self.last_read_fail_log_sec = now_sec
                    self.get_logger().warn("QCarGPS data shape invalid. Expected position>=2, orientation>=3.")
                return

            x = float(pos[0])
            y = float(pos[1])
            z = float(pos[2]) if len(pos) >= 3 else 0.0
            roll = float(ori[0]) if len(ori) >= 1 else 0.0
            pitch = float(ori[1]) if len(ori) >= 2 else 0.0
            yaw = float(ori[2])

            x, y, z, yaw = self._apply_map_transform(x, y, z, yaw)
            qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
            now_msg = self.get_clock().now().to_msg()

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            if self.publish_pose_topic:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now_msg
                pose_msg.header.frame_id = self.map_frame
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
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

            if self.publish_odom_topic:
                odom_msg = Odometry()
                odom_msg.header.stamp = now_msg
                odom_msg.header.frame_id = self.odom_frame
                odom_msg.child_frame_id = self.base_frame
                odom_msg.pose.pose.position.x = x
                odom_msg.pose.pose.position.y = y
                odom_msg.pose.pose.position.z = z
                odom_msg.pose.pose.orientation.x = qx
                odom_msg.pose.pose.orientation.y = qy
                odom_msg.pose.pose.orientation.z = qz
                odom_msg.pose.pose.orientation.w = qw
                odom_msg.pose.covariance[0] = self.pose_cov_xy
                odom_msg.pose.covariance[7] = self.pose_cov_xy
                odom_msg.pose.covariance[35] = self.pose_cov_yaw
                self.odom_pub.publish(odom_msg)

        except Exception as exc:
            if (now_sec - self.last_read_fail_log_sec) > 2.0:
                self.last_read_fail_log_sec = now_sec
                self.get_logger().warn(f"QCarGPS read failed: {exc}. Reconnecting.")
            self._close_gps()

    def destroy_node(self) -> bool:
        self._close_gps()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = QCarGPSTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
