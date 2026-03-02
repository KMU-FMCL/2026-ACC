import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("qcar2_ground_truth_tf")
    default_rviz_config = os.path.join(pkg_share, "rviz", "localization.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_qcar2_virtual_nodes = LaunchConfiguration("use_qcar2_virtual_nodes")
    use_qcar2_lidar_tf = LaunchConfiguration("use_qcar2_lidar_tf")
    use_map_server = LaunchConfiguration("use_map_server")
    use_rviz = LaunchConfiguration("use_rviz")

    map_yaml = LaunchConfiguration("map_yaml")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    gt_publish_rate_hz = LaunchConfiguration("gt_publish_rate_hz")
    read_retry_period_sec = LaunchConfiguration("read_retry_period_sec")
    gps_initial_x = LaunchConfiguration("gps_initial_x")
    gps_initial_y = LaunchConfiguration("gps_initial_y")
    gps_initial_yaw = LaunchConfiguration("gps_initial_yaw")
    gps_calibrate = LaunchConfiguration("gps_calibrate")
    pal_python_path = LaunchConfiguration("pal_python_path")
    pal_runtime_path = LaunchConfiguration("pal_runtime_path")
    virtual_qcar_type = LaunchConfiguration("virtual_qcar_type")

    apply_gps_to_map_transform = LaunchConfiguration("apply_gps_to_map_transform")
    gps_to_map_scale = LaunchConfiguration("gps_to_map_scale")
    gps_to_map_theta = LaunchConfiguration("gps_to_map_theta")
    gps_to_map_tx = LaunchConfiguration("gps_to_map_tx")
    gps_to_map_ty = LaunchConfiguration("gps_to_map_ty")
    gps_to_map_tz = LaunchConfiguration("gps_to_map_tz")

    publish_pose_topic = LaunchConfiguration("publish_pose_topic")
    publish_odom_topic = LaunchConfiguration("publish_odom_topic")
    publish_path_topic = LaunchConfiguration("publish_path_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    path_topic = LaunchConfiguration("path_topic")
    max_path_points = LaunchConfiguration("max_path_points")
    pose_cov_xy = LaunchConfiguration("pose_cov_xy")
    pose_cov_yaw = LaunchConfiguration("pose_cov_yaw")

    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_yaw = LaunchConfiguration("lidar_tf_yaw")
    lidar_tf_pitch = LaunchConfiguration("lidar_tf_pitch")
    lidar_tf_roll = LaunchConfiguration("lidar_tf_roll")
    lidar_parent_frame = LaunchConfiguration("lidar_parent_frame")
    lidar_child_frame = LaunchConfiguration("lidar_child_frame")

    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    map_yaml_param = ParameterValue(map_yaml, value_type=str)
    autostart_param = ParameterValue(autostart, value_type=bool)
    publish_rate_param = ParameterValue(gt_publish_rate_hz, value_type=float)
    read_retry_period_sec_param = ParameterValue(read_retry_period_sec, value_type=float)
    gps_initial_x_param = ParameterValue(gps_initial_x, value_type=float)
    gps_initial_y_param = ParameterValue(gps_initial_y, value_type=float)
    gps_initial_yaw_param = ParameterValue(gps_initial_yaw, value_type=float)
    gps_calibrate_param = ParameterValue(gps_calibrate, value_type=bool)
    pal_python_path_param = ParameterValue(pal_python_path, value_type=str)
    pal_runtime_path_param = ParameterValue(pal_runtime_path, value_type=str)
    virtual_qcar_type_param = ParameterValue(virtual_qcar_type, value_type=int)
    apply_gps_to_map_transform_param = ParameterValue(apply_gps_to_map_transform, value_type=bool)
    gps_to_map_scale_param = ParameterValue(gps_to_map_scale, value_type=float)
    gps_to_map_theta_param = ParameterValue(gps_to_map_theta, value_type=float)
    gps_to_map_tx_param = ParameterValue(gps_to_map_tx, value_type=float)
    gps_to_map_ty_param = ParameterValue(gps_to_map_ty, value_type=float)
    gps_to_map_tz_param = ParameterValue(gps_to_map_tz, value_type=float)
    publish_pose_topic_param = ParameterValue(publish_pose_topic, value_type=bool)
    publish_odom_topic_param = ParameterValue(publish_odom_topic, value_type=bool)
    publish_path_topic_param = ParameterValue(publish_path_topic, value_type=bool)
    max_path_points_param = ParameterValue(max_path_points, value_type=int)
    pose_cov_xy_param = ParameterValue(pose_cov_xy, value_type=float)
    pose_cov_yaw_param = ParameterValue(pose_cov_yaw, value_type=float)

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_qcar2_virtual_nodes", default_value="true"),
        DeclareLaunchArgument("use_qcar2_lidar_tf", default_value="true"),
        DeclareLaunchArgument("use_map_server", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("map_yaml", default_value="/workspaces/isaac_ros-dev/ros2/map/map.yaml"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("rviz_config_file", default_value=default_rviz_config),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("gt_publish_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("read_retry_period_sec", default_value="1.0"),
        DeclareLaunchArgument("gps_initial_x", default_value="0.0"),
        DeclareLaunchArgument("gps_initial_y", default_value="0.0"),
        DeclareLaunchArgument("gps_initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("gps_calibrate", default_value="false"),
        DeclareLaunchArgument("pal_python_path", default_value="/home/Quanser/0_libraries/python"),
        DeclareLaunchArgument("pal_runtime_path", default_value="/tmp/qcar_pal_runtime"),
        DeclareLaunchArgument("virtual_qcar_type", default_value="2"),
        DeclareLaunchArgument("apply_gps_to_map_transform", default_value="true"),
        DeclareLaunchArgument("gps_to_map_scale", default_value="0.975"),
        DeclareLaunchArgument("gps_to_map_theta", default_value="0.717702849743"),
        DeclareLaunchArgument("gps_to_map_tx", default_value="0.352846958100"),
        DeclareLaunchArgument("gps_to_map_ty", default_value="1.382286458114"),
        DeclareLaunchArgument("gps_to_map_tz", default_value="0.0"),
        DeclareLaunchArgument("publish_pose_topic", default_value="true"),
        DeclareLaunchArgument("publish_odom_topic", default_value="true"),
        DeclareLaunchArgument("publish_path_topic", default_value="true"),
        DeclareLaunchArgument("pose_topic", default_value="/gps_pose"),
        DeclareLaunchArgument("odom_topic", default_value="/gps_odom"),
        DeclareLaunchArgument("path_topic", default_value="/gps_path"),
        DeclareLaunchArgument("max_path_points", default_value="5000"),
        DeclareLaunchArgument("pose_cov_xy", default_value="0.01"),
        DeclareLaunchArgument("pose_cov_yaw", default_value="0.02"),
        DeclareLaunchArgument("lidar_tf_x", default_value="0.1"),
        DeclareLaunchArgument("lidar_tf_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument("lidar_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_child_frame", default_value="base_scan"),
    ]

    qcar2_virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("qcar2_nodes"),
                "launch",
                "qcar2_virtual_launch.py",
            )
        ),
        condition=IfCondition(use_qcar2_virtual_nodes),
    )

    fixed_lidar_tf_node = Node(
        condition=IfCondition(use_qcar2_lidar_tf),
        package="qcar2_nodes",
        executable="fixed_lidar_frame_virtual",
        name="fixed_lidar_frame",
        output="screen",
    )

    static_lidar_tf_node = Node(
        condition=UnlessCondition(use_qcar2_lidar_tf),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fixed_lidar_frame_static",
        output="screen",
        arguments=[
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_yaw,
            lidar_tf_pitch,
            lidar_tf_roll,
            lidar_parent_frame,
            lidar_child_frame,
        ],
    )

    qcar_gps_tf_node = Node(
        package="qcar2_ground_truth_tf",
        executable="qcar_gps_tf_node.py",
        name="qcar_gps_tf_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_rate_hz": publish_rate_param,
                "read_retry_period_sec": read_retry_period_sec_param,
                "gps_initial_x": gps_initial_x_param,
                "gps_initial_y": gps_initial_y_param,
                "gps_initial_yaw": gps_initial_yaw_param,
                "gps_calibrate": gps_calibrate_param,
                "pal_python_path": pal_python_path_param,
                "pal_runtime_path": pal_runtime_path_param,
                "virtual_qcar_type": virtual_qcar_type_param,
                "apply_gps_to_map_transform": apply_gps_to_map_transform_param,
                "gps_to_map_scale": gps_to_map_scale_param,
                "gps_to_map_theta": gps_to_map_theta_param,
                "gps_to_map_tx": gps_to_map_tx_param,
                "gps_to_map_ty": gps_to_map_ty_param,
                "gps_to_map_tz": gps_to_map_tz_param,
                "publish_pose_topic": publish_pose_topic_param,
                "publish_odom_topic": publish_odom_topic_param,
                "publish_path_topic": publish_path_topic_param,
                "pose_topic": pose_topic,
                "odom_topic": odom_topic,
                "path_topic": path_topic,
                "max_path_points": max_path_points_param,
                "pose_cov_xy": pose_cov_xy_param,
                "pose_cov_yaw": pose_cov_yaw_param,
            }
        ],
    )

    map_server_node = Node(
        condition=IfCondition(use_map_server),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "yaml_filename": map_yaml_param,
            },
        ],
    )

    lifecycle_manager_localization = Node(
        condition=IfCondition(use_map_server),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "autostart": autostart_param,
                "node_names": ["map_server"],
            },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_ground_truth",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time_param}],
    )

    return LaunchDescription(
        declare_args
        + [
            qcar2_virtual_launch,
            fixed_lidar_tf_node,
            static_lidar_tf_node,
            qcar_gps_tf_node,
            map_server_node,
            lifecycle_manager_localization,
            rviz_node,
        ]
    )
