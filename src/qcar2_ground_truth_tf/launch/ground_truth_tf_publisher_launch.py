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
    apply_qlabs_to_map_transform = LaunchConfiguration("apply_qlabs_to_map_transform")
    qlabs_to_map_scale = LaunchConfiguration("qlabs_to_map_scale")
    qlabs_to_map_theta = LaunchConfiguration("qlabs_to_map_theta")
    qlabs_to_map_tx = LaunchConfiguration("qlabs_to_map_tx")
    qlabs_to_map_ty = LaunchConfiguration("qlabs_to_map_ty")
    qlabs_to_map_tz = LaunchConfiguration("qlabs_to_map_tz")

    qlabs_host = LaunchConfiguration("qlabs_host")
    actor_number = LaunchConfiguration("actor_number")
    auto_discover_actor = LaunchConfiguration("auto_discover_actor")
    actor_search_min = LaunchConfiguration("actor_search_min")
    actor_search_max = LaunchConfiguration("actor_search_max")
    actor_search_period_sec = LaunchConfiguration("actor_search_period_sec")
    spawn_qcar = LaunchConfiguration("spawn_qcar")

    publish_pose_topic = LaunchConfiguration("publish_pose_topic")
    publish_path_topic = LaunchConfiguration("publish_path_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    path_topic = LaunchConfiguration("path_topic")
    max_path_points = LaunchConfiguration("max_path_points")

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
    actor_number_param = ParameterValue(actor_number, value_type=int)
    auto_discover_actor_param = ParameterValue(auto_discover_actor, value_type=bool)
    actor_search_min_param = ParameterValue(actor_search_min, value_type=int)
    actor_search_max_param = ParameterValue(actor_search_max, value_type=int)
    actor_search_period_sec_param = ParameterValue(actor_search_period_sec, value_type=float)
    spawn_qcar_param = ParameterValue(spawn_qcar, value_type=bool)
    publish_rate_param = ParameterValue(gt_publish_rate_hz, value_type=float)
    apply_qlabs_to_map_transform_param = ParameterValue(apply_qlabs_to_map_transform, value_type=bool)
    qlabs_to_map_scale_param = ParameterValue(qlabs_to_map_scale, value_type=float)
    qlabs_to_map_theta_param = ParameterValue(qlabs_to_map_theta, value_type=float)
    qlabs_to_map_tx_param = ParameterValue(qlabs_to_map_tx, value_type=float)
    qlabs_to_map_ty_param = ParameterValue(qlabs_to_map_ty, value_type=float)
    qlabs_to_map_tz_param = ParameterValue(qlabs_to_map_tz, value_type=float)
    publish_pose_topic_param = ParameterValue(publish_pose_topic, value_type=bool)
    publish_path_topic_param = ParameterValue(publish_path_topic, value_type=bool)
    max_path_points_param = ParameterValue(max_path_points, value_type=int)

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
        DeclareLaunchArgument("apply_qlabs_to_map_transform", default_value="true"),
        DeclareLaunchArgument("qlabs_to_map_scale", default_value="0.975"),
        DeclareLaunchArgument("qlabs_to_map_theta", default_value="0.717702849743"),
        DeclareLaunchArgument("qlabs_to_map_tx", default_value="0.352846958100"),
        DeclareLaunchArgument("qlabs_to_map_ty", default_value="1.382286458114"),
        DeclareLaunchArgument("qlabs_to_map_tz", default_value="0.0"),
        DeclareLaunchArgument("qlabs_host", default_value="localhost"),
        DeclareLaunchArgument("actor_number", default_value="0"),
        DeclareLaunchArgument("auto_discover_actor", default_value="true"),
        DeclareLaunchArgument("actor_search_min", default_value="0"),
        DeclareLaunchArgument("actor_search_max", default_value="255"),
        DeclareLaunchArgument("actor_search_period_sec", default_value="1.0"),
        DeclareLaunchArgument("spawn_qcar", default_value="false"),
        DeclareLaunchArgument("publish_pose_topic", default_value="true"),
        DeclareLaunchArgument("publish_path_topic", default_value="true"),
        DeclareLaunchArgument("pose_topic", default_value="/ground_truth/pose"),
        DeclareLaunchArgument("path_topic", default_value="/ground_truth/path"),
        DeclareLaunchArgument("max_path_points", default_value="5000"),
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

    ground_truth_tf_node = Node(
        package="qcar2_ground_truth_tf",
        executable="qlabs_ground_truth_tf_node.py",
        name="qlabs_ground_truth_tf_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_rate_hz": publish_rate_param,
                "apply_qlabs_to_map_transform": apply_qlabs_to_map_transform_param,
                "qlabs_to_map_scale": qlabs_to_map_scale_param,
                "qlabs_to_map_theta": qlabs_to_map_theta_param,
                "qlabs_to_map_tx": qlabs_to_map_tx_param,
                "qlabs_to_map_ty": qlabs_to_map_ty_param,
                "qlabs_to_map_tz": qlabs_to_map_tz_param,
                "qlabs_host": qlabs_host,
                "actor_number": actor_number_param,
                "auto_discover_actor": auto_discover_actor_param,
                "actor_search_min": actor_search_min_param,
                "actor_search_max": actor_search_max_param,
                "actor_search_period_sec": actor_search_period_sec_param,
                "spawn_qcar": spawn_qcar_param,
                "publish_pose_topic": publish_pose_topic_param,
                "publish_path_topic": publish_path_topic_param,
                "pose_topic": pose_topic,
                "path_topic": path_topic,
                "max_path_points": max_path_points_param,
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
            ground_truth_tf_node,
            map_server_node,
            lifecycle_manager_localization,
            rviz_node,
        ]
    )
