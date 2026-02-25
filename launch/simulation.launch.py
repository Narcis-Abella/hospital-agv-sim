import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = "hospital-agv-sim"
    package_share = get_package_share_directory(package_name)
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ── 1. ENVIRONMENT / RESOURCE PATHS ───────────────────────────────────────
    models_path = os.path.join(package_share, "models")
    workspace_root = os.path.dirname(package_share)
    ign_resource_path = f"{workspace_root}:{models_path}"

    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        ign_resource_path += f":{os.environ['IGN_GAZEBO_RESOURCE_PATH']}"

    gz_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=ign_resource_path,
    )

    sdf_file = os.path.join(package_share, "worlds", "loyola.sdf")
    xacro_file = os.path.join(package_share, "urdf", "tracer2.xacro")

    # ── 2. LAUNCH ARGUMENTS ───────────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description=(
            "Run Ignition Gazebo in headless mode (no GUI). "
            "Set to 'false' to enable the graphical client."
        ),
    )
    headless_cfg = LaunchConfiguration("headless")

    # Inform the user when running in headless mode and how to enable the GUI.
    gui_command_hint = (
        "ros2 launch hospital-agv-sim simulation.launch.py headless:=false"
    )
    headless_info = LogInfo(
        condition=IfCondition(headless_cfg),
        msg=(
            "[hospital-agv-sim] Launch is running in HEADLESS mode (no GUI). "
            "Run `" + gui_command_hint + "` to start the simulation with "
            "the graphical client enabled."
        ),
    )

    # ── 3. ROBOT DESCRIPTION ──────────────────────────────────────────────────
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {"robot_description": doc.toxml(), "use_sim_time": True}

    # ── 4. SIMULATION (IGNITION GAZEBO) ───────────────────────────────────────────
    # -s : server-only (headless). Omit for full GUI (gzclient + gzserver).
    # -r : start simulation immediately on launch.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                f'"-s -r {sdf_file}" if "',
                LaunchConfiguration("headless"),
                f'" == "true" else "-r {sdf_file}"',
            ])
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "tracer_robot",
            "-world",
            "loyola",
            "-x",
            "-13.0",
            "-y",
            "5.0",
            "-z",
            "0.3",
            "-Y",
            "-1.5708",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ── 5. GAZEBO ↔ ROS 2 BRIDGES ─────────────────────────────────────────────
    # All sensors are bridged to *_raw topics and then processed by noise nodes.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # System
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            # "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # Sensors: all go to *_raw topics for post-processing
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/livox_mid70/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/livox_mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            # Critical routing: Gazebo -> bridge -> *_raw -> noise node -> final topic
            ("/imu", "/imu_raw"),
            ("/odom", "/odom_raw"),
            ("/scan", "/scan_raw"),
            ("/livox_mid70/points", "/livox_mid70/points_raw"),
            ("/livox_mid360/points", "/livox_mid360/points_raw"),
            # Camera
            ("/rgbd_camera/camera_info", "/camera/color/camera_info"),
        ],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/rgbd_camera/image", "/rgbd_camera/depth_image"],
        remappings=[
            ("/rgbd_camera/image", "/camera/color/image_raw"),
            ("/rgbd_camera/depth_image", "/camera/aligned_depth_to_color/image_raw"),
        ],
        output="screen",
    )

    # ── 6. SENSOR NOISE NODES (C++) ───────────────────────────────────────────
    # IMU: reads /imu_raw, applies Gauss–Markov noise, publishes /imu/data
    noisy_imu_node = Node(
        package="hospital-agv-sim",
        executable="noisy_imu_cpp",
        name="noisy_imu_node",
        parameters=[{"use_sim_time": True, "update_rate": 100.0}],
        output="screen",
    )

    # 2D LiDAR: reads /scan_raw, applies range-dependent noise, publishes /scan
    noisy_lidar_node = Node(
        package="hospital-agv-sim",
        executable="noisy_lidar_cpp",
        name="noisy_lidar_node",
        parameters=[
            {
                "use_sim_time": True,
                "rel_noise": 0.01,
                "min_noise": 0.003,
            }
        ],
        output="screen",
    )

    # Wheel odometry: reads /odom_raw, applies slip + yaw drift, publishes /odom
    noisy_odom_node = Node(
        package="hospital-agv-sim",
        executable="noisy_odom_cpp",
        name="noisy_odom_node",
        parameters=[
            {
                "use_sim_time": True,
                "lin_noise_ratio": 0.02,
                "ang_noise_ratio": 0.08,
                "yaw_drift_rate": 0.005,
            }
        ],
        output="screen",
    )

    # Livox Mid-70: reads /livox_mid70/points_raw, publishes /livox_mid70/points
    noisy_mid70_node = Node(
        package="hospital-agv-sim",
        executable="noisy_livox_mid70_cpp",
        name="noisy_livox_mid70_node",
        parameters=[
            {
                "use_sim_time": True,
                "rel_noise": 0.005,  # 0.5 %
                "min_noise": 0.002,  # 2 mm
            }
        ],
        output="screen",
    )

    # Livox Mid-360: reads /livox_mid360/points_raw, publishes /livox_mid360/points
    noisy_mid360_node = Node(
        package="hospital-agv-sim",
        executable="noisy_livox_mid360_cpp",
        name="noisy_livox_mid360_node",
        parameters=[
            {
                "use_sim_time": True,
                "rel_noise": 0.005,
                "min_noise": 0.002,
            }
        ],
        output="screen",
    )

    # ── 7. LAUNCH DESCRIPTION ────────────────────────────────────────────────
    return LaunchDescription(
        [
            headless_arg,
            headless_info,
            gz_resource_path,
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            bridge,
            image_bridge,
            noisy_imu_node,
            noisy_lidar_node,
            noisy_odom_node,
            noisy_mid70_node,
            noisy_mid360_node,
        ]
    )
