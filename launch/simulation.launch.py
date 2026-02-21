import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name  = 'hospital_sim'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── 1. RUTAS ──────────────────────────────────────────────────────────────
    models_path     = os.path.join(pkg_share, 'models')
    pkg_parent_path = os.path.dirname(pkg_share)
    new_ign_path    = f"{pkg_parent_path}:{models_path}"
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        new_ign_path += f":{os.environ['IGN_GAZEBO_RESOURCE_PATH']}"

    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=new_ign_path
    )

    sdf_file   = os.path.join(pkg_share, 'worlds', 'loyola.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'tracer2.xacro')

    # ── 2. ROBOT DESCRIPTION ──────────────────────────────────────────────────
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_desc = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # ── 3. SIMULACIÓN ─────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r {sdf_file}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'tracer_robot',
            '-world', 'loyola',
            '-x', '-13.0',
            '-y',  '5.0',
            '-z',  '0.3',
            '-Y', '-1.5708',
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_desc],
        output='screen'
    )

    # ── 4. BRIDGE GAZEBO ↔ ROS2 ───────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sistema
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Sensores — todos van a topics *_raw para ser procesados
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/livox_mid70/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/livox_mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            # Ruteo crítico: Gazebo -> Bridge -> Topic_RAW -> Nodo Ruido -> Topic Final
            ('/imu',          '/imu_raw'),
            ('/odom',         '/odom_raw'),
            ('/scan',         '/scan_raw'),          
            ('/livox_mid70/points', '/livox_mid70/points_raw'),
            ('/livox_mid360/points', '/livox_mid360/points_raw'), 
            # Cámara
            ('/rgbd_camera/camera_info', '/camera/color/camera_info'),
        ],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgbd_camera/image', '/rgbd_camera/depth_image'],
        remappings=[
            ('/rgbd_camera/image',       '/camera/color/image_raw'),
            ('/rgbd_camera/depth_image', '/camera/aligned_depth_to_color/image_raw'),
        ],
        output='screen'
    )

    # ── 5. NODOS DE RUIDO (HÍBRIDO C++ / PYTHON) ──────────────────────────────

    # IMU: C++ (Nuevo)
    # Lee /imu_raw, procesa Gauss-Markov en C++, publica /imu/data
    noisy_imu_node = Node(
        package='hospital_sim',
        executable='noisy_imu_cpp',   # <--- Ejecutable C++
        name='noisy_imu_node',
        parameters=[{'use_sim_time': True, 'update_rate': 100.0}],
        output='screen'
    )

    # RPLidar: C++ (Nuevo)
    # Lee /scan_raw, aplica ruido en C++, publica /scan
    noisy_lidar_node = Node(
        package='hospital_sim',
        executable='noisy_lidar_cpp', # <--- Ejecutable C++
        name='noisy_lidar_node',
        parameters=[{
            'use_sim_time': True,
            'rel_noise':    0.01,
            'min_noise':    0.003,
        }],
        output='screen'
    )

    # Odometría: Python (Legacy)
    noisy_odom_node = Node(
        package='hospital_sim',
        executable='noisy_odom_cpp',   # <--- CAMBIADO a _cpp
        name='noisy_odom_node',
        parameters=[{
            'use_sim_time':    True,
            'lin_noise_ratio': 0.02,
            'ang_noise_ratio': 0.08,
            'yaw_drift_rate':  0.005
        }],
        output='screen'
    )

    # Livox: C++
    # Lee /livox/points_raw
    # Aplica Ruido
    noisy_mid70_node = Node(
        package='hospital_sim',
        executable='noisy_livox_mid70_cpp',
        name='noisy_livox_mid70_node',
        parameters=[{
            'use_sim_time': True,
            'rel_noise':    0.005, # 0.5%
            'min_noise':    0.002, # 2mm
        }],
        output='screen'
    )

    noisy_mid360_node = Node(
        package='hospital_sim',
        executable='noisy_livox_mid360_cpp',
        name='noisy_livox_mid360_node',
        parameters=[{'use_sim_time': True, 'rel_noise': 0.005, 'min_noise': 0.002}],
        output='screen'
    )

    # ── 6. LAUNCH DESCRIPTION ─────────────────────────────────────────────────
    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        image_bridge,
        
        # Pipeline Activo
        noisy_imu_node,
        noisy_lidar_node,
        noisy_odom_node,
        noisy_mid70_node,
        noisy_mid360_node,
    ])