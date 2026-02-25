cmake_minimum_required(VERSION 3.22)
project(hospital_sim)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# IDE / clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Build dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Note: ros_gz_sim and ros_gz_bridge are runtime dependencies only.
# No find_package needed unless linking against their C++ libraries directly.

# ── C++ noise nodes ───────────────────────────────────────────────────────────
add_executable(noisy_imu_cpp src/noisy_imu.cpp)
ament_target_dependencies(noisy_imu_cpp rclcpp sensor_msgs)

add_executable(noisy_lidar_cpp src/noisy_lidar.cpp)
ament_target_dependencies(noisy_lidar_cpp rclcpp sensor_msgs)

add_executable(noisy_odom_cpp src/noisy_odom.cpp)
ament_target_dependencies(noisy_odom_cpp rclcpp nav_msgs geometry_msgs tf2_ros)

add_executable(noisy_livox_mid70_cpp src/noisy_livox_mid70.cpp)
ament_target_dependencies(noisy_livox_mid70_cpp rclcpp sensor_msgs)

add_executable(noisy_livox_mid360_cpp src/noisy_livox_mid360.cpp)
ament_target_dependencies(noisy_livox_mid360_cpp rclcpp sensor_msgs)

# ── Install: C++ executables ──────────────────────────────────────────────────
install(TARGETS
  noisy_imu_cpp
  noisy_lidar_cpp
  noisy_odom_cpp
  noisy_livox_mid70_cpp
  noisy_livox_mid360_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install: legacy Python prototypes (reference / regression only) ───────────
install(PROGRAMS
  scripts/legacy/noisy_odom.py
  scripts/legacy/noisy_imu.py
  scripts/legacy/noisy_lidar.py
  scripts/legacy/livox_pattern.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install: resource directories ─────────────────────────────────────────────
install(DIRECTORY
  launch
  worlds
  urdf
  meshes
  models
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
