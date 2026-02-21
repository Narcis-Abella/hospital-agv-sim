cmake_minimum_required(VERSION 3.8)
project(hospital_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencias de Compilación ───
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       # Necesario para compilar los nodos C++
find_package(sensor_msgs REQUIRED)  # Necesario para mensajes Imu y LaserScan
find_package(nav_msgs REQUIRED)     # Necesario para Odometry (aunque sea en python, buena práctica)

# Nota: ros_gz_sim y bridge son dependencias de ejecución, no necesitas find_package aquí
# a menos que enlaces librerías de C++ de Gazebo directamente.

# ─── 1. Compilación de Nodos C++ ───

# Nodo IMU (C++)
add_executable(noisy_imu_cpp src/noisy_imu.cpp)
ament_target_dependencies(noisy_imu_cpp rclcpp sensor_msgs)

# Nodo LiDAR (C++)
add_executable(noisy_lidar_cpp src/noisy_lidar.cpp)
ament_target_dependencies(noisy_lidar_cpp rclcpp sensor_msgs)

# Nodo Odometría (C++)
add_executable(noisy_odom_cpp src/noisy_odom.cpp)
ament_target_dependencies(noisy_odom_cpp rclcpp nav_msgs)

# Nodo Livox Mid-70 (C++)
add_executable(noisy_livox_mid70_cpp src/noisy_livox_mid70.cpp)
ament_target_dependencies(noisy_livox_mid70_cpp rclcpp sensor_msgs)

# Nodo Livox Mid-360 (C++)
add_executable(noisy_livox_mid360_cpp src/noisy_livox_mid360.cpp)
ament_target_dependencies(noisy_livox_mid360_cpp rclcpp sensor_msgs)

# ─── 2. Instalación ───

# A) Instalar ejecutables C++
install(TARGETS
  noisy_imu_cpp
  noisy_lidar_cpp
  noisy_odom_cpp
  noisy_livox_mid70_cpp
  noisy_livox_mid360_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# B) Instalar scripts Python (Legacy/Híbrido)
install(PROGRAMS
  scripts/legacy/noisy_odom.py
  scripts/legacy/noisy_imu.py
  scripts/legacy/noisy_lidar.py
  scripts/legacy/livox_pattern.py
  DESTINATION lib/${PROJECT_NAME}
)

# C) Instalar carpetas de recursos (launch, meshes, etc.)
install(DIRECTORY 
  launch 
  worlds 
  urdf
  meshes
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()