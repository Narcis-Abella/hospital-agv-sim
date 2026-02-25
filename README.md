# hospital_sim

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)  
[![Sim](https://img.shields.io/badge/Sim-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/docs/fortress)  
[![Lang](https://img.shields.io/badge/C%2B%2B-17-green.svg)](https://en.cppreference.com/w/cpp/17)  
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

High‑fidelity ROS 2 / Ignition Gazebo hospital simulation for the AgileX Tracer 2, focused on **closing the sim‑to‑real gap in localization via physics‑grounded sensor noise models**.

---

## Project Overview & Motivation

Real deployments of the AgileX Tracer 2 in hospital corridors showed that localization algorithms tuned in “clean” simulation did not transfer: EKF and LiDAR‑based SLAM under‑estimated uncertainty and diverged once exposed to real sensor bias, drift, and range‑dependent noise. Upstream simulation packages shipped ideal sensors and simplistic noise primitives that could not be calibrated against datasheets or metrology data.

This repository provides a drop‑in ROS 2 Humble package with a hospital floor world, Tracer 2 URDF/xacro, and a **modular C++ sensor noise pipeline** sitting between Ignition Gazebo and the rest of the stack. The design goal is: if a localization stack is stable here with realistic covariances, it should be within tuning distance of a real Jetson‑powered robot on the same hardware configuration.

---

## Key Features & Technical Highlights

- Full migration of the original SocialTech ROS 1 / Gazebo Classic environment to **ROS 2 Humble + Ignition Gazebo Fortress**.  
- Hospital‑scale indoor world (~2,500 m²) with domain‑relevant assets (beds, trolleys, medical equipment) for realistic occlusions and LiDAR returns.  
- Tracer 2 robot model with RGB‑D camera, 2D LiDAR, dual Livox 3D LiDARs, IMU and wheel odometry, wired for ros_gz_bridge.  
- **Per‑sensor C++ noise nodes** implementing IMU Gauss‑Markov bias, LiDAR range‑dependent Gaussian noise, and wheel slip / yaw drift models derived from datasheets and field behaviour.  
- Realistic covariance matrices suitable for direct EKF / SLAM consumption, avoiding ad‑hoc inflations on top of ideal sensor topics.  
- Legacy Python noise prototypes preserved under `scripts/legacy/` for regression and model explainability, with measured GIL‑induced latency motivating the C++ rewrite.

### Concrete Sensor Models

| Sensor                  | Node                      | Model (conceptual)                                      | Example parameters                |
|-------------------------|---------------------------|---------------------------------------------------------|-----------------------------------|
| WT901C IMU              | `noisy_imu_cpp`           | Gauss‑Markov bias + G‑sensitivity + quantization        | `gyro_bias_tau=400 s`, `accel_bias_tau=300 s` |
| RPLidar 2D              | `noisy_lidar_cpp`         | Range‑proportional Gaussian                             | `σ = max(min_σ, rel·r)`           |
| Wheel odometry          | `noisy_odom_cpp`          | Velocity‑proportional slip + systematic yaw drift       | `lin_ratio≈2%`, `yaw_drift≈0.005 rad/m` |
| Livox Mid‑70 / Mid‑360  | `noisy_livox_mid70_cpp` / `noisy_livox_mid360_cpp` | Radial Gaussian on PointCloud2 tuned to Livox accuracy | `rel≈0.5%`, `min≈2 mm`            |

---

## Design Decisions & Rationale

- **Noise in separate C++ nodes instead of Gazebo plugins**: Gazebo’s built‑in noise parameters are intentionally simple and world‑specific. By routing `/sensor_raw → C++ noise node → /sensor`, the same physics model can be reused across robots and scenarios and tuned without touching the URDF or SDF.  
- **Python prototypes kept but not in the hot path**: Early iterations used Python nodes, which exposed >15 ms jitter at 100 Hz IMU rates on Jetson Orin due to the GIL under load. Re‑implementing in C++ removes that latency while keeping the Python versions as reference and regression tests.  
- **Datasheet‑driven parameters instead of “nice‑looking” noise**: IMU bias and Livox LiDAR accuracy are taken from Allan variance and vendor specs, so the resulting covariances are defensible in a metrology or code review context.  
- **Hospital‑specific world instead of generic maze**: The world models typical hospital furniture and equipment so that occlusions, multipath and narrow corridors resemble the eventual deployment domain. This avoids tuning SLAM on unrealistic, open‑space scenes.  
- **ROS 2 Humble + Ignition Fortress**: This combination matches current industrial ROS deployments and provides long‑term support, modern physics, and a stable API for ros_gz_bridge.
- **Reliable QoS on all noise nodes**: all noise node subscribers and publishers use `rclcpp::QoS(10)` (Reliable), matching the `parameter_bridge` default output. Using `SensorDataQoS` (BestEffort) here would work but is incorrect
  for an EKF consumer that requires message delivery guarantees. Run `ros2 topic info <topic> --verbose` to verify QoS compatibility if swapping the bridge implementation.

---

## Architecture & Data Flow

At a high level, every sensor follows the same pattern: Ignition publishes ideal data, ros_gz_bridge exposes it as ROS 2 topics, and a dedicated noise node transforms it into a processed topic consumed by downstream stacks.

```text
Ignition Gazebo Sensor
        │
        │  gz.msgs.* (internal)
        ▼
ros_gz_bridge ───────────────→ /<sensor>_raw   (ROS 2, Reliable QoS)
                                      │
                              C++ Noise Node
                                      │
                              /<sensor>        (ROS 2, processed)
                                      │
                           EKF / SLAM / Nav stack
```

Main components:

- `urdf/tracer2.xacro` and `urdf/sensors/*.xacro`: robot and sensor mounting geometry.  
- `worlds/loyola*.sdf`: hospital floor worlds with static obstacles.  
- `src/*.cpp`: C++ nodes implementing the noise models.  
- `launch/simulation.launch.py`: brings up Ignition, the Tracer 2, bridges, and all noise nodes in one command.  
- `docs/architecture.md`: additional design notes and diagrams for deeper review.

---

## Tech Stack & Environment

| Layer          | Choice / Version                             |
|----------------|----------------------------------------------|
| ROS middleware | ROS 2 Humble (Ubuntu 22.04 / JetPack 5.x)    |
| Simulator      | Ignition Gazebo 6 (Fortress)                 |
| Language       | C++17 for noise nodes, Python for legacy prototypes |
| Bridge         | `ros_gz` ROS 2–Gazebo bridge                 |
| Robot          | AgileX Tracer 2 differential‑drive AGV       |
| Compute target | NVIDIA Jetson Orin, Ubuntu 20.04 ARM64 on JetPack 5.x |

---

## Getting Started

### Prerequisites

Install the required ROS 2 Humble and simulator packages:

```bash
# ROS 2 Humble (Ubuntu 22.04 or JetPack 5.x with ROS 2)
# Refer to the official ROS 2 Humble installation guide.

sudo apt install ignition-fortress \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

All commands below assume a standard colcon workspace at `~/ros2_ws`.

### Installation

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Narcis-Abella/hospital_sim.git

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select hospital_sim
source install/setup.bash
```

---


## Usage / Quick Start

`simulation.launch.py` accepts a `headless` argument (default: `true`) that controls whether Ignition Gazebo runs with or without a graphical client.

**Headless mode** (default — recommended for Jetson or SSH sessions):

```bash
ros2 launch hospital_sim simulation.launch.py
```

**GUI mode** (opens the Ignition Gazebo graphical client):

```bash
ros2 launch hospital_sim simulation.launch.py headless:=false
```

Validate that all processed sensor topics are active:

```bash
ros2 topic list | grep -E "imu|scan|odom|livox"
```

Inspect a specific noise model output, e.g. IMU:

```bash
ros2 topic echo /imu/data --once
```

Teleoperate the robot from another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Expected processed topics after launch:

```text
/imu/data               <- IMU with Gauss-Markov bias + quantization
/scan                   <- 2D LiDAR with proportional Gaussian noise
/odom                   <- wheel odom with slip + yaw drift
/livox_mid70/points     <- front Livox with radial Gaussian noise
/livox_mid360/points    <- surround Livox with radial Gaussian noise
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
```

---

## Repository Structure

```text
hospital_sim/
├── src/                # C++ sensor noise nodes (IMU, LiDAR, odom, Livox)
├── launch/             # ROS 2 launch files (simulation bring-up)
├── urdf/
│   ├── sensors/        # Per-sensor xacro macros
│   └── tracer2.xacro   # Tracer 2 robot description
├── worlds/             # Ignition SDF worlds (hospital layouts)
├── meshes/             # Robot visual/collision geometry
├── models/             # Gazebo model assets (furniture, medical equipment)
├── scripts/
│   └── legacy/         # Python noise prototypes (superseded by C++)
├── docs/               # Architecture and design notes
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

---

## License

MIT License. See [`LICENSE`](LICENSE) for details.

---

## Acknowledgements

- Original ROS 1 hospital environment and Tracer 2 simulation by [@javicensaez](https://github.com/javicensaez/tracerSencillo) / SocialTech‑Challenge.  
- ROS 2 migration, sensor stack extension, and noise modeling by Narcis Abella.