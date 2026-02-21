# hospital_sim

**ROS 2 / Ignition Gazebo simulation package for autonomous mobile robot development in structured indoor environments.**  
Migrated and extended from [SocialTech-Challenge/SocialTech-Gazebo](https://github.com/SocialTech-Challenge/SocialTech-Gazebo) (ROS 1 / Gazebo Classic).

---

## Project Overview

This package provides a high-fidelity simulation environment for an AgileX Tracer 2 differential-drive AGV operating in a hospital floor layout. The core engineering contribution of this fork is a **physics-grounded sensor noise pipeline** that replaces Gazebo's native (and insufficient) noise primitives with per-sensor models implemented in C++.

The original ROS 1 package shipped ideal sensors. Real-world deployment on hospital environments exposed a significant sim-to-real gap: localization algorithms (EKF, FAST-LIO) tuned in simulation failed in the field because the covariance matrices were incorrect and systematic sensor biases were absent. This package closes that gap.

**Key technical additions over the upstream:**
- Full migration to **ROS 2 Humble + Ignition Gazebo 6 (Fortress)**
- RGB-D camera replacing monocular (Intel RealSense D435i model)
- **Livox Mid-70** (70° × 70° solid-state LiDAR, forward-facing)
- **Livox Mid-360** (360° × 59° solid-state LiDAR, full surround)
- C++ sensor noise nodes replacing Python prototypes (latency reduction under high-frequency callbacks)
- Realistic sensor covariance matrices for direct EKF/SLAM consumption

---

## Sensor Noise Architecture

Gazebo publishes ideal sensor data. Each sensor is routed through a dedicated C++ node before being exposed to the rest of the stack. This keeps the simulation pipeline modular: swap noise parameters without touching the URDF or the world.

```
Gazebo Sensor
     │
     │  gz.msgs.* (internal)
     ▼
ros_gz_bridge  ──────────────────────→  /sensor_raw  (ROS 2, Reliable QoS)
                                              │
                                    C++ Noise Node
                                              │
                                    /sensor  (ROS 2, processed)
                                              │
                                     SLAM / EKF / Nav stack
```

### Noise Models per Sensor

| Sensor | Node | Model | Key Parameters |
|--------|------|-------|----------------|
| **IMU** (WT901C) | `noisy_imu_cpp` | Gauss-Markov bias + G-sensitivity + ADC quantization | `gyro_bias_tau=400s`, `accel_bias_tau=300s` |
| **RPLidar** (2D scan) | `noisy_lidar_cpp` | Proportional Gaussian `σ = max(min_σ, rel·r)` | `rel=0.01`, `min=3mm` |
| **Wheel Odometry** | `noisy_odom_cpp` | Velocity-proportional slip + systematic yaw drift | `lin_ratio=2%`, `drift=0.005 rad/m` |
| **Livox Mid-70** | `noisy_livox_mid70_cpp` | Radial Gaussian on PointCloud2 | `rel=0.5%`, `min=2mm` |
| **Livox Mid-360** | `noisy_livox_mid360_cpp` | Radial Gaussian on PointCloud2 | `rel=0.5%`, `min=2mm` |

The IMU model is derived from the WT901C datasheet (Allan variance figures). Livox noise parameters match the Mid-series published ranging accuracy (±2 cm at 1–6 m, ±3 cm at 6–40 m).

> **Design note on Python → C++ migration:**  
> Original noise nodes were prototyped in Python (`scripts/legacy/`). At 100 Hz IMU callbacks, the Python GIL introduced non-deterministic latency spikes (measured >15 ms jitter on Jetson Orin under load). All nodes were reimplemented in C++ with the same mathematical model. The Python versions are retained in `scripts/legacy/` for reference and regression benchmarking.

---

## Hardware Context

| Component | Specification |
|-----------|--------------|
| **Compute** | NVIDIA Jetson Orin (JetPack 5.x, Ubuntu 20.04 ARM64) |
| **Robot** | AgileX Tracer 2 — differential drive, 42 kg, 600×400 mm footprint |
| **2D LiDAR** | RPLidar S2 — 360°, 30 m range |
| **3D LiDAR (front)** | Livox Mid-70 — 70°×70° FOV, 0.1–40 m |
| **3D LiDAR (surround)** | Livox Mid-360 — 360°×59° FOV, 0.1–40 m |
| **Depth Camera** | Intel RealSense D435i — RGB-D, IMU |
| **Environment** | Hospital indoor layout, ~2,500 m², static obstacles |

---

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04) or ROS 2 Humble on JetPack 5.x
- Ignition Gazebo 6 (Fortress): `sudo apt install ignition-fortress`
- `ros_gz` bridge: `sudo apt install ros-humble-ros-gz`
- `xacro`: `sudo apt install ros-humble-xacro`
- `robot_state_publisher`: included in `ros-humble-robot-state-publisher`

---

## Installation

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/Narcis-Abella/hospital_sim.git

# 2. Build
cd ~/ros2_ws
colcon build --packages-select hospital_sim
source install/setup.bash
```

---

## Usage

```bash
# Launch full simulation (Gazebo + all sensor nodes)
ros2 launch hospital_sim simulation.launch.py

# Verify sensor pipeline is active
ros2 topic list | grep -E "imu|scan|odom|livox"

# Inspect noise node output (example: IMU)
ros2 topic echo /imu/data --once

# Teleop (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Expected active topics after launch:**

```
/imu/data               ← processed (Gauss-Markov + quantization)
/scan                   ← processed (proportional Gaussian)
/odom                   ← processed (slip + yaw drift)
/livox_mid70/points     ← processed (radial Gaussian)
/livox_mid360/points    ← processed (radial Gaussian)
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
```

---

## Repository Structure

```
hospital_sim/
├── src/                   C++ sensor noise nodes
├── launch/                ROS 2 launch files
├── urdf/
│   ├── sensors/           Per-sensor xacro macros
│   └── tracer2.xacro      Top-level robot description
├── worlds/                Ignition SDF world (hospital layout)
├── meshes/                Robot visual/collision geometry
├── models/                Gazebo model assets
├── scripts/
│   └── legacy/            Python noise prototypes (superseded by C++)
└── docs/                  Architecture diagrams and design notes
```

---

## License

MIT — see [LICENSE](LICENSE)

---

## Acknowledgements

Original ROS 1 simulation environment by [@javicensaez](https://github.com/javicensaez/tracerSencillo) / SocialTech-Challenge.  
ROS 2 migration, sensor stack extension, and noise modeling by Narcis Abella.
