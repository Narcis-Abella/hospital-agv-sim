# hospital-agv-sim

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Sim](https://img.shields.io/badge/Sim-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/docs/fortress)
[![Metrology](https://img.shields.io/badge/Research-Noise%20Models-gold.svg)](docs/RESEARCH.md)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

ROS 2 / Ignition Gazebo hospital simulation for the AgileX Tracer 2.  
The package focuses on reducing the sim‑to‑real gap in localization by making the behaviour of the simulated sensors closer to the real hardware.

---

## Motivation and scope

In real deployments of the AgileX Tracer 2 in hospital corridors, localization pipelines that were stable in “clean” simulation degraded noticeably on the robot. EKF and LiDAR‑based SLAM tended to underestimate uncertainty because the simulated sensors were almost ideal:

- the IMU showed little bias or drift,
- LiDAR ranges were nearly noise‑free,
- wheel odometry did not accumulate the heading error observed after a sequence of turns.

This repository extends a standard ROS 2 simulation with:

1. A hospital‑scale world (~2,500 m²) with domain‑relevant occlusions and corridors.
2. A URDF/xacro model of the Tracer 2 with a consistent TF tree and sensor frames.
3. A C++ sensor noise pipeline that consumes ideal Gazebo topics and republishes ROS 2 topics with realistic noise and covariances.

The goal is to tune the localization stack once in this environment and obtain behaviour that transfers more directly to the physical platform.

---

## Noise pipeline overview

The noise pipeline is implemented as a set of C++ nodes, one per sensor type (IMU, 2D LiDAR, 3D LiDAR, wheel odometry). Each node:

- subscribes to an ideal `/sensor_raw` topic,
- applies a stochastic model derived from datasheets and literature where possible,
- publishes a realistic `/sensor` topic with mean behaviour and covariances consistent with that model.

![Sensor Noise Pipeline](docs/noisy_diagram.png)

The diagram shows how ideal Gazebo topics are bridged into ROS 2 and passed through the noise nodes before reaching the localization stack.

### Model summary

| Sensor                  | Model (summary)                                  | Main reference                              |
|-------------------------|--------------------------------------------------|---------------------------------------------|
| IMU (WT901C / MPU‑9250) | Gauss‑Markov bias + white noise + quantization | MPU‑9250 spec, Woodman 2007                 |
| 2D LiDAR (RPLIDAR A2M12)| Range‑dependent Gaussian                        | RPLIDAR A2 spec, beam model literature      |
| 3D LiDAR (Livox Mid‑360)| Radial Gaussian, distance dependent             | Livox Mid‑360 spec, ICP error studies       |
| Wheel odometry          | Slip + yaw random walk                          | Borenstein & Feng 1996; Siegwart et al. 2011|

All parameter choices, unit conversions and detailed justifications are in `docs/RESEARCH.md`; implementation details and covariances are described in `docs/architecture.md`.

---

## Design choices

- **Noise layer instead of over‑tuning SLAM**  
  The dominant source of mismatch between simulation and reality in this context is unrealistic sensor behaviour. It is therefore preferable to make the sensor topics more representative of the hardware than to compensate with aggressive SLAM parameter tuning that does not transfer.

- **C++ implementation for the critical path**  
  Early prototypes of the noise nodes were written in Python. At IMU rates of 100 Hz and above on Jetson platforms, interpreter overhead and the GIL introduced measurable jitter. The current C++ implementation keeps latency and timing variation low enough for realistic evaluation.

- **Realism versus full calibration**  
  IMU and LiDAR noise parameters follow datasheets and standard stochastic models. Odometry parameters are chosen to reflect the order of magnitude observed on the Tracer 2, but they are not the result of a dedicated calibration campaign. The emphasis is on reproducing typical failure modes rather than matching a single calibrated robot exactly.


---

## Visuals and tech stack

Comparison of the Livox Mid-360 point cloud in RViz2: **Realistic Noise Model** (left) vs. **Raw Simulation** (right) . As distance increases, the point cloud spreads radially in a way that looks much closer to the real sensor.

<div align="center">
  <img src="docs/livox_noisy_vs_raw.gif" width="100%" alt="Livox Noise Model Comparison">
</div>

| Component        | Version / notes                                |
|-----------------|-----------------------------------------------|
| ROS 2           | Humble                                        |
| Simulator       | Ignition Gazebo Fortress                      |
| Language        | C++17                                         |
| Tested on       | Ubuntu 22.04, Jetson Orin                     |

---

## Getting started

### Prerequisites

- ROS 2 Humble installed and sourced.
- `colcon` available.
- Ignition Gazebo Fortress working.
- A ROS 2 workspace (for example `~/ros2_ws`).

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Narcis-Abella/hospital-agv-sim.git

cd ..
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select hospital-agv-sim
source install/setup.bash
```

### Run

```bash
# Hospital world + Tracer 2 + noise nodes
ros2 launch hospital-agv-sim simulation.launch.py headless:=false
```

Check the launch file for available options.

---

## Repository structure

```text
.
├── CMakeLists.txt      # Build configuration
├── package.xml         # ROS 2 package manifest
├── README.md           # Project overview and usage
├── LICENSE             # MIT license
├── docs/               # Research notes and architecture
│   ├── RESEARCH.md
│   └── architecture.md
├── launch/             # Launch files
├── src/                # C++ noise nodes
├── scripts/            # Legacy Python nodes (kept for reference)
├── urdf/               # Robot and sensor xacros
├── meshes/             # Meshes for the robot and environment
├── models/             # Gazebo models
└── worlds/             # Hospital simulation worlds
```

---

## License

MIT License. See `LICENSE` for details.

---

## Acknowledgements

- Hospital world for the original ROS 1 project by [`javicensaez/tracerSencillo`](https://github.com/javicensaez/tracerSencillo).
- ROS 2 migration, sensor stack and noise nodes by Narcis Abella.


