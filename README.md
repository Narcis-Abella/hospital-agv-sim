# hospital-agv-sim

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)  
[![Sim](https://img.shields.io/badge/Sim-Ignition%20Gazebo%20Fortress-orange.svg)](https://gazebosim.org/docs/fortress)  
[![Metrology](https://img.shields.io/badge/Research-Noise%20Models-gold.svg)](docs/RESEARCH.md)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

High‑fidelity ROS 2 / Ignition Gazebo hospital simulation for the AgileX Tracer 2, focused on **closing the sim‑to‑real gap in localization via physics‑grounded sensor noise models**.

---

## 🔬 Research-Backed Simulation

Unlike standard simulation packages that use ideal sensors or simplistic Gaussian noise, this repository implements **metrologically grounded noise models**. Every sensor parameter is derived from technical datasheets or field observations on real hardware.

> **Technical Documentation:** For a deep dive into the mathematical models (Gauss-Markov, radial Gaussian, etc.) and the datasheet justifications, see [**docs/RESEARCH.md**](docs/RESEARCH.md).

---

## Project Overview & Motivation

Real deployments of the AgileX Tracer 2 in hospital corridors showed that localization algorithms tuned in “clean” simulation did not transfer: EKF and LiDAR‑based SLAM under‑estimated uncertainty and diverged once exposed to real sensor bias, drift, and range‑dependent noise. 

This repository provides a drop‑in ROS 2 Humble package with:
1. **Hospital Scale World:** ~2,500 m² environment with domain‑relevant occlusions.
2. **Tracer 2 URDF/xacro:** Accurate geometry and TF tree for the AgileX platform.
3. **C++ Sensor Noise Pipeline:** A modular middleware that transforms ideal simulation data into realistic, datasheet‑compliant ROS 2 topics.

---

## Key Features & Technical Highlights

- **Physics-Grounded Noise Nodes:** Custom C++ implementation of IMU Gauss‑Markov bias, range‑proportional LiDAR noise, and wheel slip models.
- **Direct EKF Compatibility:** Nodes publish realistic covariance matrices, eliminating the need for arbitrary "tuning" inflations in `robot_localization`.
- **Latency Optimized:** C++ rewrite of legacy Python prototypes to ensure <1 ms processing jitter on Jetson Orin NX at high IMU rates (100 Hz+).
- **Sim-to-Real Roadmap:** Explicitly identifies validation gaps (Allan variance, odometry drift) to be closed with physical platform measurements.

### Sensor Model Summary

| Sensor                  | Model (Rationale)                                      | Primary Ref. |
|-------------------------|--------------------------------------------------------|--------------|
| **WitMotion WT901C**    | Gauss‑Markov Bias + G‑sensitivity + Quantization       | [Woodman 2007] |
| **RPLidar A2M12**       | Range‑proportional Gaussian (Triangulation model)      | [Thrun 2005] |
| **Livox Mid‑360**       | Radial Gaussian (ToF accuracy spec)                    | [Pomerleau 2013] |
| **Wheel Odometry**      | Proportional Slip + Unbounded Yaw Random Walk          | [Borenstein 1996] |

---

## Architecture & Data Flow

![Sensor Noise Pipeline](docs/noisy_diagram.PNG)

---

## 🎬 Noise Model Visualization (Ideal vs. Realistic)

Comparison of the Livox Mid-360 point cloud in RViz2: **Raw Simulation** (left) vs. **Realistic Noise Model** (right). Note the radial "spreading" of points as distance increases, mimicking actual sensor behavior.

<div align="center">
  <img src="docs/livox_raw_vs_noisy.gif" width="100%" alt="Livox Noise Model Comparison">
</div>

---

## 🚀 Research Roadmap (Sim-to-Real Validation)

Current status: **Datasheet-informed estimates**.

- [ ] **Allan Variance Pass:** characterization of the physical WT901C IMU to replace literature-standard bias parameters with measured values.
- [ ] **Odometry Drift Pass:** In-place rotation tests with the Tracer 2.0 to validate the 8% angular noise ratio.
- [ ] **Reflectivity Analysis:** Implementation of reflectivity-dependent LiDAR noise for hospital-grade materials (linoleum, glass).
- [ ] **Code Refactoring:** Unification of Livox nodes into a parametric C++ class.

---

## Installation & Usage

Refer to the [original documentation section](README.md#getting-started) for build instructions and launch commands.

```bash
# Launch simulation with all noise nodes enabled
ros2 launch hospital-agv-sim simulation.launch.py headless:=false
```

---

## License

MIT License. See [`LICENSE`](LICENSE) for details.

---

## Acknowledgements

- Original ROS 1 hospital environment by [@javicensaez](https://github.com/javicensaez/tracerSencillo) / SocialTech‑Challenge.  
- ROS 2 migration, sensor stack extension, and metrological noise modeling by Narcis Abella.
