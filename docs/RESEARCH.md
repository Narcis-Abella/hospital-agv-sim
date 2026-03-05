# Sensor Noise Models — Research and Parameters

> Validation status: all parameters in this document are estimates derived from vendor datasheets and field observation. There is no full calibration campaign on the physical platform yet. The values are meant to be plausible and conservative. Allan variance and dedicated hardware tests are listed as future work.

This document explains the noise models implemented in `src/` and where the numbers come from. The goal is that anyone touching these nodes later can see which parts come from datasheets, which parts come from papers, and which parts are just reasonable guesses.

---

## 1. IMU — WitMotion WT901C-TTL (`noisy_imu.cpp`)

### 1.1 Sensor identification

The WT901C-TTL is a consumer-grade AHRS module built around the InvenSense MPU-9250 MEMS die. Raw inertial measurements (gyroscope + accelerometer) come from the MPU-9250. The WitMotion board adds its own Kalman filter on top, but in this pipeline we only use the raw IMU topic, so the relevant specifications are those of the bare MPU-9250.

Datasheet: InvenSense MPU-9250 Product Specification Rev 1.1 (PS-MPU-9250A-01).

### 1.2 Gyroscope model

The node uses a first-order Gauss-Markov bias process on top of white measurement noise, plus a small G-sensitivity term and ADC quantization. This structure follows standard MEMS IMU models (for example Woodman 2007 and IEEE Std 952-1997).

#### White noise (Angle Random Walk)

The MPU-9250 datasheet reports:

```
Rate Noise Spectral Density: 0.01 °/s/√Hz  (typical, DLPF enabled)
```

Converting to rad/s/√Hz and integrating over a 100 Hz bandwidth:

```
PSD = 0.01 × (π/180) ≈ 1.745 × 10⁻⁴ rad/s/√Hz
σ_white = PSD × √(BW) = 1.745e-4 × √100 ≈ 0.00175 rad/s
```

This matches `gyro_white_std_ = 0.00175` in the code.

#### Bias instability (Gauss-Markov process)

The discrete-time Gauss-Markov recursion is:

```
b[k] = exp(-dt/τ) · b[k-1] + w[k]

w ~ N(0, σ_b · √(1 - exp(-2dt/τ)))
```

The datasheet does not publish bias instability directly. It only gives worst-case bounds like initial ZRO tolerance and variation over temperature. For sensors of this class, Allan variance studies typically show in-run bias in the range 0.01–0.05 °/s (1σ). The value used here is:

```
gyro_bias_std_ = 0.0003 rad/s  ≈  0.017 °/s
τ = 400 s
```

These numbers come from typical values reported in the literature, not from a calibration of this specific unit.

#### G-sensitivity

The MPU-9250 datasheet reports cross-axis sensitivity of ±2%. The implementation sets:

```
G_sensitivity_ = gyro_white_std_ / 9.81  ≈  1.78 × 10⁻⁴ (rad/s)/(m/s²)
```

This is a small value and has little effect in practice. It is not a rigorous derivation from the ±2% figure, so it should be treated as an approximation.

#### ADC quantization

The MPU-9250 uses 16-bit ADCs. At FS_SEL=3 (±2000 °/s) and AFS_SEL=3 (±16 g):

```
gyro_LSB  = (2 × 2000°/s × π/180) / 65536 ≈ 1.065 × 10⁻³ rad/s per LSB
accel_LSB = (2 × 16 × 9.81)       / 65536 ≈ 4.789 × 10⁻³ m/s² per LSB
```

These values come directly from the sensitivity scale factors in the MPU-9250 tables.

#### Accelerometer white noise

The datasheet reports:

```
Noise Power Spectral Density (low-noise mode): 300 µg/√Hz
```

Integrated over 100 Hz:

```
σ_accel_white = 300e-6 × 9.81 × √100 ≈ 0.0294 m/s²
```

The implementation uses `accel_white_std_ = 0.020 m/s²`, which is a bit optimistic and assumes that WitMotion’s on-board filtering effectively reduces the bandwidth. This is again a reasonable guess, not a measured value.

---

## 2. 2D LiDAR — SLAMTEC RPLiDAR A2M12 (`noisy_lidar.cpp`)

### 2.1 Datasheet reference

SLAMTEC RPLIDAR A2 specifications.

### 2.2 Noise model

Triangulation-based LiDAR range error grows with distance. A range-proportional noise model is consistent with both the physics and the specification.

From the A2M12 spec:

| Distance range | Accuracy (typical) |
|----------------|--------------------|
| ≤ 3 m          | ±1% of range       |
| 3–5 m          | ±2% of range       |
| 5–25 m         | ±2.5% of range     |

The model uses a piecewise range-proportional σ:

```
σ(r) = max(min_noise, rel_noise(r) × r)

rel_noise(r):
  r <= 3m      -> 1.0%
  3m < r <= 5m -> 2.0%
  r > 5m       -> 2.5%

min_noise = 0.003 m
```

The spec talks about accuracy bounds, not Gaussian standard deviation. Treating these percentages as 1σ is a simplification that keeps the model close to the usual beam model in probabilistic robotics.

---

## 3. 3D LiDAR — Livox Mid-360 (`noisy_livox_mid360.cpp`)

### 3.1 Datasheet reference

Livox Mid-360 technical specifications.

### 3.2 Noise model

Time-of-flight and phase-shift LiDAR errors mainly affect range, not angle. The node applies noise along the line of sight:

```
ratio = 1 + Δr/r
(x', y', z') = ratio × (x, y, z)
```

The datasheet gives range precision values that degrade with distance. The implementation approximates this with a simple two-region model:

```
σ(r) = max(min_noise, rel_noise(r) × r)

rel_noise(r):
  r <= 6m -> 0.4%
  r > 6m  -> 0.7%

min_noise = 0.002 m
```

Angular noise is not modelled explicitly. At the distances of interest, its effect is comparable to the radial noise already applied.

---

## 4. 3D LiDAR — Livox Mid-70 (`noisy_livox_mid70.cpp`)

The Mid-70 is a longer-range ToF sensor. Its architecture is similar to the Mid-360 but with better precision at long range.

Datasheet: Livox Mid-70 product page.

The model is:

```
σ(r) = max(min_noise, rel_noise(r) × r)

rel_noise(r):
  r <= 20m -> 0.1%
  r > 20m  -> 0.3%

min_noise = 0.002 m
```

This follows the typical precision ranges in the Livox material. There is no separate calibration for this node.

---

## 5. Wheel odometry — AgileX Tracer 2.0 (`noisy_odom.cpp`)

### 5.1 Context and limitations

The Tracer 2.0 uses brushless DC motors with integrated encoders. Public documentation on encoder resolution and systematic odometry errors is limited.

The noise parameters here are guided by:

- the behaviour of the real platform (especially during in-place rotations);
- classical odometry error models in the literature.

They should be read as reasonable defaults, not as measured values.

### 5.2 Error model structure

The model follows the structure from Borenstein and Feng (1996): the main systematic errors for a differential-drive robot are encoder imbalance and wheel diameter uncertainty, which show up as heading drift that grows with distance. Random components are modelled as proportional to the commanded velocity.

### 5.3 Linear slip (`lin_noise_ratio = 0.02`)

A 2% linear velocity noise is consistent with published values for indoor AGVs with rubber wheels on smooth floors. It models encoder quantization and small amounts of slip.

### 5.4 Angular noise (`ang_noise_ratio = 0.08`)

This value is not derived from a datasheet. It comes from observation: on the physical Tracer 2.0, commanded 90° rotations often end with a visible heading error of several degrees. An 8% ratio reproduces this behaviour in simulation and keeps heading clearly less reliable than linear motion.

A more careful treatment would command several known rotation angles, record `/odom`, and compute statistics. That work is left as future hardware validation.

### 5.5 Yaw random walk (`yaw_drift_rate = 0.005 rad/m`)

This parameter models unbounded heading drift with distance. At 0.005 rad/m, 10 m of travel produce roughly ±0.05 rad (about ±2.9°) of additional uncertainty. This is mainly there to stress-test loop closure and IMU fusion.

### 5.6 Covariance matrices

| Field | Value | Comment |
|-------|-------|---------|
| `pose.yaw` | 0.08 rad² | high uncertainty: EKF should prefer IMU heading |
| `pose.z`, `roll`, `pitch` | 1e6 | unmeasured DOFs on a planar robot |
| `twist.vy` | 1e-4 m²/s² | near-zero, enforces non-holonomic behaviour |
| `twist.vx` | 0.001 m²/s² | encoder-based linear velocity |
| `twist.wz` | 0.05 rad²/s² | less precise angular rate |

---

## 6. RGB-D camera — Intel RealSense D455

The RealSense D455 is only used for visual loop closure in the SLAM stack. There is no dedicated C++ noise node. Depth noise is added directly in the Gazebo sensor plugin in `urdf/sensors/camera.xacro`:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.003</stddev>
</noise>
```

The camera does not drive incremental pose in this setup. Its main role is place recognition (detecting that the robot has been in a location before), so this simple depth noise is enough for now.

By contrast, IMU, LiDAR and odometry errors integrate over time and are the main source of drift. That is why they get dedicated C++ nodes and more detailed models.

---

## 7. Summary table

| Node                  | Sensor             | Model                               | Main source                             | Status |
|-----------------------|--------------------|-------------------------------------|-----------------------------------------|--------|
| `noisy_imu.cpp`       | WT901C / MPU-9250  | Gauss-Markov + white + quantization | MPU-9250 datasheet, Woodman 2007        | partial calibration |
| `noisy_lidar.cpp`     | RPLiDAR A2M12      | Piecewise proportional Gaussian     | RPLIDAR A2 spec, beam model literature  | uses accuracy as σ |
| `noisy_livox_mid360`  | Livox Mid-360      | Piecewise radial Gaussian           | Mid-360 datasheet, ICP error studies    | close to spec |
| `noisy_livox_mid70`   | Livox Mid-70       | Piecewise radial Gaussian           | Mid-70 product information               | approximate |
| `noisy_odom.cpp`      | Tracer 2 encoders  | Slip + yaw random walk              | Borenstein 1996, Siegwart 2011, tests   | estimates only |

---

## 8. Future work

1. Record a long static bag for the WT901C and run Allan variance to identify ARW, bias instability and correlation time on the real unit.
2. Characterise odometry angular error by commanding several rotation angles, logging `/odom`, and computing the difference.
3. Revisit the G-sensitivity coefficient using the MPU-9250 cross-axis spec or direct measurement.
4. Add reflectivity-dependent LiDAR noise for typical hospital materials (linoleum, glass, metal doors).
5. Refactor Mid-360 and Mid-70 into a single node with a sensor type parameter.

---

## 9. References

The list below is not exhaustive, but it covers the main sources used when choosing models and parameters.

- O. J. Woodman, "An Introduction to Inertial Navigation", University of Cambridge Technical Report UCAM-CL-TR-696, 2007.
- IEEE Std 952-1997, "IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros".
- S. Thrun, W. Burgard, D. Fox, "Probabilistic Robotics", MIT Press, 2005.
- F. Pomerleau, F. Colas, R. Siegwart, S. Magnenat, "Comparing ICP Variants on Real-World Data Sets", Autonomous Robots, 34(3), 133–148, 2013.
- J. Borenstein, L. Feng, "Measurement and Correction of Systematic Odometry Errors in Mobile Robots", IEEE Transactions on Robotics and Automation, 12(5), 869–880, 1996.
- R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, "Introduction to Autonomous Mobile Robots", 2nd ed., MIT Press, 2011.

