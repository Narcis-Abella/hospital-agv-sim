# Sensor Noise Models — Research & Parameter Justification

> **Validation status:** All parameters in this document are **estimates derived from vendor datasheets and field observation**. No systematic calibration campaign has been performed on the physical platform. The values are designed to be plausible and conservative, not validated against recorded ground truth. A real-hardware measurement pass (Allan variance on the physical IMU, rosbag-based odometry drift characterisation) is explicitly listed as future work.

This document consolidates the scientific rationale behind every noise model implemented in `src/`. Its purpose is twofold: to make the design decisions transparent and reproducible, and to position the repository as a metrologically grounded simulation framework rather than a collection of ad-hoc noise injections.

---

## 1. IMU — WitMotion WT901C-TTL (`noisy_imu.cpp`)

### 1.1 Sensor identification

The WT901C-TTL is a consumer-grade AHRS module built around the **InvenSense MPU-9250** MEMS die. The raw inertial measurements (gyroscope + accelerometer) come directly from the MPU-9250; the onboard microcontroller applies WitMotion's Kalman filter to produce fused angle outputs. In this pipeline the raw IMU topic is consumed before any WitMotion fusion, so the relevant noise specifications are those of the bare MPU-9250.

**Datasheet reference:** [InvenSense MPU-9250 Product Specification Rev 1.1 (PS-MPU-9250A-01)](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)

### 1.2 Gyroscope model

The node implements a **first-order Gauss-Markov bias process** superimposed on white measurement noise, plus G-sensitivity cross-coupling and ADC quantization. The Gauss-Markov process is the standard stochastic model for MEMS gyroscope in-run bias, as established in [Woodman 2007] and formalised for Allan variance identification in [IEEE Std 952-1997].

#### White noise (Angle Random Walk)

The MPU-9250 datasheet (Table 1, §3.1) reports:

```
Rate Noise Spectral Density: 0.01 °/s/√Hz  (typical, DLPF enabled)
```

Converting to rad/s/√Hz and integrating over a 100 Hz bandwidth:

```
PSD = 0.01 × (π/180) ≈ 1.745 × 10⁻⁴ rad/s/√Hz
σ_white = PSD × √(BW) = 1.745e-4 × √100 ≈ 0.00175 rad/s
```

This directly matches `gyro_white_std_ = 0.00175` in the code.

#### Bias instability (Gauss-Markov process)

The first-order Gauss-Markov process is the canonical model for MEMS IMU in-run bias drift [Woodman 2007, §3.2]. It captures two key physical properties: the bias is correlated in time (it does not jump randomly) and it eventually decorrelates (unlike a pure random walk, it is mean-reverting). The discrete-time recursion is:

```
b[k] = exp(-dt/τ) · b[k-1] + w[k],   w ~ N(0, σ_b · √(1 - exp(-2dt/τ)))
```

The MPU-9250 datasheet does not publish a bias instability figure directly; it reports **Initial ZRO Tolerance ±5 °/s** and **ZRO Variation Over Temperature ±30 °/s** — worst-case bounds, not in-run instability. For MEMS gyroscopes of this class, Allan variance analyses consistently report in-run bias instability in the range **0.01–0.05 °/s (1σ)** [IEEE Std 952-1997]. The value used:

```
gyro_bias_std_ = 0.0003 rad/s  ≈  0.017 °/s
```

The **correlation time τ = 400 s** is a literature-informed assumption for MEMS gyroscopes under stable thermal conditions [Woodman 2007].

> **Validation gap:** Bias instability and τ have not been characterised via Allan variance on the physical WT901C unit. A static rosbag of ≥30 min at rest would allow an Allan deviation plot to validate or correct these values [IEEE Std 952-1997].

#### G-sensitivity

The MPU-9250 datasheet reports **Cross-Axis Sensitivity ±2%** (Table 1). The implementation uses:

```
G_sensitivity_ = gyro_white_std_ / 9.81  ≈  1.78 × 10⁻⁴ (rad/s)/(m/s²)
```

> **Note:** This derivation (ARW / g) is not a standard way to express G-sensitivity. The correct parameter would be derived from the ±2% cross-axis figure or measured under controlled vibration. The current value is numerically small and produces a negligible effect in practice, but it is **not metrologically justified**. Flagged as known approximation; functional impact is minimal.

#### ADC quantization

The MPU-9250 uses **16-bit ADCs**. At FS_SEL=3 (±2000 °/s) and AFS_SEL=3 (±16g), matching the WT901C defaults:

```
gyro_LSB  = (2 × 2000°/s × π/180) / 65536 = 1.065 × 10⁻³ rad/s per LSB
accel_LSB = (2 × 16 × 9.81)       / 65536 = 4.789 × 10⁻³ m/s² per LSB
```

Both values are computed from sensitivity scale factors in Tables 1 and 2 of the MPU-9250 datasheet.

#### Accelerometer white noise

The MPU-9250 datasheet (Table 2, §3.2) reports:

```
Noise Power Spectral Density (low-noise mode): 300 µg/√Hz
```

Converting and integrating over 100 Hz bandwidth:

```
σ_accel_white = 300e-6 × 9.81 × √100 ≈ 0.0294 m/s²
```

The code uses `accel_white_std_ = 0.020 m/s²`, slightly optimistic relative to the raw PSD — implicitly assuming the WitMotion onboard filter reduces effective bandwidth. Acknowledged approximation.

---

## 2. 2D LiDAR — SLAMTEC RPLiDAR A2M12 (`noisy_lidar.cpp`)

### 2.1 Datasheet reference

**[SLAMTEC RPLIDAR A2 Specifications](https://www.slamtec.com/en/Lidar/A2Spec)**

### 2.2 Noise model

Triangulation-based LiDAR range error is physically dominated by geometric uncertainty that grows with distance, justifying a range-proportional noise model [Thrun, Burgard & Fox 2005, Ch. 6.3]. The RPLiDAR A2M12 specification confirms this structure:

| Distance range | Accuracy (typical) |
|---|---|
| ≤ 3 m | ±1% of range |
| 3 – 5 m | ±2% of range |
| 5 – 25 m | ±2.5% of range |

The model used:

```
σ(r) = max(min_noise, rel_noise × r)

rel_noise = 0.01  (1% — conservative bound valid for ≤3 m)
min_noise  = 0.003 m  (3 mm floor for very short ranges)
```

Using 1% across the full range is conservative: at distances >3 m the actual accuracy is up to 2.5%, so the simulation slightly underestimates real noise at medium and long range.

> **Note:** The A2M12 specification uses the term "accuracy" (systematic bound), not 1σ standard deviation. Treating it as a Gaussian σ is a modelling simplification consistent with the beam model of [Thrun et al. 2005]. A more rigorous model would separate systematic and random components.

---

## 3. 3D LiDAR — Livox Mid-360 (`noisy_livox_mid360.cpp`)

### 3.1 Datasheet reference

**[Livox Mid-360 Technical Specifications](https://www.livoxtech.com/mid-360/specs)**

### 3.2 Noise model

ToF and phase-shift LiDAR measurement errors manifest primarily as **radial range perturbations**, not lateral angular errors [Pomerleau et al. 2013, §2]. The noise is therefore applied along the line-of-sight, preserving azimuth and elevation:

```
ratio = 1 + Δr/r
(x', y', z') = ratio × (x, y, z)
```

The Livox Mid-360 datasheet provides a direct 1σ spec:

```
Range Precision (1σ):
  ≤ 2 cm  @ 10 m  (80% reflectivity, 25°C)
  ≤ 3 cm  @ 0.2 m (80% reflectivity, 25°C)
```

Parameters:

```
rel_noise = 0.005  (0.5% → σ ≈ 2 cm at 4 m — matches ≤2 cm spec)
min_noise = 0.002  (2 mm floor)
```

The angular precision spec (< 0.15° 1σ) is not modelled; at 1–10 m it contributes < 2.6 cm lateral error, comparable to the range noise already applied.

---

## 4. 3D LiDAR — Livox Mid-70 (`noisy_livox_mid70.cpp`)

The Mid-70 shares the same Livox ToF architecture and reports the same **±2 cm ranging accuracy** at similar conditions.

**[Livox Mid-70 Product Page / Specifications](https://www.livoxtech.com/mid-70)**

Parameters are identical to the Mid-360 node (`rel_noise = 0.005`, `min_noise = 0.002`).

> **Code note:** `noisy_livox_mid70.cpp` and `noisy_livox_mid360.cpp` share identical logic with different node names and topic routes. Consolidating them into a single parametric node is a known refactoring opportunity.

---

## 5. Wheel Odometry — AgileX Tracer 2.0 (`noisy_odom.cpp`)

### 5.1 Context and limitations

The AgileX Tracer 2.0 uses brushless DC motors with integrated encoders. No public datasheet for encoder resolution or systematic odometry error characterisation has been located. The noise parameters are **empirically motivated**, not datasheet-derived.

**[AgileX Tracer 2.0 Specifications (Trossen Robotics)](https://docs.trossenrobotics.com/agilex_tracer_docs/specifications.html)**

### 5.2 Error model structure

The odometry noise model follows the structure established by [Borenstein & Feng 1996] for differential-drive robots: the dominant systematic error sources are encoder imbalance and wheel diameter uncertainty, which manifest as heading drift proportional to the distance travelled. The stochastic component is modelled as proportional to the commanded velocity [Siegwart, Nourbakhsh & Scaramuzza 2011, Ch. 5.2].

### 5.3 Linear slip (`lin_noise_ratio = 0.02`, i.e. 2%)

A 2% linear velocity noise is consistent with the error magnitudes characterised by [Borenstein & Feng 1996] for rubber-wheeled indoor AGVs on smooth floors. It models encoder quantization and minor wheel slip.

### 5.4 Angular noise (`ang_noise_ratio = 0.08`, i.e. 8%)

> **Important caveat:** This value is **not derived from a datasheet** and has not been validated with a structured measurement protocol. It is based on direct observation of the physical Tracer 2.0: the robot exhibits a **visibly large yaw error after in-place rotations** — 90° commanded turns produce visible heading offsets of several degrees. The 8% figure is a conservative estimate intended to reproduce this behaviour in simulation.
>
> The high angular-to-linear ratio (8% vs. 2%) is physically consistent with the findings of [Borenstein & Feng 1996]: small encoder imbalances between left and right wheels amplify into large yaw divergence over time, making heading the dominant dead-reckoning error source in differential-drive systems.
>
> **Pending validation:** Commanding known rotation angles (90°, 180°, 360°) via the ROS 2 control interface, recording `/odom`, and computing actual vs. commanded yaw over multiple trials. Explicitly deferred to the hardware validation phase (Plan Phase 3, post-exams).

### 5.5 Yaw random walk (`yaw_drift_rate = 0.005 rad/m`)

This models the **systematic, unbounded heading drift** accumulating with distance — the random walk component identified by [Borenstein & Feng 1996] as distinct from per-step stochastic noise. At 0.005 rad/m, after 10 m the expected drift is ±0.05 rad (≈ ±2.9°). The drift is intentionally unbounded to stress-test SLAM loop-closure and IMU fusion.

### 5.6 Covariance matrices

| Field | Value | Rationale |
|---|---|---|
| `pose.yaw` | 0.08 rad² | High uncertainty — EKF should weight IMU heading over odometry |
| `pose.z`, `roll`, `pitch` | 1×10⁶ | Unmeasured DOFs (planar robot) |
| `twist.vy` | 1×10⁻⁴ m²/s² | Near-zero; enforces non-holonomic constraint |
| `twist.vx` | 0.001 m²/s² | Reliable encoder measurement |
| `twist.wz` | 0.05 rad²/s² | Imprecise differential drive |

---

## 6. Summary Table

| Node | Sensor | Model | Primary source | Validation status |
|---|---|---|---|---|
| `noisy_imu.cpp` | WT901C / MPU-9250 | Gauss-Markov bias + white noise + G-sensitivity + ADC quant. | MPU-9250 datasheet §3.1–3.2; [Woodman 2007]; [IEEE 952-1997] | Partial — white noise and ADC from datasheet; bias τ and instability from literature |
| `noisy_lidar.cpp` | RPLiDAR A2M12 | Proportional Gaussian `σ = max(min, rel·r)` | SLAMTEC A2 spec; [Thrun et al. 2005] | Conservative estimate — accuracy treated as 1σ (simplification) |
| `noisy_livox_mid360.cpp` | Livox Mid-360 | Radial Gaussian `σ = max(min, rel·r)` | Livox Mid-360 datasheet (1σ); [Pomerleau et al. 2013] | Direct datasheet match |
| `noisy_livox_mid70.cpp` | Livox Mid-70 | Radial Gaussian (same as Mid-360) | Livox Mid-70 product page | Same architecture, assumed same accuracy |
| `noisy_odom.cpp` | Tracer 2.0 encoders | Proportional slip + yaw random walk + systematic drift | [Borenstein & Feng 1996]; [Siegwart et al. 2011]; field observation | **Linear: literature-consistent; Angular 8%: empirical observation, unvalidated — pending hardware measurement** |

---

## 7. Future Work

1. **Allan variance of physical WT901C:** Record ≥30 min static rosbag on the Jetson Orin NX, run Allan deviation analysis [IEEE Std 952-1997] to characterise `gyro_bias_tau_`, `gyro_bias_std_`, and confirm `gyro_white_std_`.
2. **Odometry angular error characterisation:** Command known rotation angles on the Tracer 2.0, record rosbags, compute yaw error statistics [Borenstein & Feng 1996]. Replace the empirical 8% with a measured value.
3. **G-sensitivity correction:** Derive the coefficient correctly from the MPU-9250 ±2% cross-axis sensitivity spec, or measure it on the physical unit.
4. **LiDAR reflectivity dependence:** Both Livox and RPLiDAR accuracy degrade on low-reflectivity surfaces. A reflectivity-modulated σ would improve fidelity for hospital environments.
5. **Mid-70 / Mid-360 refactor:** Consolidate into a single `noisy_livox.cpp` node with a `sensor_model` parameter.

---

## 8. Methodological References

The following references justify the choice of stochastic models, not just the numerical parameter values.

### [Woodman 2007]
O. J. Woodman. *An Introduction to Inertial Navigation*. Technical Report UCAM-CL-TR-696, University of Cambridge Computer Laboratory, 2007.
→ Establishes the first-order Gauss-Markov process as the standard model for MEMS IMU in-run bias drift (§3.2). Defines the relationship between Allan variance parameters (ARW, bias instability, rate random walk) and the continuous-time stochastic differential equations used in the noise nodes.
**URL:** https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf

### [IEEE Std 952-1997]
IEEE Standard Specification Format Guide and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros. IEEE Std 952-1997 (R2008). IEEE, 1997.
→ The normative standard defining Allan variance as the method for identifying gyroscope noise coefficients (ARW, bias instability, τ). Provides the mathematical framework linking the Allan deviation plot to the parameters of the Gauss-Markov model.
**URL:** https://ieeexplore.ieee.org/document/660628

### [Thrun, Burgard & Fox 2005]
S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics*. MIT Press, 2005. ISBN 978-0-262-20162-9.
→ Chapter 6.3 formalises the beam model of range finders. Derives the range-proportional Gaussian as the dominant noise term for correct measurements, alongside mixture components for unexpected obstacles and sensor failures. Justifies modelling LiDAR measurement noise as `σ(r) = f(r)` rather than a fixed additive term.
**URL:** http://robots.stanford.edu/probabilistic-robotics/

### [Pomerleau, Colas & Siegwart 2013]
F. Pomerleau, F. Colas, R. Siegwart, S. Magnenat. *Comparing ICP Variants on Real-World Data Sets: Open-source library and experimental protocol*. Autonomous Robots, 34(3), 133–148, 2013.
→ Section 2 analyses the structure of 3D LiDAR point cloud errors in practice. Confirms that ToF and phase-shift LiDAR errors are predominantly radial (range direction), not lateral — justifying the radial perturbation model applied in `noisy_livox_mid360.cpp` and `noisy_livox_mid70.cpp`.
**DOI:** https://doi.org/10.1007/s10514-013-9327-2

### [Borenstein & Feng 1996]
J. Borenstein, L. Feng. *Measurement and Correction of Systematic Odometry Errors in Mobile Robots*. IEEE Transactions on Robotics and Automation, 12(5), 869–880, 1996.
→ The foundational empirical study of differential-drive odometry error. Identifies encoder imbalance and wheel diameter uncertainty as the dominant error sources, quantifies their effect as a heading drift proportional to distance, and establishes that angular error significantly exceeds linear error — the physical basis for `ang_noise_ratio` (8%) >> `lin_noise_ratio` (2%) in `noisy_odom.cpp`.
**DOI:** https://doi.org/10.1109/70.544770

### [Siegwart, Nourbakhsh & Scaramuzza 2011]
R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza. *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press, 2011. ISBN 978-0-262-01535-6.
→ Chapter 5.2 formalises the probabilistic odometry model for differential-drive robots with velocity-proportional noise components. Provides the theoretical framework for modelling slip as `noise ~ N(0, (ratio × |v|)²)`, directly matching the implementation in `noisy_odom.cpp`.
**URL:** https://mitpress.mit.edu/9780262015356/introduction-to-autonomous-mobile-robots/
