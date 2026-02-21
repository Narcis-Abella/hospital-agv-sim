# Sensor Noise Pipeline — Architecture Notes

## Motivation

Standard Gazebo noise primitives (`<noise><type>gaussian</type>`) apply uncorrelated white noise uniformly. This is insufficient for:

1. **IMU**: Real MEMS IMUs exhibit correlated bias drift (modeled as Gauss-Markov process), G-sensitivity (accelerometer coupling into gyro), and ADC quantization effects. White noise alone produces over-optimistic covariance estimates.

2. **LiDAR (2D/3D)**: Ranging noise is proportional to distance, not constant. A flat `stddev` underestimates noise at range and overestimates it at close proximity.

3. **Odometry**: Wheel slip is proportional to commanded velocity. Yaw accumulates systematic drift proportional to distance traveled (encoder resolution and differential errors).

## Pipeline Design

Each sensor follows the same pattern:

```
[Gazebo Plugin] → gz topic → [ros_gz_bridge] → /sensor_raw → [C++ Noise Node] → /sensor
```

The bridge uses **Reliable QoS** for all sensor topics. Noise nodes must subscribe with matching QoS — BestEffort subscriptions will silently fail to connect to Reliable publishers in ROS 2 DDS.

## IMU Model Detail

The WT901C gyroscope bias is modeled as a first-order Gauss-Markov process:

```
b[k] = α·b[k-1] + w[k]

where:
  α = exp(-dt / τ)          # correlation coefficient
  τ = 400 s                  # bias time constant (gyro)
  w ~ N(0, σ_b · √(1-α²))   # driving noise
```

G-sensitivity couples linear acceleration into angular velocity output:
```
ω_measured = ω_true + b + K_g · a
```
where `K_g = 0.00175 / 9.81` (from datasheet sensitivity spec).

ADC quantization uses the full-scale range / 16-bit resolution:
```
gyro_LSB  = (2 × 2000°/s × π/180) / 65536
accel_LSB = (2 × 16g × 9.81)      / 65536
```

## Covariance Matrix Assignment

Noise nodes assign covariance matrices to every published message. This is critical for EKF-based fusion (e.g., `robot_localization`): if covariances are zero or `-1` (unknown), the filter either ignores the sensor or behaves numerically unstably.

| Matrix | Key non-zero entries |
|--------|---------------------|
| `pose.covariance[0,7]` | x, y: 0.005 |
| `pose.covariance[35]` | yaw: 0.08 (high → trust IMU over odom for heading) |
| `twist.covariance[0]` | vx: 0.001 |
| `twist.covariance[35]` | ωz: 0.05 |
| `angular_velocity_covariance` | diagonal: σ²_gyro_total |
| `linear_acceleration_covariance` | diagonal: σ²_accel_total |

## Livox Simulation Limitations

The Mid-70 and Mid-360 are simulated using Ignition's `gpu_lidar` plugin with a uniform grid scan pattern. The real sensors use a **non-repetitive Rosette scan pattern** that achieves progressive FOV coverage over time (~1.4 s for full Mid-70 coverage). This temporal accumulation property is not reproduced in simulation.

Impact: SLAM algorithms that rely on Livox's non-repetitive pattern for feature richness (e.g., FAST-LIO2 with its full-frame accumulation) will see a uniform grid instead. For localization benchmarking this is acceptable; for front-end evaluation it is not.

A custom `gpu_livox` Gazebo plugin implementing the Rosette pattern is a planned future extension.
