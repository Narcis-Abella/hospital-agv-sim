## Pipeline design

Each sensor follows the same pattern:

```text
[Gazebo plugin] --> /gz/topic --> [ros_gz_bridge] --> /sensor_raw --> [C++ noise node] --> /sensor
```

The bridge uses Reliable QoS for all sensor topics (default for `parameter_bridge`).
Noise nodes subscribe with matching Reliable QoS. A Reliable subscriber connected to a
BestEffort publisher will not match in ROS 2 DDS, so it is worth checking with
`ros2 topic info <topic> --verbose` when changing the bridge or QoS.

## IMU model

The WT901C gyroscope bias is modeled as a first-order Gauss-Markov process:

```text
b[k] = α · b[k-1] + w[k]

α = exp(-dt / τ)           # correlation coefficient
τ = 400 s                  # bias time constant (gyro)
w ~ N(0, σ_b · √(1 - α²))  # driving noise
```

The measured angular rate is:

```text
ω_measured = ω_true + η + b + K_g · a

η ~ N(0, σ_ARW²)          # white noise (angle random walk)
b                          # Gauss-Markov bias state
K_g = 0.00175 / 9.81      # G-sensitivity (rad/s per m/s²)
a                          # linear acceleration vector
```

ADC quantization uses full-scale range / 16-bit resolution:

```text
gyro_LSB  = (2 × 2000°/s × π/180) / 65536   # rad/s per LSB
accel_LSB = (2 × 16g × 9.81)      / 65536   # m/s² per LSB
```

## Covariance matrices

Noise nodes fill the covariance matrices on each published message. If covariances are
set to zero or -1, `robot_localization` may either ignore the sensor or behave badly.

The indices below refer to the flat 36-element row-major array of the 6×6 covariance matrix.

| Matrix                | Index      | Value | Comment                          |
|-----------------------|-----------|-------|----------------------------------|
| `pose.covariance`     | [0], [7]  | 0.005 | x, y position uncertainty        |
| `pose.covariance`     | [14,21,28]| 1e6   | z, roll, pitch: unmeasured      |
| `pose.covariance`     | [35]      | 0.08  | yaw: high uncertainty            |
| `twist.covariance`    | [0]       | 0.001 | vx: encoder-based velocity       |
| `twist.covariance`    | [7]       | 0.0001| vy: near-zero (non-holonomic)    |
| `twist.covariance`    | [35]      | 0.05  | ωz: less precise                 |
| `angular_velocity_covariance` | [0,4,8] | σ²_gyro_total | white + bias variance |
| `linear_acceleration_covariance` | [0,4,8] | σ²_accel_total | white + bias variance |

