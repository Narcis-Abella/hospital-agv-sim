#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>
#include <cmath>

class NoisyImuNode : public rclcpp::Node {
public:
    NoisyImuNode() : Node("noisy_imu_node") {
        this->declare_parameter("update_rate", 100.0);
        dt_ = 1.0 / this->get_parameter("update_rate").as_double();

        // WT901C gyroscope parameters (Allan variance / datasheet)
        gyro_white_std_ = 0.00175;  // rad/s  — angle random walk (ARW)
        gyro_bias_std_  = 0.0003;   // rad/s  — bias instability (1σ)
        gyro_bias_tau_  = 400.0;    // s      — Gauss-Markov correlation time

        // WT901C accelerometer parameters
        accel_white_std_ = 0.020;   // m/s²   — velocity random walk (VRW)
        accel_bias_std_  = 0.010;   // m/s²   — bias instability (1σ)
        accel_bias_tau_  = 300.0;   // s      — Gauss-Markov correlation time

        // G-sensitivity: linear-accel-to-gyro cross-coupling.
        // Approximated from WT901C cross-axis spec. Units: (rad/s)/(m/s²).
        G_sensitivity_ = gyro_white_std_ / 9.81;

        // ADC quantization step sizes (16-bit, symmetric ±FS range)
        gyro_lsb_  = (2.0 * (2000.0 * M_PI / 180.0)) / 65536.0;  // rad/s per LSB
        accel_lsb_ = (2.0 * 16.0 * 9.81) / 65536.0;               // m/s² per LSB

        // Static covariance matrices: total variance = white_noise² + bias²
        // Row-major, diagonal 3×3 — passed directly to EKF/SLAM consumers.
        double gyro_var  = std::pow(gyro_white_std_, 2) + std::pow(gyro_bias_std_, 2);
        double accel_var = std::pow(accel_white_std_, 2) + std::pow(accel_bias_std_, 2);
        double ori_var   = std::pow(0.5 * M_PI / 180.0, 2);  // 0.5 deg orientation uncertainty

        std::fill(gyro_cov_.begin(),  gyro_cov_.end(),  0.0);
        std::fill(accel_cov_.begin(), accel_cov_.end(), 0.0);
        std::fill(ori_cov_.begin(),   ori_cov_.end(),   0.0);

        gyro_cov_[0]  = gyro_cov_[4]  = gyro_cov_[8]  = gyro_var;
        accel_cov_[0] = accel_cov_[4] = accel_cov_[8] = accel_var;
        ori_cov_[0]   = ori_cov_[4]   = ori_cov_[8]   = ori_var;

        gen_       = std::mt19937(std::random_device{}());
        dist_norm_ = std::normal_distribution<double>(0.0, 1.0);

        // Reliable QoS: matches parameter_bridge default output.
        auto qos = rclcpp::QoS(10);
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_raw", qos,
            std::bind(&NoisyImuNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);

        RCLCPP_INFO(this->get_logger(),
            "IMU noise node started — model: Gauss-Markov bias + white noise "
            "+ G-sensitivity + ADC quantization");
    }

private:
    // Advance the first-order Gauss-Markov bias state.
    // α = exp(-dt/τ), process noise std = σ_bias * sqrt(1 - α²).
    void step_bias(double dt) {
        const double ag = std::exp(-dt / gyro_bias_tau_);
        const double aa = std::exp(-dt / accel_bias_tau_);
        const double gd = gyro_bias_std_  * std::sqrt(1.0 - ag * ag);
        const double ad = accel_bias_std_ * std::sqrt(1.0 - aa * aa);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_[i]  = ag * gyro_bias_[i]  + gd * dist_norm_(gen_);
            accel_bias_[i] = aa * accel_bias_[i] + ad * dist_norm_(gen_);
        }
    }

    // Round val to the nearest ADC quantization step.
    static double quantize(double val, double lsb) {
        return std::round(val / lsb) * lsb;
    }

    void callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Use actual inter-message dt for correct Gauss-Markov evolution.
        double dt = dt_;
        if (!first_msg_) {
            double dt_stamp = (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds();
            if (dt_stamp > 0.0 && dt_stamp < 1.0) dt = dt_stamp;
        }
        first_msg_ = false;
        last_stamp_ = rclcpp::Time(msg->header.stamp);

        step_bias(dt);

        auto out = *msg;

        // Gyro: white noise + Gauss-Markov bias + G-sensitivity + quantization
        out.angular_velocity.x = quantize(
            msg->angular_velocity.x
            + gyro_white_std_ * dist_norm_(gen_)
            + gyro_bias_[0]
            + G_sensitivity_ * msg->linear_acceleration.x, gyro_lsb_);
        out.angular_velocity.y = quantize(
            msg->angular_velocity.y
            + gyro_white_std_ * dist_norm_(gen_)
            + gyro_bias_[1]
            + G_sensitivity_ * msg->linear_acceleration.y, gyro_lsb_);
        out.angular_velocity.z = quantize(
            msg->angular_velocity.z
            + gyro_white_std_ * dist_norm_(gen_)
            + gyro_bias_[2]
            + G_sensitivity_ * msg->linear_acceleration.z, gyro_lsb_);

        // Accel: white noise + Gauss-Markov bias + quantization
        out.linear_acceleration.x = quantize(
            msg->linear_acceleration.x
            + accel_white_std_ * dist_norm_(gen_)
            + accel_bias_[0], accel_lsb_);
        out.linear_acceleration.y = quantize(
            msg->linear_acceleration.y
            + accel_white_std_ * dist_norm_(gen_)
            + accel_bias_[1], accel_lsb_);
        out.linear_acceleration.z = quantize(
            msg->linear_acceleration.z
            + accel_white_std_ * dist_norm_(gen_)
            + accel_bias_[2], accel_lsb_);

        out.angular_velocity_covariance    = gyro_cov_;
        out.linear_acceleration_covariance = accel_cov_;
        out.orientation_covariance         = ori_cov_;

        pub_->publish(out);
    }

    // Sensor model parameters
    double dt_;
    double gyro_white_std_, gyro_bias_std_, gyro_bias_tau_;
    double accel_white_std_, accel_bias_std_, accel_bias_tau_;
    double G_sensitivity_, gyro_lsb_, accel_lsb_;

    // Gauss-Markov bias state
    double gyro_bias_[3]  = {0.0, 0.0, 0.0};
    double accel_bias_[3] = {0.0, 0.0, 0.0};

    // Static covariance matrices (3×3 diagonal, row-major)
    std::array<double, 9> gyro_cov_, accel_cov_, ori_cov_;

    // RNG
    std::mt19937 gen_;
    std::normal_distribution<double> dist_norm_;

    // Timestamp tracking for real dt computation
    bool first_msg_ = true;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyImuNode>());
    rclcpp::shutdown();
    return 0;
}
