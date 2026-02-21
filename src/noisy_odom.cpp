#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <cmath>
#include <array>

class NoisyOdomNode : public rclcpp::Node {
public:
    NoisyOdomNode() : Node("noisy_odom_node") {
        // ── Parámetros ──
        this->declare_parameter("lin_noise_ratio", 0.02); // 2% de Vx
        this->declare_parameter("ang_noise_ratio", 0.08); // 8% de Wz
        this->declare_parameter("yaw_drift_rate", 0.005); // rad/m

        lin_ratio_ = this->get_parameter("lin_noise_ratio").as_double();
        ang_ratio_ = this->get_parameter("ang_noise_ratio").as_double();
        drift_rate_ = this->get_parameter("yaw_drift_rate").as_double();

        // ── Inicialización de Covarianzas (Copia exacta del Python) ──
        // Pose: x, y, z, roll, pitch, yaw
        pose_cov_.fill(0.0);
        pose_cov_[0]  = 0.005; // x
        pose_cov_[7]  = 0.005; // y
        pose_cov_[14] = 1e6;   // z (no medido)
        pose_cov_[21] = 1e6;   // roll
        pose_cov_[28] = 1e6;   // pitch
        pose_cov_[35] = 0.08;  // yaw (Alto para que EKF confíe en IMU)

        // Twist: vx, vy, vz, wx, wy, wz
        twist_cov_.fill(0.0);
        twist_cov_[0]  = 0.001;  // vx (fiable)
        twist_cov_[7]  = 0.0001; // vy (no-holonómico)
        twist_cov_[14] = 1e6;    // vz
        twist_cov_[21] = 1e6;    // wx
        twist_cov_[28] = 1e6;    // wy
        twist_cov_[35] = 0.05;   // wz (impreciso)

        // ── Random & State ──
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_norm_ = std::normal_distribution<double>(0.0, 1.0);

        yaw_drift_ = 0.0;
        first_msg_ = true;

        // ── Pub/Sub ──
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10, std::bind(&NoisyOdomNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(), "Odom C++ Node Started. Drift rate: %.3f rad/m", drift_rate_);
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto new_msg = *msg; // Copia del mensaje

        double vx = new_msg.twist.twist.linear.x;
        double wz = new_msg.twist.twist.angular.z;

        // 1. Ruido Lineal (Slippage)
        if (std::abs(vx) > 0.001) {
            double sigma = std::abs(vx) * lin_ratio_;
            new_msg.twist.twist.linear.x += sigma * dist_norm_(gen_);
        }

        // 2. Ruido Angular (Error diferencial)
        if (std::abs(wz) > 0.001) {
            double sigma = std::abs(wz) * ang_ratio_;
            new_msg.twist.twist.angular.z += sigma * dist_norm_(gen_);
        }

        // 3. Deriva Acumulada de Yaw (Systematic Drift)
        double pos_x = new_msg.pose.pose.position.x;
        double pos_y = new_msg.pose.pose.position.y;

        if (!first_msg_) {
            double dx = pos_x - last_x_;
            double dy = pos_y - last_y_;
            double dist_step = std::sqrt(dx*dx + dy*dy);

            if (dist_step > 0.001) {
                double sigma_drift = drift_rate_ * dist_step;
                yaw_drift_ += sigma_drift * dist_norm_(gen_);
            }
        } else {
            first_msg_ = false;
        }
        
        last_x_ = pos_x;
        last_y_ = pos_y;

        // 4. Aplicar Deriva al Quaternion
        // Extraer Yaw actual
        double qx = new_msg.pose.pose.orientation.x;
        double qy = new_msg.pose.pose.orientation.y;
        double qz = new_msg.pose.pose.orientation.z;
        double qw = new_msg.pose.pose.orientation.w;

        // Yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double current_yaw = std::atan2(siny_cosp, cosy_cosp);

        double noisy_yaw = current_yaw + yaw_drift_;

        // Reconstruir Quaternion (Roll=0, Pitch=0)
        new_msg.pose.pose.orientation.x = 0.0;
        new_msg.pose.pose.orientation.y = 0.0;
        new_msg.pose.pose.orientation.z = std::sin(noisy_yaw / 2.0);
        new_msg.pose.pose.orientation.w = std::cos(noisy_yaw / 2.0);

        // 5. Asignar Covarianzas
        new_msg.pose.covariance = pose_cov_;
        new_msg.twist.covariance = twist_cov_;

        pub_->publish(new_msg);
    }

    // Variables de configuración
    double lin_ratio_, ang_ratio_, drift_rate_;
    
    // Estado
    double yaw_drift_;
    double last_x_, last_y_;
    bool first_msg_;

    // Generador Aleatorio
    std::mt19937 gen_;
    std::normal_distribution<double> dist_norm_;

    // Covarianzas fijas
    std::array<double, 36> pose_cov_;
    std::array<double, 36> twist_cov_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyOdomNode>());
    rclcpp::shutdown();
    return 0;
}