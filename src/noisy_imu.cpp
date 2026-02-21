#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>
#include <cmath>

class ImuRealisticNode : public rclcpp::Node {
public:
    ImuRealisticNode() : Node("imu_realistic_model") {
        // Parámetros
        this->declare_parameter("update_rate", 100.0);
        double rate = this->get_parameter("update_rate").as_double();
        dt_ = 1.0 / rate;

        // Parametros Gyro (WT901C)
        gyro_white_std_ = 0.00175; // rad/s
        gyro_bias_std_ = 0.0003;
        gyro_bias_tau_ = 400.0;
        
        // Parametros Accel
        accel_white_std_ = 0.020; // m/s²
        accel_bias_std_ = 0.010;
        accel_bias_tau_ = 300.0;

        // Sensibilidad G y Cuantización
        double g_sens = (0.00175) / 9.81; 
        G_sensitivity_ = g_sens;
        gyro_lsb_ = (2.0 * (2000.0 * M_PI / 180.0)) / 65536.0;
        accel_lsb_ = (2.0 * 16.0 * 9.81) / 65536.0;

        // Inicializar Gauss-Markov
        alpha_gyro_ = std::exp(-dt_ / gyro_bias_tau_);
        alpha_accel_ = std::exp(-dt_ / accel_bias_tau_);
        
        gyro_drive_std_ = gyro_bias_std_ * std::sqrt(1.0 - alpha_gyro_ * alpha_gyro_);
        accel_drive_std_ = accel_bias_std_ * std::sqrt(1.0 - alpha_accel_ * alpha_accel_);

        // Generador de números aleatorios
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_norm_ = std::normal_distribution<double>(0.0, 1.0);

        // Covarianzas
        double gyro_var = std::pow(gyro_white_std_, 2) + std::pow(gyro_bias_std_, 2);
        double accel_var = std::pow(accel_white_std_, 2) + std::pow(accel_bias_std_, 2);
        double ori_var = std::pow(0.5 * M_PI / 180.0, 2);

        // Rellenar matrices covarianza una sola vez
        std::fill(std::begin(gyro_cov_), std::end(gyro_cov_), 0.0);
        gyro_cov_[0] = gyro_cov_[4] = gyro_cov_[8] = gyro_var;
        
        std::fill(std::begin(accel_cov_), std::end(accel_cov_), 0.0);
        accel_cov_[0] = accel_cov_[4] = accel_cov_[8] = accel_var;

        std::fill(std::begin(ori_cov_), std::end(ori_cov_), 0.0);
        ori_cov_[0] = ori_cov_[4] = ori_cov_[8] = ori_var;

        // Pub/Sub
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_raw", 10, std::bind(&ImuRealisticNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        RCLCPP_INFO(this->get_logger(), "IMU C++ Node Started using logic: Gauss-Markov + Quantization");
    }

private:
    void step_bias() {
        // Gyro Bias Update
        for(int i=0; i<3; i++) {
            gyro_bias_[i] = alpha_gyro_ * gyro_bias_[i] + gyro_drive_std_ * dist_norm_(gen_);
            accel_bias_[i] = alpha_accel_ * accel_bias_[i] + accel_drive_std_ * dist_norm_(gen_);
        }
    }

    double quantize(double val, double lsb) {
        return std::round(val / lsb) * lsb;
    }

    void callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto new_msg = *msg; // Copia
        step_bias();

        // Aplicar bias y G-sensitivity al Gyro
        double gx = msg->angular_velocity.x + gyro_bias_[0] + G_sensitivity_ * msg->linear_acceleration.x;
        double gy = msg->angular_velocity.y + gyro_bias_[1] + G_sensitivity_ * msg->linear_acceleration.y;
        double gz = msg->angular_velocity.z + gyro_bias_[2] + G_sensitivity_ * msg->linear_acceleration.z;

        new_msg.angular_velocity.x = quantize(gx, gyro_lsb_);
        new_msg.angular_velocity.y = quantize(gy, gyro_lsb_);
        new_msg.angular_velocity.z = quantize(gz, gyro_lsb_);

        // Aplicar bias al Accel
        double ax = msg->linear_acceleration.x + accel_bias_[0];
        double ay = msg->linear_acceleration.y + accel_bias_[1];
        double az = msg->linear_acceleration.z + accel_bias_[2];

        new_msg.linear_acceleration.x = quantize(ax, accel_lsb_);
        new_msg.linear_acceleration.y = quantize(ay, accel_lsb_);
        new_msg.linear_acceleration.z = quantize(az, accel_lsb_);

        // Asignar covarianzas
        new_msg.angular_velocity_covariance = gyro_cov_;
        new_msg.linear_acceleration_covariance = accel_cov_;
        new_msg.orientation_covariance = ori_cov_;

        pub_->publish(new_msg);
    }

    // Variables
    double dt_, gyro_white_std_, gyro_bias_std_, gyro_bias_tau_;
    double accel_white_std_, accel_bias_std_, accel_bias_tau_;
    double G_sensitivity_, gyro_lsb_, accel_lsb_;
    
    double alpha_gyro_, alpha_accel_, gyro_drive_std_, accel_drive_std_;
    
    double gyro_bias_[3] = {0.0, 0.0, 0.0};
    double accel_bias_[3] = {0.0, 0.0, 0.0};
    
    std::array<double, 9> gyro_cov_, accel_cov_, ori_cov_;

    std::mt19937 gen_;
    std::normal_distribution<double> dist_norm_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRealisticNode>());
    rclcpp::shutdown();
    return 0;
}