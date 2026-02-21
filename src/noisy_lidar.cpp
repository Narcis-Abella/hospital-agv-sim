#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>
#include <cmath>
#include <algorithm>

class NoisyLidarNode : public rclcpp::Node {
public:
    NoisyLidarNode() : Node("noisy_lidar_node") {
        this->declare_parameter("rel_noise", 0.01);
        this->declare_parameter("min_noise", 0.003);

        rel_noise_ = this->get_parameter("rel_noise").as_double();
        min_noise_ = this->get_parameter("min_noise").as_double();

        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_norm_ = std::normal_distribution<float>(0.0, 1.0);

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10, std::bind(&NoisyLidarNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        
        RCLCPP_INFO(this->get_logger(), "Lidar C++ Node Started. Rel: %.2f, Min: %.3f", rel_noise_, min_noise_);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto new_msg = *msg; // Copia eficiente
        size_t n = new_msg.ranges.size();

        for(size_t i = 0; i < n; ++i) {
            float r = new_msg.ranges[i];
            
            // Filtro de validez: Inf, NaN o <= 0
            if(!std::isfinite(r) || r <= 0.0f) continue;

            // Calculo de sigma
            float sigma = std::max((float)min_noise_, (float)(rel_noise_ * r));
            
            // Generar ruido
            float noise = sigma * dist_norm_(gen_);
            float noisy_r = r + noise;

            // Clamp (evitar salir del rango físico del sensor)
            if (noisy_r < new_msg.range_min) noisy_r = new_msg.range_min;
            if (noisy_r > new_msg.range_max) noisy_r = new_msg.range_max;

            new_msg.ranges[i] = noisy_r;
        }

        pub_->publish(new_msg);
    }

    double rel_noise_, min_noise_;
    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyLidarNode>());
    rclcpp::shutdown();
    return 0;
}