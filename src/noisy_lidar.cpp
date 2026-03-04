#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>
#include <cmath>
#include <algorithm>

class NoisyLidarNode : public rclcpp::Node {
public:
    NoisyLidarNode() : Node("noisy_lidar_node") {
        this->declare_parameter("min_noise", 0.003f); // minimum noise floor (3 mm)
        min_noise_ = static_cast<float>(this->get_parameter("min_noise").as_double());

        gen_       = std::mt19937(std::random_device{}());
        dist_norm_ = std::normal_distribution<float>(0.0f, 1.0f);

        // Reliable QoS: matches parameter_bridge default output.
        auto qos = rclcpp::QoS(10);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", qos,
            std::bind(&NoisyLidarNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos);

        RCLCPP_INFO(this->get_logger(),
            "2D LiDAR noise node started — model: piecewise range-proportional (RPLiDAR A2M12 spec)");
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto out = *msg;
        const size_t n = out.ranges.size();

        for (size_t i = 0; i < n; ++i) {
            const float r = out.ranges[i];

            // Skip invalid returns (inf, NaN, out-of-range).
            if (!std::isfinite(r) || r <= 0.0f) continue;

            // Piecewise range-proportional Gaussian noise (RPLiDAR A2M12 datasheet).
            // r <= 3m    -> 1%
            // 3m < r <= 5m -> 2%
            // r > 5m     -> 2.5%
            float rel_noise;
            if (r <= 3.0f) {
                rel_noise = 0.01f;
            } else if (r <= 5.0f) {
                rel_noise = 0.02f;
            } else {
                rel_noise = 0.025f;
            }

            const float sigma   = std::max(min_noise_, rel_noise * r);
            const float noisy_r = r + sigma * dist_norm_(gen_);

            out.ranges[i] = std::clamp(noisy_r, out.range_min, out.range_max);
        }

        pub_->publish(out);
    }

    float min_noise_;

    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyLidarNode>());
    rclcpp::shutdown();
    return 0;
}
