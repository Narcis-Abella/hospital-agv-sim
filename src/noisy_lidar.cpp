#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <random>
#include <cmath>
#include <algorithm>

class NoisyLidarNode : public rclcpp::Node {
public:
    NoisyLidarNode() : Node("noisy_lidar_node") {
        this->declare_parameter("rel_noise", 0.01);   // fractional range noise (1%)
        this->declare_parameter("min_noise", 0.003f); // minimum noise floor (3 mm)

        rel_noise_ = static_cast<float>(this->get_parameter("rel_noise").as_double());
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
            "2D LiDAR noise node started — model: proportional Gaussian "
            "σ = max(%.3f m, %.2f·r)", min_noise_, rel_noise_);
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto out = *msg;
        const size_t n = out.ranges.size();

        for (size_t i = 0; i < n; ++i) {
            const float r = out.ranges[i];

            // Skip invalid returns (inf, NaN, out-of-range).
            if (!std::isfinite(r) || r <= 0.0f) continue;

            // Range-proportional Gaussian: σ = max(min_noise, rel_noise * r).
            // Derived from RPLidar S2 ranging accuracy spec (±1% of distance).
            const float sigma   = std::max(min_noise_, rel_noise_ * r);
            const float noisy_r = r + sigma * dist_norm_(gen_);

            out.ranges[i] = std::clamp(noisy_r, out.range_min, out.range_max);

            // Note: intensity values are left unmodified (ideal).
            // If reflectance-based segmentation is added downstream,
            // intensity noise should be modelled here.
        }

        pub_->publish(out);
    }

    float rel_noise_;
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
