#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>
#include <cmath>

// Applies range-proportional radial Gaussian noise to Livox Mid-70 PointCloud2.
//
// Noise model: σ = max(min_noise, rel_noise * r), applied along line-of-sight.
// Parameters match Livox Mid-70 datasheet ranging accuracy (±2 cm at 1–6 m).
//
// Topic routing:
//   /livox_mid70/points_raw → [this node] → /livox_mid70/points
class NoisyLivoxMid70Node : public rclcpp::Node {
public:
    NoisyLivoxMid70Node() : Node("noisy_livox_mid70_node") {

        // rel_noise: fraction of range used as σ (0.5% ≈ 2 cm at 4 m)
        // min_noise: absolute noise floor regardless of range (2 mm)
        this->declare_parameter("rel_noise", 0.005);
        this->declare_parameter("min_noise", 0.002);

        rel_noise_ = static_cast<float>(this->get_parameter("rel_noise").as_double());
        min_noise_ = static_cast<float>(this->get_parameter("min_noise").as_double());

        gen_       = std::mt19937(std::random_device{}());
        dist_norm_ = std::normal_distribution<float>(0.0f, 1.0f);

        // Reliable QoS: matches parameter_bridge default output.
        // Verify with: ros2 topic info /livox_mid70/points_raw --verbose
        auto qos = rclcpp::QoS(10);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox_mid70/points_raw", qos,
            std::bind(&NoisyLivoxMid70Node::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox_mid70/points", qos);

        RCLCPP_INFO(this->get_logger(),
            "Livox Mid-70 noise node started — model: radial Gaussian "
            "σ = max(%.3f m, %.1f%% of r)",
            min_noise_, rel_noise_ * 100.0f);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Shallow copy: same field layout, stride, and header.
        // Only x/y/z bytes are modified via iterators.
        auto out = *msg;

        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

        for (; in_x != in_x.end();
             ++in_x, ++in_y, ++in_z,
             ++out_x, ++out_y, ++out_z)
        {
            const float x = *in_x, y = *in_y, z = *in_z;

            // Pass through invalid returns unchanged.
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            const float dist = std::sqrt(x * x + y * y + z * z);
            if (dist < 1e-6f) continue;

            // Radial noise: perturb range, preserve azimuth and elevation.
            // ratio = 1 + Δr/r  →  (x',y',z') = ratio * (x,y,z)
            const float sigma = std::max(min_noise_, rel_noise_ * dist);
            const float ratio = 1.0f + (sigma * dist_norm_(gen_)) / dist;

            *out_x = x * ratio;
            *out_y = y * ratio;
            *out_z = z * ratio;
        }

        pub_->publish(out);
    }

    float rel_noise_;
    float min_noise_;

    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyLivoxMid70Node>());
    rclcpp::shutdown();
    return 0;
}
