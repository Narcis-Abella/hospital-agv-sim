#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// LivoxNoiseNode
//   Lee  /livox/points_raw  (PointCloud2 de Gazebo)
//   Aplica ruido gaussiano proporcional a la distancia
//   Publica /livox/points    (PointCloud2, listo para SLAM que acepte PC2)
// ─────────────────────────────────────────────────────────────────────────────
class LivoxNoiseNode : public rclcpp::Node {
public:
    LivoxNoiseNode() : Node("livox_noise_node") {
        this->declare_parameter("rel_noise", 0.005); // 0.5% de la distancia
        this->declare_parameter("min_noise", 0.002); // 2 mm mínimo

        rel_noise_ = this->get_parameter("rel_noise").as_double();
        min_noise_ = this->get_parameter("min_noise").as_double();

        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_norm_ = std::normal_distribution<float>(0.0f, 1.0f);

        // QoS BestEffort para compatibilidad con Gazebo
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox_mid70/points_raw", qos,
            std::bind(&LivoxNoiseNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_mid70/points", 10);

        RCLCPP_INFO(this->get_logger(),
            "Livox Noise Node started → /livox_mid70/points (PointCloud2). rel=%.3f min=%.3f",
            rel_noise_, min_noise_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Copia del mensaje (header, campos, stride... todo igual)
        auto out = *msg;

        // Iteradores de LECTURA sobre el mensaje original
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

        // Iteradores de ESCRITURA sobre la copia
        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

        for (; in_x != in_x.end();
             ++in_x, ++in_y, ++in_z,
             ++out_x, ++out_y, ++out_z)
        {
            float x = *in_x, y = *in_y, z = *in_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            float dist = std::sqrt(x*x + y*y + z*z);
            if (dist < 1e-6f) continue;

            // σ proporcional a la distancia, con mínimo físico
            float sigma = std::max((float)min_noise_, (float)(rel_noise_ * dist));
            float noise = sigma * dist_norm_(gen_);
            float ratio = 1.0f + noise / dist;

            *out_x = x * ratio;
            *out_y = y * ratio;
            *out_z = z * ratio;
        }

        pub_->publish(out);
    }

    double rel_noise_, min_noise_;
    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxNoiseNode>());
    rclcpp::shutdown();
    return 0;
}
