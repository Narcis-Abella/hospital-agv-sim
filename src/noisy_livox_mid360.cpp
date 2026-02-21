#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// NoisyLivoxMid360Node
//
//   Specs ruido Livox Mid-360 (datasheet):
//     - Rango   1–6 m   → σ ≈ 2 cm  (0.020 m)
//     - Rango   6–40 m  → σ ≈ 3 cm  (0.030 m)
//   Lo modelamos como ruido proporcional con piso mínimo:
//     σ = max(min_noise, rel_noise × dist)
//
//   Flujo:
//     /livox_mid360/points_raw  (PointCloud2, BestEffort, de Gazebo bridge)
//     → aplica ruido gaussiano radial
//     → /livox_mid360/points    (PointCloud2, Reliable)
// ─────────────────────────────────────────────────────────────────────────────
class NoisyLivoxMid360Node : public rclcpp::Node {
public:
    NoisyLivoxMid360Node() : Node("noisy_livox_mid360_node") {

        // ── Parámetros ──────────────────────────────────────────────────────
        // rel_noise: fracción de la distancia usada como σ
        // min_noise: σ mínimo aunque el punto esté muy cerca (piso físico)
        this->declare_parameter("rel_noise", 0.005); // 0.5% → ~2 cm a 4 m
        this->declare_parameter("min_noise", 0.002); // 2 mm mínimo absoluto

        rel_noise_ = this->get_parameter("rel_noise").as_double();
        min_noise_ = this->get_parameter("min_noise").as_double();

        // ── RNG ─────────────────────────────────────────────────────────────
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_norm_ = std::normal_distribution<float>(0.0f, 1.0f);

        // ── QoS ─────────────────────────────────────────────────────────────
        // El bridge de Gazebo publica con BestEffort; el subscriber debe matchear
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        // ── Pub / Sub ────────────────────────────────────────────────────────
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox_mid360/points_raw", qos,
            std::bind(&NoisyLivoxMid360Node::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox_mid360/points", qos);

        RCLCPP_INFO(this->get_logger(),
            "Noisy Livox Mid-360 Node started. rel_noise=%.4f  min_noise=%.4f",
            rel_noise_, min_noise_);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Copia superficial: mismo layout de campos, mismo stride, mismo header.
        // Solo modificamos los bytes de x/y/z mediante iteradores.
        auto out = *msg;

        // Iteradores lectura (sobre msg original)
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

        // Iteradores escritura (sobre la copia)
        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

        for (; in_x != in_x.end();
               ++in_x, ++in_y, ++in_z,
               ++out_x, ++out_y, ++out_z)
        {
            const float x = *in_x;
            const float y = *in_y;
            const float z = *in_z;

            // Saltar puntos inválidos (NaN, Inf) — los dejamos como están
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            const float dist = std::sqrt(x*x + y*y + z*z);
            if (dist < 1e-6f) continue; // evitar división por cero

            // σ proporcional a la distancia con piso mínimo
            const float sigma = std::max(static_cast<float>(min_noise_),
                                         static_cast<float>(rel_noise_) * dist);

            // Ruido radial: desplazamos el punto a lo largo de su vector
            const float noise  = sigma * dist_norm_(gen_);
            const float ratio  = 1.0f + noise / dist;

            *out_x = x * ratio;
            *out_y = y * ratio;
            *out_z = z * ratio;
        }

        pub_->publish(out);
    }

    // ── Miembros ──────────────────────────────────────────────────────────────
    double rel_noise_;
    double min_noise_;

    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyLivoxMid360Node>());
    rclcpp::shutdown();
    return 0;
}
