#!/usr/bin/env python3
"""
noisy_lidar.py — Ruido realista para RPLidar A2M12 en Ignition Gazebo / ROS2.

Pipeline:
  /scan_raw  (LaserScan perfecto de Gazebo)
      ↓
  [este nodo]
    - Ruido gaussiano proporcional a la distancia: σ(d) = max(min_noise, rel_noise × d)
    - Solo se aplica a puntos válidos (filtra inf / nan)
    - Clamp al rango válido del sensor
      ↓
  /scan  (LaserScan realista para Nav2 / SLAM)

Parámetros ROS2 configurables (sin recompilar):
  rel_noise   (default: 0.01)   — ruido relativo: 1% de la distancia (datasheet A2M12)
  min_noise   (default: 0.003)  — ruido mínimo absoluto en metros: 3mm

Referencia: SLAMTEC RPLidar A2M12 datasheet, error de distancia < 1% @ 0.15–12m.
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class NoisyLidarNode(Node):

    def __init__(self):
        super().__init__('noisy_lidar_node')

        # ── Parámetros ROS2 (configurables desde launch.py sin recompilar) ────
        self.declare_parameter('rel_noise', 0.01)    # 1% de la distancia
        self.declare_parameter('min_noise', 0.003)   # 3mm mínimo absoluto

        self.rel_noise = self.get_parameter('rel_noise').value
        self.min_noise = self.get_parameter('min_noise').value

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        # Entrada: scan perfecto de Gazebo (bridge remapea /scan → /scan_raw)
        # Salida:  scan con ruido realista para Nav2 / SLAM
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self.callback, 10
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            f'\n[Noisy Lidar Node] RPLidar A2M12\n'
            f'  rel_noise : {self.rel_noise * 100:.1f}% de la distancia\n'
            f'  min_noise : {self.min_noise * 1000:.0f} mm mínimo absoluto\n'
            f'  /scan_raw → /scan'
        )

    def callback(self, msg: LaserScan):
        new_msg = copy.deepcopy(msg)
        ranges  = np.array(msg.ranges, dtype=np.float32)

        # Máscara de puntos válidos (excluye inf, nan y ceros)
        valid = np.isfinite(ranges) & (ranges > 0.0)

        if np.any(valid):
            valid_ranges = ranges[valid]

            # σ(d) = max(min_noise, rel_noise × d)
            # Calculado solo sobre puntos válidos para evitar sigma=inf en inválidos
            sigma = np.maximum(self.min_noise, self.rel_noise * valid_ranges)

            # Ruido gaussiano en la dirección radial (igual que en el sensor real)
            noise = np.random.normal(0.0, sigma).astype(np.float32)
            ranges[valid] = valid_ranges + noise

            # Clamp al rango válido del sensor (no puede salir del rango físico)
            ranges[valid] = np.clip(
                ranges[valid], msg.range_min, msg.range_max
            )

        new_msg.ranges = ranges.tolist()
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoisyLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()