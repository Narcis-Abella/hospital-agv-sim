#!/usr/bin/env python3
"""
noisy_odom.py — Odometría realista para Agilex Tracer 2.0 en Ignition Gazebo / ROS2.

Pipeline:
  /odom_raw  (Odometry perfecta de Gazebo DiffDrive)
      ↓
  [este nodo]
    1. Ruido de encoder proporcional a la velocidad lineal  (slippage, cuantización)
    2. Ruido de yaw proporcional a la velocidad angular     (error diferencial encoder)
    3. Deriva acumulada de yaw proporcional a la distancia  (error sistemático)
    4. Covarianzas coherentes para robot_localization / EKF
      ↓
  /odom  (Odometry realista para EKF)

Fundamento físico — Tracer 2.0, encoder 1024 líneas fotoeléctrico incremental:
  Resolución lineal  : 2π × 0.0625 / 1024 ≈ 0.000383 m/pulso
  Resolución angular : 2π / 1024 ≈ 0.00614 rad/pulso por rueda
  Error de yaw observado empíricamente: ~30–40° por vuelta completa in-place
    → σ_yaw ≈ 0.08–0.12 rad en giro de 360°
    → yaw_var de pose ≈ 0.08 (deliberadamente alto para que el EKF confíe en la IMU)

Filosofía de covarianzas para este robot:
  El Tracer 2.0 es fiable en traslación recta pero impreciso en rotación.
  El EKF debe:
    - Confiar MUCHO en la odometría para Vx  (encoder lineal es bueno)
    - Confiar POCO  en la odometría para yaw (delegar a la IMU WT901C)
    - Confiar NADA  en la odometría para Vy  (robot no-holonómico, Vy ≈ 0)

Parámetros ROS2 configurables:
  lin_noise_ratio  (default: 0.02)  — 2% de Vx: ruido de encoder lineal
  ang_noise_ratio  (default: 0.08)  — 8% de ωz: ruido encoder diferencial en giro
  yaw_drift_rate   (default: 0.005) — rad/m: deriva sistemática de yaw por recorrido
"""

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class NoisyOdomNode(Node):

    def __init__(self):
        super().__init__('noisy_odom_node')

        # ── Parámetros ROS2 ───────────────────────────────────────────────────
        self.declare_parameter('lin_noise_ratio', 0.02)   # 2% de Vx
        self.declare_parameter('ang_noise_ratio', 0.08)   # 8% de ωz — Tracer impreciso en giro
        self.declare_parameter('yaw_drift_rate',  0.005)  # rad/m recorrido

        self.lin_ratio  = self.get_parameter('lin_noise_ratio').value
        self.ang_ratio  = self.get_parameter('ang_noise_ratio').value
        self.drift_rate = self.get_parameter('yaw_drift_rate').value

        # ── Estado interno ────────────────────────────────────────────────────
        self._yaw_drift = 0.0       # deriva acumulada del yaw [rad]
        self._last_x    = None      # posición anterior para calcular distancia recorrida
        self._last_y    = None

        # ── Covarianzas fijas para robot_localization ─────────────────────────
        #
        # Matriz Pose 6×6, orden [x, y, z, roll, pitch, yaw]
        # Índices diagonales: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        #
        # x, y  : σ² = 0.005 → σ = 0.07m  — bueno a corto plazo en línea recta
        # yaw   : σ² = 0.08  → σ = 0.28rad ≈ 16° — refleja imprecisión real del Tracer
        #         DELIBERADAMENTE ALTO: el EKF debe delegar el yaw a la IMU WT901C
        # z, roll, pitch: no medidos → valor alto para que el EKF los ignore
        #
        self._pose_cov = np.zeros(36)
        self._pose_cov[0]  = 0.005    # x
        self._pose_cov[7]  = 0.005    # y
        self._pose_cov[14] = 1e6      # z      — no medido
        self._pose_cov[21] = 1e6      # roll   — no medido
        self._pose_cov[28] = 1e6      # pitch  — no medido
        self._pose_cov[35] = 0.08     # yaw    — impreciso, ceder a IMU

        # Matriz Twist 6×6, orden [vx, vy, vz, ωx, ωy, ωz]
        # Índices diagonales: 0=vx, 7=vy, 14=vz, 21=ωx, 28=ωy, 35=ωz
        #
        # vx  : σ² = 0.001 → σ = 0.032 m/s — encoder lineal es fiable
        # vy  : σ² = 0.0001 — no-holonómico, Vy ≈ 0 por diseño
        # ωz  : σ² = 0.05  → σ = 0.22 rad/s — encoder diferencial impreciso en giro
        # resto: no medidos
        #
        self._twist_cov = np.zeros(36)
        self._twist_cov[0]  = 0.001    # vx  — fiable
        self._twist_cov[7]  = 0.0001   # vy  — no-holonómico
        self._twist_cov[14] = 1e6      # vz  — no medido
        self._twist_cov[21] = 1e6      # ωx  — no medido
        self._twist_cov[28] = 1e6      # ωy  — no medido
        self._twist_cov[35] = 0.05     # ωz  — impreciso en giro

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self.sub = self.create_subscription(Odometry, '/odom_raw', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info(
            f'\n[Noisy Odom Node] Agilex Tracer 2.0\n'
            f'  lin_noise_ratio : {self.lin_ratio*100:.0f}% de Vx\n'
            f'  ang_noise_ratio : {self.ang_ratio*100:.0f}% de ωz  '
            f'← crítico para yaw\n'
            f'  yaw_drift_rate  : {self.drift_rate*1000:.1f} mrad/m recorrido\n'
            f'  pose_cov[yaw]   : {self._pose_cov[35]:.3f} rad²  '
            f'(σ ≈ {np.sqrt(self._pose_cov[35])*57.3:.1f}°) — EKF delega yaw a IMU'
        )

    # ── Callback ──────────────────────────────────────────────────────────────

    def callback(self, msg: Odometry):
        new_msg = copy.deepcopy(msg)

        # Velocidades actuales del mensaje de Gazebo
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        # ── 1. Ruido proporcional a la velocidad lineal ───────────────────────
        # Modela slippage y cuantización del encoder en traslación
        # Solo se aplica si hay movimiento significativo
        if abs(vx) > 0.001:
            vx_noise = np.random.normal(0.0, abs(vx) * self.lin_ratio)
            new_msg.twist.twist.linear.x = vx + vx_noise

        # ── 2. Ruido proporcional a la velocidad angular ──────────────────────
        # Modela el error diferencial del encoder en giros
        # Este es el origen del problema de imprecisión en giros del Tracer
        if abs(wz) > 0.001:
            wz_noise = np.random.normal(0.0, abs(wz) * self.ang_ratio)
            new_msg.twist.twist.angular.z = wz + wz_noise

        # ── 3. Deriva acumulada de yaw ────────────────────────────────────────
        # Modela el error sistemático por pequeñas imprecisiones en wheel_separation
        # y desgaste diferencial de ruedas. Crece con la distancia recorrida.
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        if self._last_x is not None:
            dx = pos_x - self._last_x
            dy = pos_y - self._last_y
            dist_step = np.sqrt(dx*dx + dy*dy)

            if dist_step > 0.001:   # solo acumular si hay movimiento real
                # Paseo aleatorio del yaw proporcional a la distancia
                self._yaw_drift += np.random.normal(0.0, self.drift_rate * dist_step)

        self._last_x = pos_x
        self._last_y = pos_y

        # Aplicar deriva al yaw de la pose
        # Extraer yaw actual del quaternion
        q = msg.pose.pose.orientation
        # yaw = atan2(2(wz+xy), 1-2(y²+z²))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_current = np.arctan2(siny_cosp, cosy_cosp)
        yaw_noisy   = yaw_current + self._yaw_drift

        # Reconstruir quaternion con el yaw perturbado
        # (roll=0, pitch=0 para robot diferencial en suelo plano)
        new_msg.pose.pose.orientation.x = 0.0
        new_msg.pose.pose.orientation.y = 0.0
        new_msg.pose.pose.orientation.z = np.sin(yaw_noisy / 2.0)
        new_msg.pose.pose.orientation.w = np.cos(yaw_noisy / 2.0)

        # ── 4. Estampar covarianzas coherentes ───────────────────────────────
        new_msg.pose.covariance  = self._pose_cov.tolist()
        new_msg.twist.covariance = self._twist_cov.tolist()

        self.pub.publish(new_msg)


# ── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
