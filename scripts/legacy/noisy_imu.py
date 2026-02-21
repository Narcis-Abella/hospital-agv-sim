#!/usr/bin/env python3
"""
Modelo de ruido realista para WitMotion WT901C-TTL en Ignition Gazebo / ROS2.

Arquitectura (Opción B):
  - Gazebo  → ruido blanco IID por muestra (configurado en imu.xacro)
  - Este nodo → efectos correlacionados en el tiempo:
      1. Bias derivante (proceso Gauss-Markov de 1er orden)
      2. Sensibilidad-g del giroscopio
      3. Cuantización del ADC (16 bits)
      4. Covarianzas coherentes para robot_localization / EKF

Referencias:
  - IEEE Std 952-1997 (modelo de error de IMU)
  - Tedaldi et al. 2014, "A Robust and Easy to Implement Method for IMU Calibration"
  - WitMotion WT901C-TTL datasheet
"""

import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np


class ImuRealisticModel(Node):

    def __init__(self):
        super().__init__('imu_realistic_model')

        # ── Tasa de actualización ─────────────────────────────────────────────
        # Debe coincidir exactamente con <update_rate> en imu.xacro
        self.update_rate = 100.0        # Hz
        self.dt          = 1.0 / self.update_rate

        # ── Parámetros del GIROSCOPIO (WT901C-TTL datasheet) ─────────────────
        # Ruido blanco: 0.1°/s RMS → modelado por Gazebo en xacro
        # Solo necesitamos el valor aquí para calcular la covarianza total
        self.gyro_white_noise_std = np.deg2rad(0.1)    # 0.00175 rad/s

        # Bias: proceso Gauss-Markov (oscila acotado, no diverge)
        self.gyro_bias_std = 0.0003     # rad/s — std del proceso estacionario
        self.gyro_bias_tau = 400.0      # seg   — tiempo de correlación (MEMS budget)

        # ── Parámetros del ACELERÓMETRO (WT901C-TTL datasheet) ───────────────
        # Ruido blanco: ~0.002g RMS → modelado por Gazebo en xacro
        self.accel_white_noise_std = 0.020              # m/s²

        # Bias: proceso Gauss-Markov
        self.accel_bias_std = 0.010     # m/s²
        self.accel_bias_tau = 300.0     # seg

        # ── Sensibilidad-g del giroscopio ────────────────────────────────────
        # Acoplamiento lineal aceleración → velocidad angular percibida
        # Típico para MEMS budget: ~0.1°/s per g = 0.00175 (rad/s)/(m/s²)
        # Relevante en el Tracer 2.0 al acelerar/frenar (eje Z principalmente)
        g_sens = np.deg2rad(0.1) / 9.81  # ≈ 1.785e-4 (rad/s)/(m/s²)
        self.G_sensitivity = np.diag([g_sens, g_sens, g_sens])

        # ── Cuantización del ADC (16 bits) ───────────────────────────────────
        # Giroscopio: rango ±2000°/s → resolución por LSB
        self.gyro_lsb  = (2.0 * np.deg2rad(2000.0)) / (2**16)  # ≈ 0.00107 rad/s
        # Acelerómetro: rango ±16g → resolución por LSB
        self.accel_lsb = (2.0 * 16.0 * 9.81)        / (2**16)  # ≈ 0.0048  m/s²

        # ── Estado interno del bias (Gauss-Markov, inicializado a 0) ─────────
        self.gyro_bias  = np.zeros(3)
        self.accel_bias = np.zeros(3)

        # Factores de correlación temporal (discretización exacta del proceso GM)
        self.alpha_gyro  = np.exp(-self.dt / self.gyro_bias_tau)
        self.alpha_accel = np.exp(-self.dt / self.accel_bias_tau)

        # Std del ruido de conducción del proceso GM
        # Garantiza varianza estacionaria = bias_std² en t→∞
        self.gyro_drive_std  = self.gyro_bias_std  * np.sqrt(1.0 - self.alpha_gyro**2)
        self.accel_drive_std = self.accel_bias_std * np.sqrt(1.0 - self.alpha_accel**2)

        # ── Covarianzas para robot_localization ──────────────────────────────
        # Varianza total = varianza ruido blanco (Gazebo) + varianza bias GM
        # Esto es lo que el EKF debe conocer como incertidumbre del sensor
        gyro_var  = self.gyro_white_noise_std**2  + self.gyro_bias_std**2   # (rad/s)²
        accel_var = self.accel_white_noise_std**2 + self.accel_bias_std**2  # (m/s²)²

        # Orientación: WT901C especifica ±0.05° estático, pero en dinámico
        # (robot en movimiento) el error sube a ~0.5°–1°
        ori_std = np.deg2rad(0.5)       # 0.5° RMS en condiciones dinámicas
        ori_var = ori_std**2            # (rad)²

        # Matrices de covarianza 3×3 (diagonales, sin correlaciones cruzadas)
        z = 0.0
        self.gyro_cov  = [gyro_var,  z, z,  z, gyro_var,  z,  z, z, gyro_var ]
        self.accel_cov = [accel_var, z, z,  z, accel_var, z,  z, z, accel_var]
        self.ori_cov   = [ori_var,   z, z,  z, ori_var,   z,  z, z, ori_var  ]

        # ── Pub / Sub ─────────────────────────────────────────────────────────
        self.sub = self.create_subscription(Imu, '/imu_raw', self.callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        self.get_logger().info(
            f'\n[IMU Realistic Model] WitMotion WT901C-TTL\n'
            f'  gyro  σ_white={self.gyro_white_noise_std*1000:.3f} mrad/s | '
            f'σ_bias={self.gyro_bias_std*1000:.3f} mrad/s | τ={self.gyro_bias_tau}s\n'
            f'  accel σ_white={self.accel_white_noise_std*1000:.1f} mm/s² | '
            f'σ_bias={self.accel_bias_std*1000:.1f} mm/s² | τ={self.accel_bias_tau}s\n'
            f'  gyro_lsb={self.gyro_lsb*1000:.4f} mrad/s | '
            f'accel_lsb={self.accel_lsb*1000:.3f} mm/s²'
        )

    # ── Métodos auxiliares ────────────────────────────────────────────────────

    def _step_bias_gauss_markov(self):
        """
        Avanza el proceso de Gauss-Markov un paso dt para ambos sensores.
        Ecuación: b[k] = α·b[k-1] + σ_drive·w[k],  w[k] ~ N(0,1)
        """
        self.gyro_bias  = (self.alpha_gyro  * self.gyro_bias
                           + self.gyro_drive_std  * np.random.randn(3))
        self.accel_bias = (self.alpha_accel * self.accel_bias
                           + self.accel_drive_std * np.random.randn(3))

    @staticmethod
    def _quantize(value: np.ndarray, lsb: float) -> np.ndarray:
        """Aplica cuantización uniforme de ADC (redondeo al LSB más cercano)."""
        return np.round(value / lsb) * lsb

    # ── Callback principal ────────────────────────────────────────────────────

    def callback(self, msg: Imu):
        new_msg = copy.deepcopy(msg)    # copia real del mensaje, no alias

        # 1. Avanzar el estado del bias (Gauss-Markov)
        self._step_bias_gauss_markov()

        # 2. Leer mediciones de Gazebo (ya llevan ruido blanco del xacro)
        gyro_raw  = np.array([msg.angular_velocity.x,
                               msg.angular_velocity.y,
                               msg.angular_velocity.z])
        accel_raw = np.array([msg.linear_acceleration.x,
                               msg.linear_acceleration.y,
                               msg.linear_acceleration.z])

        # 3. Giroscopio: añadir bias GM + acoplamiento g-sensitivity
        gyro_out = gyro_raw + self.gyro_bias + self.G_sensitivity @ accel_raw
        gyro_out = self._quantize(gyro_out, self.gyro_lsb)

        new_msg.angular_velocity.x = float(gyro_out[0])
        new_msg.angular_velocity.y = float(gyro_out[1])
        new_msg.angular_velocity.z = float(gyro_out[2])

        # 4. Acelerómetro: añadir bias GM
        accel_out = accel_raw + self.accel_bias
        accel_out = self._quantize(accel_out, self.accel_lsb)

        new_msg.linear_acceleration.x = float(accel_out[0])
        new_msg.linear_acceleration.y = float(accel_out[1])
        new_msg.linear_acceleration.z = float(accel_out[2])

        # 5. Estampar covarianzas (workaround bug covarianza Ignition Gazebo)
        new_msg.orientation_covariance         = self.ori_cov
        new_msg.angular_velocity_covariance    = self.gyro_cov
        new_msg.linear_acceleration_covariance = self.accel_cov

        self.pub.publish(new_msg)


# ── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ImuRealisticModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()