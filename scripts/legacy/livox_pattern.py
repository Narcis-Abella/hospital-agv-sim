#!/usr/bin/env python3
"""
livox_pattern.py — Nodo de inyección de ruido OPTIMIZADO y BLINDADO.

Correcciones Finales:
  - Elimina RuntimeWarning filtrando valores 'inf' (rayos perdidos de Gazebo).
  - Salida confirmada en /livox/points_cloud.
  - Gestión de memoria robusta (contiguous array).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# ── Serialización Vectorizada (High Performance) ──────────────────────────────

def xyz_to_pointcloud2_fastlio_optimized(xyz: np.ndarray, header: Header) -> PointCloud2:
    """
    Serializa numpy (N, 3) a PointCloud2 compatible con Fast-LIO2.
    """
    n_points = xyz.shape[0]
    point_step = 18 

    # Definir campos: x, y, z, intensity, tag, line
    fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='tag',       offset=16, datatype=PointField.UINT8,   count=1),
        PointField(name='line',      offset=17, datatype=PointField.UINT8,   count=1),
    ]

    if n_points == 0:
        return PointCloud2(header=header, height=1, width=0, fields=fields, 
                           point_step=point_step, row_step=0, data=b'', is_dense=True)

    # 1. Crear buffer
    buffer = np.zeros((n_points, point_step), dtype=np.uint8)

    # 2. Convertir XYZ a bytes
    # IMPORTANTE: .copy() asegura memoria contigua para evitar ValueError
    xyz_f32 = xyz.astype(np.float32)
    
    buffer[:, 0:4]  = xyz_f32[:, 0].copy().view(np.uint8).reshape(-1, 4) # X
    buffer[:, 4:8]  = xyz_f32[:, 1].copy().view(np.uint8).reshape(-1, 4) # Y
    buffer[:, 8:12] = xyz_f32[:, 2].copy().view(np.uint8).reshape(-1, 4) # Z

    # 3. Calcular intensidad (basada en rango)
    intensity = np.linalg.norm(xyz_f32, axis=1).astype(np.float32)
    buffer[:, 12:16] = intensity.copy().view(np.uint8).reshape(-1, 4)

    # 4. Empaquetar
    msg = PointCloud2()
    msg.header       = header
    msg.height       = 1
    msg.width        = n_points
    msg.fields       = fields
    msg.is_bigendian = False
    msg.point_step   = point_step
    msg.row_step     = point_step * n_points
    msg.data         = buffer.tobytes()
    msg.is_dense     = True
    return msg


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Deserializa PointCloud2 a numpy (N,3)."""
    point_step = msg.point_step
    data_bytes = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = len(data_bytes) // point_step
    
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw_view = data_bytes.reshape(n_points, point_step)
    xyz = np.zeros((n_points, 3), dtype=np.float32)

    xyz[:, 0] = raw_view[:, 0:4].copy().view(dtype=np.float32).ravel()
    xyz[:, 1] = raw_view[:, 4:8].copy().view(dtype=np.float32).ravel()
    xyz[:, 2] = raw_view[:, 8:12].copy().view(dtype=np.float32).ravel()
    
    return xyz


# ── Nodo Principal ────────────────────────────────────────────────────────────

class LivoxNoisyNode(Node):

    def __init__(self):
        super().__init__('livox_pattern_node')

        self.declare_parameter('rel_noise', 0.005)
        self.declare_parameter('min_noise', 0.002)

        self.rel_noise = self.get_parameter('rel_noise').value
        self.min_noise = self.get_parameter('min_noise').value

        self.sub = self.create_subscription(
            PointCloud2, '/livox/points_raw', self.callback, 10
        )
        self.pub = self.create_publisher(PointCloud2, '/livox/points_cloud', 10)

        self.get_logger().info(
            f'[Livox Optimized Node] Activo.\n'
            f'  Input : /livox/points_raw\n'
            f'  Output: /livox/points_cloud'
        )

    def _apply_distance_noise(self, xyz: np.ndarray) -> np.ndarray:
        if len(xyz) == 0:
            return xyz

        # ── PASO CRÍTICO: Eliminar Infinitos y NaNs ──
        # Gazebo envía 'inf' cuando el láser no toca nada.
        # Si intentamos normalizar un vector (inf, inf, inf), Python da error.
        mask = np.isfinite(xyz).all(axis=1)
        xyz_clean = xyz[mask]

        if len(xyz_clean) == 0:
            return xyz_clean

        # Calcular distancias sobre los puntos válidos
        dist = np.linalg.norm(xyz_clean, axis=1, keepdims=True)
        
        # Modelo de ruido
        sigma = np.maximum(self.min_noise, self.rel_noise * dist)
        
        # Calcular dirección y aplicar ruido
        # Usamos 'where' por seguridad, aunque ya no debería haber dist=0 o inf
        safe_dist = np.where(dist < 1e-9, 1e-9, dist)
        direction = xyz_clean / safe_dist
        
        noise_magnitude = np.random.normal(0.0, sigma)
        xyz_noisy = xyz_clean + (direction * noise_magnitude)

        return xyz_noisy.astype(np.float32)

    def callback(self, msg: PointCloud2):
        # 1. Deserializar
        xyz = pointcloud2_to_xyz(msg)
        if len(xyz) == 0: return

        # 2. Limpiar y Añadir Ruido (Maneja Infs internamente)
        xyz_noisy = self._apply_distance_noise(xyz)

        # 3. Serializar
        out_msg = xyz_to_pointcloud2_fastlio_optimized(xyz_noisy, msg.header)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxNoisyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()