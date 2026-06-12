import rclpy
from rclpy.node import Node
import cupy as cp
import numpy as np

# CRITICAL FIX 1: Prevent aggressive GPU memory pool from crashing the Jetson
cp.cuda.set_allocator(None)

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.get_logger().info("Initializing GPU-Accelerated Sensor Fusion...")

        # CRITICAL FIX 2: Force float32 for Jetson hardware optimization
        # (Replace these values with your actual OAK-D intrinsic matrix)
        self.K = cp.array([[600.0, 0.0, 320.0],
                           [0.0, 600.0, 240.0],
                           [0.0, 0.0, 1.0]], dtype=cp.float32)

        # (Replace with your actual LiDAR-to-Camera extrinsic matrix)
        self.extrinsic = cp.eye(4, dtype=cp.float32) 

    def project_lidar_to_camera(self, lidar_points_np):
        """
        Projects 3D LiDAR points to 2D camera plane using the Jetson GPU.
        lidar_points_np: standard numpy array from ROS 2 PointCloud2 message
        """
        # 1. Move data: CPU (NumPy) -> GPU (CuPy) as float32
        points_gpu = cp.asarray(lidar_points_np, dtype=cp.float32)

        # 2. Make points homogeneous (x, y, z, 1)
        num_points = points_gpu.shape[0]
        ones = cp.ones((num_points, 1), dtype=cp.float32)
        points_homo = cp.hstack((points_gpu[:, :3], ones))

        # 3. Transform points to camera frame (Fast GPU Matrix Math)
        cam_points = cp.dot(self.extrinsic, points_homo.T).T

        # 4. Filter out points behind the camera (Z <= 0)
        front_points_mask = cam_points[:, 2] > 0
        cam_points_front = cam_points[front_points_mask]

        # 5. Project to 2D pixels (u, v)
        z_c = cam_points_front[:, 2].reshape(-1, 1)
        pixels_homo = cp.dot(self.K, cam_points_front[:, :3].T).T / z_c

        u = pixels_homo[:, 0]
        v = pixels_homo[:, 1]

        # 6. Move results back to CPU so ROS 2 can publish them standardly
        u_cpu = cp.asnumpy(u)
        v_cpu = cp.asnumpy(v)
        depths_cpu = cp.asnumpy(z_c.flatten())
        
        return u_cpu, v_cpu, depths_cpu
