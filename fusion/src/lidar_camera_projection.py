#!/usr/bin/env python3
# lidar_camera_projection.py

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
import yaml

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ros2_camera_lidar_fusion.read_yaml import extract_configuration

# =============================================================================
# Helper functions — CPU only (file I/O and message parsing)
# =============================================================================

def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No extrinsic file found: {yaml_path}")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    if 'extrinsic_matrix' not in data:
        raise KeyError(f"YAML {yaml_path} has no 'extrinsic_matrix' key.")
    T = np.array(data['extrinsic_matrix'], dtype=np.float32)
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix is not 4x4.")
    return T


def load_camera_calibration(yaml_path: str) -> (np.ndarray, np.ndarray):
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No camera calibration file: {yaml_path}")
    with open(yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)
    camera_matrix = np.array(
        calib_data['camera_matrix']['data'], dtype=np.float32
    ).reshape((3, 3))
    dist_coeffs = np.array(
        calib_data['distortion_coefficients']['data'], dtype=np.float32
    ).reshape((1, -1))
    return camera_matrix, dist_coeffs


def pointcloud2_to_xyz_array_fast(cloud_msg: PointCloud2, skip_rate: int = 1) -> np.ndarray:
    if cloud_msg.height == 0 or cloud_msg.width == 0:
        return np.zeros((0, 3), dtype=np.float32)
    field_names = [f.name for f in cloud_msg.fields]
    if not all(k in field_names for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('_', 'V{}'.format(cloud_msg.point_step - 12))
    ])
    raw_data = np.frombuffer(cloud_msg.data, dtype=dtype)
    points = np.zeros((raw_data.shape[0], 3), dtype=np.float32)
    points[:, 0] = raw_data['x']
    points[:, 1] = raw_data['y']
    points[:, 2] = raw_data['z']
    if skip_rate > 1:
        points = points[::skip_rate]
    return points


# =============================================================================
# ROS2 Node (CPU-only projection)
# =============================================================================

class LidarCameraProjectionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection_node')

        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")
            return

        config_folder = config_file['general']['config_folder']

        # Load calibration from YAML
        extrinsic_yaml = os.path.join(
            config_folder, config_file['general']['camera_extrinsic_calibration']
        )
        T_cpu = load_extrinsic_matrix(extrinsic_yaml)

        camera_yaml = os.path.join(
            config_folder, config_file['general']['camera_intrinsic_calibration']
        )
        camera_matrix_cpu, self.dist_coeffs = load_camera_calibration(camera_yaml)

        # Keep calibration matrices in CPU memory for RPi5 compatibility.
        self.cpu_T = T_cpu.T.astype(np.float32, copy=False)
        self.cpu_fx = np.float32(camera_matrix_cpu[0, 0])
        self.cpu_fy = np.float32(camera_matrix_cpu[1, 1])
        self.cpu_cx = np.float32(camera_matrix_cpu[0, 2])
        self.cpu_cy = np.float32(camera_matrix_cpu[1, 2])

        # Distance filter (meters)
        self.max_distance = 10.0
        self.min_distance = 0.3

        lidar_topic = config_file['lidar']['lidar_topic']
        image_topic = config_file['camera']['image_topic']
        self.get_logger().info(f"Subscribing to lidar: {lidar_topic}")
        self.get_logger().info(f"Subscribing to image: {image_topic}")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = Subscriber(self, Image, image_topic, qos_profile=sensor_qos)
        self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic, qos_profile=sensor_qos)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=5,
            slop=0.07
        )
        self.ts.registerCallback(self.sync_callback)

        self.pub_image = self.create_publisher(Image, "sensorfusion_out", 1)
        self.bridge = CvBridge()
        self.skip_rate = 1

        self.get_logger().info("LidarCameraProjectionNode ready (CPU-only projection).")

    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        # ── 1. Parse Image (CPU) ──────────────────────────────────────────
        cv_image = np.ascontiguousarray(
            self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8'),
            dtype=np.uint8
        )
        h, w = cv_image.shape[:2]

        # ── 2. Parse Point Cloud (CPU) ────────────────────────────────────
        xyz_np = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)
        if xyz_np.shape[0] == 0:
            self._publish(cv_image, image_msg)
            return

        # ── 3. Filter Distance on CPU ─────────────────────────────────────
        pts = xyz_np.astype(np.float32, copy=False)
        dist = np.linalg.norm(pts, axis=1)
        dist_mask = (dist >= self.min_distance) & (dist <= self.max_distance)
        pts = pts[dist_mask]
        dist = dist[dist_mask]

        if pts.shape[0] == 0:
            self._publish(cv_image, image_msg)
            return

        # ── 4. Transform & Project on CPU ─────────────────────────────────
        ones = np.ones((pts.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack((pts, ones))
        cam_pts = pts_h @ self.cpu_T

        front_mask = cam_pts[:, 2] > 0.0
        cam_pts = cam_pts[front_mask]
        dist = dist[front_mask]

        if cam_pts.shape[0] == 0:
            self._publish(cv_image, image_msg)
            return

        x = cam_pts[:, 0] / cam_pts[:, 2]
        y = cam_pts[:, 1] / cam_pts[:, 2]
        u = np.floor(self.cpu_fx * x + self.cpu_cx + 0.5).astype(np.int32)
        v = np.floor(self.cpu_fy * y + self.cpu_cy + 0.5).astype(np.int32)

        # ── 5. Boundary Check & Colors on CPU ─────────────────────────────
        valid = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u = u[valid]
        v = v[valid]
        dist = dist[valid]

        if u.shape[0] == 0:
            self._publish(cv_image, image_msg)
            return

        intensity = np.clip(dist / self.max_distance * 255, 0, 255).astype(np.uint8)
        colors = np.zeros((u.shape[0], 3), dtype=np.uint8)
        colors[:, 1] = intensity
        colors[:, 2] = 255 - intensity

        # ── 6. Draw directly on the CPU image ────────────────────────────
        cv_image[v, u] = colors
        cv_image[np.clip(v + 1, 0, h - 1), u] = colors
        cv_image[v, np.clip(u + 1, 0, w - 1)] = colors
        cv_image[np.clip(v + 1, 0, h - 1), np.clip(u + 1, 0, w - 1)] = colors

        self._publish(cv_image, image_msg)

    def _publish(self, cv_image: np.ndarray, image_msg: Image):
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = image_msg.header
        self.pub_image.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
