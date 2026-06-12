"""
lidar_camera_projection.py — Fuses LiDAR point cloud with camera image.

Pipeline:
  1. Subscribe to synced camera image + LiDAR point cloud
  2. Filter LiDAR points by distance (min/max range)
  3. Transform points: LiDAR frame -> camera frame (extrinsic matrix)
  4. Project 3D points onto 2D image (camera intrinsics)
  5. Draw color-coded dots: red=close, green=medium, blue=far
  6. Publish fused image to /sensorfusion_out
"""

import os
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import cupy as cp  # <--- IMPORTED CUPY FOR GPU ACCELERATION
import yaml
import struct

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ros2_camera_lidar_fusion.read_yaml import extract_configuration


# =============================================================================
# Helper functions
# =============================================================================

def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
    """Load 4x4 extrinsic matrix (LiDAR -> camera transform) from YAML."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No extrinsic file found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if 'extrinsic_matrix' not in data:
        raise KeyError(f"YAML {yaml_path} has no 'extrinsic_matrix' key.")

    T = np.array(data['extrinsic_matrix'], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix is not 4x4.")
    return T


def load_camera_calibration(yaml_path: str) -> (np.ndarray, np.ndarray):
    """Load camera intrinsic matrix (3x3) and distortion coefficients from YAML."""
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No camera calibration file: {yaml_path}")

    with open(yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)

    # Stored as flat 9-element list — reshape to 3x3
    camera_matrix = np.array(
        calib_data['camera_matrix']['data'], dtype=np.float64
    ).reshape((3, 3))

    dist_coeffs = np.array(
        calib_data['distortion_coefficients']['data'], dtype=np.float64
    ).reshape((1, -1))

    return camera_matrix, dist_coeffs


def pointcloud2_to_xyz_array_fast(cloud_msg: PointCloud2, skip_rate: int = 1) -> np.ndarray:
    """
    Convert PointCloud2 message to Nx3 numpy array (x, y, z) in meters.
    skip_rate: use every Nth point (1=all, 2=half, 4=quarter) to reduce CPU load.
    """
    if cloud_msg.height == 0 or cloud_msg.width == 0:
        return np.zeros((0, 3), dtype=np.float32)

    field_names = [f.name for f in cloud_msg.fields]
    if not all(k in field_names for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)

    # Structured dtype: x, y, z (12 bytes) + remaining fields as padding
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


def get_color_for_distance(distance: float, max_dist: float) -> tuple:
    """
    Map distance to BGR color: red=close, green=medium, blue=far.
    Returns (B, G, R) tuple for OpenCV.
    """
    ratio = float(np.clip(distance / max_dist, 0.0, 1.0))

    if ratio < 0.5:
        # Red -> Green
        t = ratio / 0.5
        r, g, b = int(255 * (1 - t)), int(255 * t), 0
    else:
        # Green -> Blue
        t = (ratio - 0.5) / 0.5
        r, g, b = 0, int(255 * (1 - t)), int(255 * t)

    return (b, g, r)  # OpenCV uses BGR


# =============================================================================
# ROS2 Node
# =============================================================================

class LidarCameraProjectionNode(Node):
    """
    Projects LiDAR points onto camera image with distance-based color coding.

    Subscribes: camera image + LiDAR point cloud (time-synced)
    Publishes:  /sensorfusion_out — camera image with LiDAR overlay
    """

    def __init__(self):
        super().__init__('lidar_camera_projection_node')

        # Load config
        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")
            return

        config_folder = config_file['general']['config_folder']

        # Extrinsic matrix: transforms LiDAR points into camera frame
        extrinsic_yaml = os.path.join(
            config_folder, config_file['general']['camera_extrinsic_calibration']
        )
        self.T_lidar_to_cam = load_extrinsic_matrix(extrinsic_yaml)

        # Camera intrinsics: projects 3D camera-frame points onto 2D image
        camera_yaml = os.path.join(
            config_folder, config_file['general']['camera_intrinsic_calibration']
        )
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(camera_yaml)

        self.get_logger().info("Loaded extrinsic:\n{}".format(self.T_lidar_to_cam))
        self.get_logger().info("Camera matrix:\n{}".format(self.camera_matrix))
        self.get_logger().info("Distortion coeffs:\n{}".format(self.dist_coeffs))

        # Distance filter: only show points within this range (meters)
        self.max_distance = 10.0  # discard points farther than this
        self.min_distance = 0.3   # discard points closer than this (noise)
        self.point_radius = 3     # dot size in pixels

        # Subscribers — synced within 70ms time window
        lidar_topic = config_file['lidar']['lidar_topic']
        image_topic = config_file['camera']['image_topic']
        self.get_logger().info(f"Subscribing to lidar: {lidar_topic}")
        self.get_logger().info(f"Subscribing to image: {image_topic}")

        self.image_sub = Subscriber(self, Image, image_topic)
        self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=5,
            slop=0.07  # max time difference between camera and LiDAR frames
        )
        self.ts.registerCallback(self.sync_callback)

        # Publisher
        self.pub_image = self.create_publisher(Image, "sensorfusion_out", 1)
        self.bridge = CvBridge()

        # Use every point; increase to 2 or 4 to reduce CPU load
        self.skip_rate = 1

    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        """Called on each synced camera + LiDAR pair."""

        # Convert ROS image to OpenCV BGR
        cv_image = np.ascontiguousarray(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8'), dtype=np.uint8)

        # Parse point cloud into Nx3 array
        xyz_lidar = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)
        if xyz_lidar.shape[0] == 0:
            self.get_logger().warn("Empty cloud. Nothing to project.")
            self._publish(cv_image, image_msg)
            return

        # Filter by distance range
        distances = np.linalg.norm(xyz_lidar, axis=1)
        dist_mask = (distances >= self.min_distance) & (distances <= self.max_distance)
        xyz_lidar = xyz_lidar[dist_mask]
        distances = distances[dist_mask]

        if xyz_lidar.shape[0] == 0:
            self.get_logger().info("No points within distance range.")
            self._publish(cv_image, image_msg)
            return

        # Transform LiDAR points -> camera frame using extrinsic matrix
        xyz_lidar_f64 = xyz_lidar.astype(np.float64)
        ones = np.ones((xyz_lidar_f64.shape[0], 1), dtype=np.float64)
        xyz_lidar_h = np.hstack((xyz_lidar_f64, ones))      # Nx4 homogeneous
        
        # --- GPU ACCELERATION BLOCK (CuPy) ---
        # 1. Send data to GPU memory
        gpu_xyz_lidar_h = cp.asarray(xyz_lidar_h)
        gpu_T_matrix = cp.asarray(self.T_lidar_to_cam.T)
        
        # 2. Perform the heavy matrix multiplication on the GPU
        gpu_xyz_cam = gpu_xyz_lidar_h @ gpu_T_matrix
        
        # 3. Bring the result back to CPU memory (NumPy) for the rest of the script
        xyz_cam = cp.asnumpy(gpu_xyz_cam)[:, :3]  
        # -------------------------------------

        # Keep only points in front of the camera (Z > 0)
        mask_in_front = (xyz_cam[:, 2] > 0.0)
        xyz_cam_front = xyz_cam[mask_in_front]
        distances_front = distances[mask_in_front]

        if xyz_cam_front.shape[0] == 0:
            self.get_logger().info("No points in front of camera.")
            self._publish(cv_image, image_msg)
            return

        # Project 3D -> 2D pixels (rvec/tvec are zero — already in camera frame)
        # Manual projection — avoids cv2.projectPoints OpenCV 4.13 issue
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        x = xyz_cam_front[:, 0] / xyz_cam_front[:, 2]
        y = xyz_cam_front[:, 1] / xyz_cam_front[:, 2]
        u_proj = (fx * x + cx).astype(np.float32)
        v_proj = (fy * y + cy).astype(np.float32)

        h, w = cv_image.shape[:2]

        # --- VECTORIZED PROJECTION (NumPy Array Modifying) ---
        # 1. Convert to integers all at once
        u_int = np.floor(u_proj + 0.5).astype(np.int32)
        v_int = np.floor(v_proj + 0.5).astype(np.int32)

        # 2. Create a mask to filter out points that fall outside the image boundaries
        valid_mask = (u_int >= 0) & (u_int < w) & (v_int >= 0) & (v_int < h)
        
        u_valid = u_int[valid_mask]
        v_valid = v_int[valid_mask]
        dist_valid = distances_front[valid_mask]

        if len(u_valid) > 0:
            # 3. Calculate colors for all valid points simultaneously
            color_intensity = np.clip(dist_valid / 10.0 * 255, 0, 255).astype(np.uint8)
            
            # 4. Create an array of BGR colors for each point
            # Format: (0, color_intensity, 255 - color_intensity)
            colors = np.zeros((len(u_valid), 3), dtype=np.uint8)
            colors[:, 1] = color_intensity          # Green channel
            colors[:, 2] = 255 - color_intensity    # Red channel
            
            # 5. Instantly color the pixels in the NumPy image array
            cv_image[v_valid, u_valid] = colors
            
            # OPTIONAL: Make points slightly thicker (2x2 pixels) so they are easier to see
            cv_image[np.clip(v_valid+1, 0, h-1), u_valid] = colors
            cv_image[v_valid, np.clip(u_valid+1, 0, w-1)] = colors
            cv_image[np.clip(v_valid+1, 0, h-1), np.clip(u_valid+1, 0, w-1)] = colors
        # -----------------------------------------------------

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = image_msg.header
        self.pub_image.publish(out_msg)

    def _publish(self, cv_image: np.ndarray, image_msg: Image):
        """Publish OpenCV image as ROS Image message."""
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = image_msg.header
        self.pub_image.publish(out_msg)

    def _draw_legend(self, img: np.ndarray):
        """Draw distance color bar (0m -> max_distance) in top-right corner."""
        h, w = img.shape[:2]
        bar_h, bar_w = 10, 150
        x0, y0 = w - bar_w - 10, 10

        for i in range(bar_w):
            color = get_color_for_distance((i / bar_w) * self.max_distance, self.max_distance)
            cv2.line(img, (x0 + i, y0), (x0 + i, y0 + bar_h), color, 1)

        cv2.putText(img, "0m", (x0, y0 + bar_h + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, f"{int(self.max_distance)}m", (x0 + bar_w - 20, y0 + bar_h + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)


# =============================================================================
# Entry point
# =============================================================================

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
