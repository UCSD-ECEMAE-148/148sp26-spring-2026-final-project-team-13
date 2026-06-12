#!/usr/bin/env python3
#lidar_camera_project_detection.py

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
import yaml
import struct

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from ros2_camera_lidar_fusion.read_yaml import extract_configuration

try:
    from ros2_camera_lidar_fusion.hailo_yolo_client import (
        HailoYoloClient,
        hailo_device_present,
        kill_stale_hailo_bridges,
        resolve_hailo_hef_model,
    )
except ImportError:
    HailoYoloClient = None
    hailo_device_present = lambda: False
    resolve_hailo_hef_model = lambda arch=None: None


def load_extrinsic_matrix(yaml_path: str) -> np.ndarray:
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
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"No camera calibration file: {yaml_path}")
    with open(yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)
    camera_matrix = np.array(calib_data['camera_matrix']['data'], dtype=np.float64).reshape((3, 3))
    dist_coeffs = np.array(calib_data['distortion_coefficients']['data'], dtype=np.float64).reshape((1, -1))
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


def detect_cones(frame, hsv_lb, hsv_ub):
    """
    Detects cones using HSV color thresholding and convex hull shape analysis.
    Taken from GitHub reference — finds upward-pointing triangular shapes.
    Returns list of bounding rects (x, y, w, h).
    """
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_thresh = cv2.inRange(frame_hsv, hsv_lb, hsv_ub)
    kernel = np.ones((5, 5))
    img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
    img_edges = cv2.Canny(img_thresh_blurred, 80, 160)
    contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    approx_contours = [cv2.approxPolyDP(c, 10, closed=True) for c in contours]
    all_convex_hulls = [cv2.convexHull(ac) for ac in approx_contours]
    convex_hulls_3to10 = [cv2.convexHull(ch) for ch in all_convex_hulls if 3 <= len(ch) <= 10]

    bounding_rects = []
    for ch in convex_hulls_3to10:
        if convex_hull_pointing_up(ch):
            bounding_rects.append(cv2.boundingRect(ch))
    return bounding_rects


def convex_hull_pointing_up(ch):
    """Returns True if the convex hull is pointing upward (cone shape)."""
    x, y, w, h = cv2.boundingRect(ch)
    if w / h >= 0.8:
        return False

    vertical_center = y + h / 2
    points_above = [p for p in ch if p[0][1] < vertical_center]
    points_below = [p for p in ch if p[0][1] >= vertical_center]

    if not points_below:
        return False

    left_x = min(p[0][0] for p in points_below)
    right_x = max(p[0][0] for p in points_below)

    for p in points_above:
        if p[0][0] < left_x or p[0][0] > right_x:
            return False
    return True


class LidarCameraProjectionDetectionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection_node')

        config_file = extract_configuration()
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")
            return

        config_folder = config_file['general']['config_folder']
        extrinsic_yaml = os.path.join(config_folder, config_file['general']['camera_extrinsic_calibration'])
        self.T_lidar_to_cam = load_extrinsic_matrix(extrinsic_yaml)

        camera_yaml = os.path.join(config_folder, config_file['general']['camera_intrinsic_calibration'])
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(camera_yaml)

        self.get_logger().info("Loaded extrinsic:\n{}".format(self.T_lidar_to_cam))
        self.get_logger().info("Camera matrix:\n{}".format(self.camera_matrix))
        self.get_logger().info("Distortion coeffs:\n{}".format(self.dist_coeffs))

        # YOLO backend: hailo (NPU .hef) or cpu (ultralytics .pt). HSV cones always run.
        self.enable_detection = os.environ.get("SENSORFUSION_ENABLE_DETECTION", "1") == "1"
        self.detection_backend = os.environ.get("SENSORFUSION_DETECTION_BACKEND", "auto").lower()
        self.detection_conf = float(os.environ.get("SENSORFUSION_DETECTION_CONF", "0.25"))
        self.yolo_model = None
        self.hailo_client = None

        if self.enable_detection:
            backend = self._resolve_detection_backend()
            if backend == "hailo":
                hef_path = self._resolve_hailo_model()
                if hef_path is None:
                    self.get_logger().warn("No Hailo .hef model found; falling back to CPU YOLO.")
                    backend = "cpu"
                elif HailoYoloClient is None:
                    self.get_logger().warn("Hailo client unavailable; falling back to CPU YOLO.")
                    backend = "cpu"
                else:
                    hailo_err = None
                    for attempt in range(2):
                        try:
                            if attempt > 0:
                                kill_stale_hailo_bridges()
                                self.get_logger().warn("Retrying Hailo init after releasing stale NPU handles...")
                            self.get_logger().info(f"Loading Hailo model from {hef_path}.")
                            self.hailo_client = HailoYoloClient(
                                hef_path, conf_thresh=self.detection_conf, logger=self.get_logger()
                            )
                            self.detection_backend = "hailo"
                            self.get_logger().info("Hailo NPU detection enabled.")
                            hailo_err = None
                            break
                        except Exception as exc:
                            hailo_err = exc
                            if self.hailo_client is not None:
                                self.hailo_client.close()
                                self.hailo_client = None
                    if hailo_err is not None:
                        self.get_logger().warn(f"Hailo init failed ({hailo_err}); falling back to CPU YOLO.")
                        backend = "cpu"

            if backend == "cpu":
                self.detection_backend = "cpu"
                if YOLO is None:
                    self.get_logger().warn("ultralytics is not installed; YOLO disabled (HSV cones still active).")
                    self.enable_detection = False
                else:
                    model_path = self._resolve_cpu_model()
                    if model_path is None:
                        self.get_logger().warn("No YOLO model found; YOLO disabled (HSV cones still active).")
                        self.enable_detection = False
                    else:
                        self.get_logger().info(f"Loading CPU YOLO model from {model_path}.")
                        self.yolo_model = YOLO(model_path, task='detect')
                        self.get_logger().info("CPU YOLO detection enabled.")

        if not self.enable_detection:
            self.get_logger().info("YOLO disabled; running LiDAR projection + HSV cone detection.")
        # HSV thresholds for cone color detection (from GitHub reference)
        self.cone_hsv_lb = np.array([109, 83, 131])
        self.cone_hsv_ub = np.array([180, 255, 255])

        # Max distance for color coding (meters)
        self.max_dist_thresh = 10

        lidar_topic = config_file['lidar']['lidar_topic']
        image_topic = config_file['camera']['image_topic']
        self.get_logger().info(f"Subscribing to lidar: {lidar_topic}")
        self.get_logger().info(f"Subscribing to image: {image_topic}")

        # Camera publishes RELIABLE; LiDAR publishes RELIABLE — match both.
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = Subscriber(self, Image, image_topic, qos_profile=reliable_qos)
        self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic, qos_profile=reliable_qos)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=5,
            slop=0.2
        )
        self.ts.registerCallback(self.sync_callback)

        self.pub_image = self.create_publisher(Image, "sensorfusion_out2", 1)
        self.bridge = CvBridge()
        self.skip_rate = 1
        self._det_log_counter = 0

    def _resolve_detection_backend(self):
        backend = self.detection_backend
        if backend == "auto":
            if hailo_device_present() and resolve_hailo_hef_model() is not None:
                return "hailo"
            return "cpu"
        return backend

    def _resolve_hailo_model(self):
        return resolve_hailo_hef_model()

    def _resolve_cpu_model(self):
        """Pick a YOLO weights file. Pi 5 uses .pt; Jetson may use TensorRT .engine."""
        candidates = []
        env_model = os.environ.get("SENSORFUSION_DETECTION_MODEL", "").strip()
        if env_model and env_model.endswith((".pt", ".engine", ".onnx")):
            candidates.append(env_model)

        config_folder = os.environ.get(
            "SENSORFUSION_MODEL_DIR",
            "/ros2_ws/src/ros2_camera_lidar_fusion/models",
        )
        for name in ("yolov8n.pt", "yolov8n.engine", "yolov8n.onnx"):
            candidates.append(os.path.join(config_folder, name))
            candidates.append(name)

        for path in candidates:
            if path and os.path.isfile(path):
                return path

        if YOLO is not None:
            return "yolov8n.pt"
        return None

    def _run_yolo_detections(self, cv_image, w, h):
        """Return list of (class_name, conf, x1, y1, x2, y2)."""
        detections = []
        try:
            if self.detection_backend == "hailo" and self.hailo_client is not None:
                for class_name, conf, x1, y1, x2, y2 in self.hailo_client.detect(cv_image):
                    detections.append((class_name, conf, x1, y1, x2, y2))
            elif self.yolo_model is not None:
                results = self.yolo_model(cv_image, verbose=False)
                for result in results:
                    for box in result.boxes:
                        class_name = self.yolo_model.names[int(box.cls[0])]
                        conf = float(box.conf[0])
                        b = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                        x1, y1 = max(0, x1), max(0, y1)
                        x2, y2 = min(w, x2), min(h, y2)
                        detections.append((class_name, conf, x1, y1, x2, y2))
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
        self._det_log_counter += 1
        if self._det_log_counter % 30 == 0:
            self.get_logger().info(f"YOLO detections this frame: {len(detections)}")
        return detections

    def destroy_node(self):
        if self.hailo_client is not None:
            self.hailo_client.close()
            self.hailo_client = None
        super().destroy_node()

    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        # Convert ROS image to OpenCV BGR
        cv_image = np.array(
            self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8'),
            dtype=np.uint8
        )
        h, w = cv_image.shape[:2]

        xyz_lidar = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)
        n_points = xyz_lidar.shape[0]
        if n_points == 0:
            self.get_logger().warn("Empty cloud. Nothing to project.")
            self._publish(cv_image, image_msg)
            return

        # Compute distance per point for color coding and depth matrix
        distances = np.linalg.norm(xyz_lidar, axis=1)

        # Transform LiDAR -> camera frame
        xyz_lidar_f64 = xyz_lidar.astype(np.float64)
        ones = np.ones((n_points, 1), dtype=np.float64)
        xyz_lidar_h = np.hstack((xyz_lidar_f64, ones))
        xyz_cam_h = xyz_lidar_h @ self.T_lidar_to_cam.T
        xyz_cam = xyz_cam_h[:, :3]

        # Keep only points in front of camera
        mask_in_front = (xyz_cam[:, 2] > 0.0)
        xyz_cam_front = xyz_cam[mask_in_front]
        distances_front = distances[mask_in_front]

        if xyz_cam_front.shape[0] == 0:
            self.get_logger().info("No points in front of camera (z>0).")
            self._publish(cv_image, image_msg)
            return

        # Manual projection — avoids cv2.projectPoints issues
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        x = xyz_cam_front[:, 0] / xyz_cam_front[:, 2]
        y = xyz_cam_front[:, 1] / xyz_cam_front[:, 2]
        u_proj = (fx * x + cx).astype(np.float32)
        v_proj = (fy * y + cy).astype(np.float32)

        # Build depth matrix — image-sized array storing closest LiDAR distance per pixel
        # Used to get distance for each bounding box via np.min(depth_matrix[y1:y2, x1:x2])
        depth_matrix = np.full((h, w), np.inf, dtype=np.float32)
        u_int_all = np.round(u_proj).astype(int)
        v_int_all = np.round(v_proj).astype(int)
        valid = (u_int_all >= 0) & (u_int_all < w) & (v_int_all >= 0) & (v_int_all < h)
        for i in np.where(valid)[0]:
            if distances_front[i] < depth_matrix[v_int_all[i], u_int_all[i]]:
                depth_matrix[v_int_all[i], u_int_all[i]] = distances_front[i]

        # ── YOLO Detection ──────────────────────────────────────────────────
        all_boxes = []  # list of (x1, y1, x2, y2)
        if self.enable_detection:
            for class_name, conf, x1, y1, x2, y2 in self._run_yolo_detections(cv_image, w, h):
                all_boxes.append((x1, y1, x2, y2))

                box_depths = depth_matrix[y1:y2, x1:x2]
                valid_depths = box_depths[box_depths < np.inf]

                if len(valid_depths) > 0:
                    closest = np.min(valid_depths)
                    pixel_height = y2 - y1
                    physical_height = (pixel_height * closest) / fy

                    label = f"{class_name} {closest:.2f}m H:{physical_height:.2f}m ({conf:.0%})"
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    label = f"{class_name} --m ({conf:.0%})"
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(cv_image, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 255), 2)

        # ── HSV Cone Detection (from GitHub reference) ───────────────────────
        cone_boxes = detect_cones(cv_image, self.cone_hsv_lb, self.cone_hsv_ub)
        for (cx_box, cy_box, cw, ch_box) in cone_boxes:
            x1, y1, x2, y2 = cx_box, cy_box, cx_box + cw, cy_box + ch_box
            all_boxes.append((x1, y1, x2, y2))
            box_depths = depth_matrix[y1:y2, x1:x2]
            valid_depths = box_depths[box_depths < np.inf]
            
            if len(valid_depths) > 0:
                cone_dist = np.min(valid_depths)
                # --- NEW HEIGHT CALCULATION ---
                pixel_height = y2 - y1
                physical_height = (pixel_height * cone_dist) / fy
                
                label = f"cone {cone_dist:.2f}m H:{physical_height:.2f}m"
            else:
                label = "cone --m"
                
            # Green box for HSV cone detections
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 0), 2)

        # ── LiDAR projection overlay ─────────────────────────────────────────
        # Draw all projected points (full fusion view), then highlight points
        # inside detection boxes more brightly.
        for i in range(len(u_proj)):
            u_int = int(u_proj[i] + 0.5)
            v_int = int(v_proj[i] + 0.5)
            if not (0 <= u_int < w and 0 <= v_int < h):
                continue

            inside_box = any(
                x1 <= u_int <= x2 and y1 <= v_int <= y2
                for (x1, y1, x2, y2) in all_boxes
            )
            color_intensity = int(np.clip(distances_front[i] / self.max_dist_thresh * 255, 0, 255))
            color = (0, color_intensity, 255 - color_intensity)
            radius = 3 if inside_box else 2
            if inside_box:
                cv2.circle(cv_image, (u_int, v_int), radius, color, -1)
            else:
                # Dim full-scene overlay so the camera feed + boxes stay readable.
                cv2.circle(cv_image, (u_int, v_int), 1, (color_intensity // 2, color_intensity, 128), -1)

        self._publish(cv_image, image_msg)

    def _publish(self, cv_image: np.ndarray, image_msg: Image):
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = image_msg.header
        self.pub_image.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
