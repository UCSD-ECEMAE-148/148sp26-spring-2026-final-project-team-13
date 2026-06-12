import sys
import threading
from pathlib import Path
from typing import Any, Dict, Optional

import depthai as dai
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

PACKAGE_NAME = "ros2_depthai_package"
NODE_NAME = "camera_publisher"

DEFAULT_PARAMETER_DEFAULTS: Dict[str, Any] = {
    "topic_name": "/oak/rgb/image_raw",
    "fps": 30.0,
    "width": 640,
    "height": 400,
    "frame_id": "oak_rgb_camera_frame",
    "anti_banding_mode": "mains_60_hz",
    "qos_reliability": "reliable",
    "qos_depth": 10,
}


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        startup_defaults = self._load_startup_defaults()
        self.declare_parameter("topic_name", startup_defaults["topic_name"])
        self.declare_parameter("fps", startup_defaults["fps"])
        self.declare_parameter("width", startup_defaults["width"])
        self.declare_parameter("height", startup_defaults["height"])
        self.declare_parameter("frame_id", startup_defaults["frame_id"])
        self.declare_parameter("anti_banding_mode", startup_defaults["anti_banding_mode"])
        self.declare_parameter("qos_reliability", startup_defaults["qos_reliability"])
        self.declare_parameter("qos_depth", startup_defaults["qos_depth"])

        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.width = self.get_parameter("width").get_parameter_value().integer_value
        self.height = self.get_parameter("height").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.anti_banding_mode = (
            self.get_parameter("anti_banding_mode").get_parameter_value().string_value.lower()
        )
        self.qos_reliability = (
            self.get_parameter("qos_reliability").get_parameter_value().string_value.lower()
        )
        self.qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value

        self._validate_parameters()

        qos_profile = self._make_qos_profile(self.qos_reliability, self.qos_depth)
        self.publisher = self.create_publisher(Image, self.topic_name, qos_profile)

        self.pipeline = None
        self.camera_connected = False
        self.device = None
        self.rgb_queue = None
        self.last_reconnect_attempt = 0.0
        self.reconnect_interval = 2.0

        if self._initialize_camera():
            self.get_logger().info("Camera device initialized successfully")
        else:
            self.get_logger().warning(
                "Camera not detected at startup. Please connect a DepthAI camera."
            )

        self._stop_event = threading.Event()
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        if self.camera_connected:
            self.get_logger().info(
                f"Publishing RGB frames on {self.topic_name} as sensor_msgs/msg/Image "
                f"at {self.width}x{self.height} @ {self.fps:.1f} FPS "
                f"(qos_reliability={self.qos_reliability}, qos_depth={self.qos_depth}, "
                f"anti_banding_mode={self.anti_banding_mode})"
            )
        else:
            self.get_logger().warning(
                "Node running without camera. "
                "Will attempt to reconnect when camera becomes available."
            )

    def _find_config_root(self) -> Optional[Path]:
        module_dir = Path(__file__).resolve().parent
        candidate_dirs = [module_dir.parent / "config"]
        candidate_dirs.extend(
            ancestor / "share" / PACKAGE_NAME / "config" for ancestor in module_dir.parents
        )

        seen = set()
        for candidate in candidate_dirs:
            resolved = str(candidate.resolve())
            if resolved in seen:
                continue
            seen.add(resolved)
            if (candidate / "camera.yaml").exists():
                return candidate
        return None

    def _load_yaml_parameters(self, path: Path) -> Dict[str, Any]:
        if not path.exists():
            self.get_logger().warn(f"Missing config file: {path}")
            return {}

        with path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}

        if not isinstance(loaded, dict):
            raise ValueError(f"Config file must contain a mapping: {path}")

        node_section = loaded.get(NODE_NAME, {})
        if not isinstance(node_section, dict):
            raise ValueError(f"Top-level '{NODE_NAME}' must map to a dictionary: {path}")

        params = node_section.get("ros__parameters", {})
        if not isinstance(params, dict):
            raise ValueError(f"'ros__parameters' must be a dictionary in: {path}")

        return params

    def _load_startup_defaults(self) -> Dict[str, Any]:
        defaults = dict(DEFAULT_PARAMETER_DEFAULTS)
        config_root = self._find_config_root()
        if config_root is None:
            self.get_logger().warn(
                "No package config directory found; using in-code parameter defaults."
            )
            return defaults

        merged_parameters: Dict[str, Any] = {}
        merged_parameters.update(self._load_yaml_parameters(config_root / "camera.yaml"))

        known_parameter_names = set(DEFAULT_PARAMETER_DEFAULTS.keys())
        defaults.update(
            {
                key: value
                for key, value in merged_parameters.items()
                if key in known_parameter_names
            }
        )
        defaults["topic_name"] = str(defaults["topic_name"])
        defaults["fps"] = float(defaults["fps"])
        defaults["width"] = int(defaults["width"])
        defaults["height"] = int(defaults["height"])
        defaults["frame_id"] = str(defaults["frame_id"])
        defaults["anti_banding_mode"] = str(defaults["anti_banding_mode"])
        defaults["qos_reliability"] = str(defaults["qos_reliability"])
        defaults["qos_depth"] = int(defaults["qos_depth"])

        self.get_logger().info(f"Loaded startup config from {config_root}")
        return defaults

    def _validate_parameters(self) -> None:
        if not self.topic_name:
            raise ValueError("topic_name must not be empty")
        if self.fps <= 0:
            raise ValueError("fps must be > 0")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be > 0")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if self.qos_depth <= 0:
            raise ValueError("qos_depth must be > 0")

    def _make_qos_profile(self, reliability_name: str, depth: int) -> QoSProfile:
        reliability_map = {
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }
        if reliability_name not in reliability_map:
            raise ValueError("qos_reliability must be one of: reliable, best_effort")

        return QoSProfile(
            reliability=reliability_map[reliability_name],
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(depth),
        )

    def _build_pipeline(
        self, width: int, height: int, fps: float, anti_banding_mode_name: str
    ) -> dai.Pipeline:
        anti_banding_map = {
            "off": dai.CameraControl.AntiBandingMode.OFF,
            "auto": dai.CameraControl.AntiBandingMode.AUTO,
            "mains_50_hz": dai.CameraControl.AntiBandingMode.MAINS_50_HZ,
            "mains_60_hz": dai.CameraControl.AntiBandingMode.MAINS_60_HZ,
        }
        if anti_banding_mode_name not in anti_banding_map:
            raise ValueError(
                "anti_banding_mode must be one of: off, auto, mains_50_hz, mains_60_hz"
            )

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName("rgb")
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setPreviewSize(int(width), int(height))
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(float(fps))
        cam_rgb.initialControl.setAntiBandingMode(anti_banding_map[anti_banding_mode_name])
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline

    def _initialize_camera(self) -> bool:
        try:
            if self.pipeline is None:
                self.pipeline = self._build_pipeline(
                    width=self.width,
                    height=self.height,
                    fps=self.fps,
                    anti_banding_mode_name=self.anti_banding_mode,
                )

            available_devices = []
            if hasattr(dai.Device, "getAllAvailableDevices"):
                available_devices = dai.Device.getAllAvailableDevices()
            if not available_devices:
                self.get_logger().info("No DepthAI devices detected yet")
                self.pipeline = None
                self.device = None
                self.rgb_queue = None
                self.camera_connected = False
                return False

            self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)
            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=2, blocking=True)
            self.camera_connected = True
            return True
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize camera device: {e}")
            if self.device is not None:
                try:
                    self.device.close()
                except Exception:
                    pass
            self.pipeline = None
            self.device = None
            self.rgb_queue = None
            self.camera_connected = False
            return False

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self.camera_connected:
                current_time = self.get_clock().now().nanoseconds * 1e-9
                if current_time - self.last_reconnect_attempt >= self.reconnect_interval:
                    self.last_reconnect_attempt = current_time
                    self._attempt_reconnect()
                self._stop_event.wait(timeout=0.5)
                continue

            try:
                packet = self.rgb_queue.get()
                if packet is None:
                    continue

                frame = packet.getCvFrame()
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.height = frame.shape[0]
                msg.width = frame.shape[1]
                msg.encoding = "bgr8"
                msg.is_bigendian = False
                msg.step = frame.shape[1] * frame.shape[2]
                msg.data = frame.tobytes()
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Camera disconnected during operation: {e}")
                self._handle_camera_disconnect()

    def _attempt_reconnect(self) -> None:
        if self._initialize_camera():
            self.get_logger().info("Camera reconnected successfully")
        else:
            self.get_logger().debug("Reconnection attempt failed")

    def _handle_camera_disconnect(self) -> None:
        if self.device is not None:
            try:
                self.device.close()
            except Exception as e:
                self.get_logger().debug(f"Error closing device during disconnect: {e}")
            self.device = None
        self.pipeline = None
        self.rgb_queue = None
        self.camera_connected = False
        self.get_logger().warning("Camera connection lost. Will attempt to reconnect...")

    def destroy_node(self) -> None:
        self._stop_event.set()
        if hasattr(self, "_capture_thread"):
            self._capture_thread.join(timeout=2.0)
        if hasattr(self, "device") and self.device is not None:
            try:
                self.device.close()
            except Exception as e:
                self.get_logger().warning(f"Error closing device during cleanup: {e}")
        return super().destroy_node()


def main() -> None:
    rclpy.init(args=sys.argv)
    node: Optional[CameraPublisher] = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
