#!/usr/bin/env python3
"""
obstacle_avoidance_node.py

Subscribes to the fused sensor output from lidar_camera_projection_detection.py
and the raw LiDAR point cloud to perform obstacle avoidance for Donkeycar.

Architecture:
  - Listens to /sensorfusion_out2 (fused image with YOLO + depth info)
  - Listens to /livox/lidar (raw PointCloud2 from Livox Mid-360)
  - Divides the LiDAR forward cone into LEFT / CENTER / RIGHT zones
  - Publishes /obstacle/cmd with throttle + steering override
  - Publishes /obstacle/debug_image for visualization

Decision logic:
  CLEAR   → pass through (no override)
  STEER   → obstacle in center zone, one side is clear → steer that way
  STOP    → obstacle in center + both sides blocked → throttle = 0

Integrate into Donkeycar manage.py by subscribing to /obstacle/cmd
and using it to override pilot/throttle and pilot/steering.
"""

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Reuse the fast point cloud parser from your existing fusion code
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
    raw = np.frombuffer(cloud_msg.data, dtype=dtype)
    pts = np.zeros((raw.shape[0], 3), dtype=np.float32)
    pts[:, 0] = raw['x']
    pts[:, 1] = raw['y']
    pts[:, 2] = raw['z']
    if skip_rate > 1:
        pts = pts[::skip_rate]
    return pts


# ─────────────────────────────────────────────────────────────────────────────
# Zone definitions (all angles in radians, measured from forward = +X axis)
#
#  Livox Mid-360 coordinate frame (standard):
#    +X = forward
#    +Y = left
#    +Z = up
#
#  yaw = atan2(y, x)
#  pitch = atan2(z, sqrt(x^2 + y^2))
#
#  We only care about horizontal (yaw) zones and filter by pitch to avoid
#  picking up ground points or ceiling hits.
# ─────────────────────────────────────────────────────────────────────────────

class ObstacleZone:
    """Defines a horizontal angular zone in the LiDAR frame."""
    def __init__(self, yaw_min_deg: float, yaw_max_deg: float,
                 pitch_min_deg: float = -15.0, pitch_max_deg: float = 20.0,
                 min_range: float = 0.15, max_range: float = 4.0):
        self.yaw_min   = np.radians(yaw_min_deg)
        self.yaw_max   = np.radians(yaw_max_deg)
        self.pitch_min = np.radians(pitch_min_deg)
        self.pitch_max = np.radians(pitch_max_deg)
        self.min_range = min_range
        self.max_range = max_range

    def nearest(self, points: np.ndarray) -> float:
        """
        Return the nearest point distance (meters) inside this zone.
        points: Nx3 array in LiDAR frame (x=fwd, y=left, z=up)
        Returns float('inf') if no points fall in the zone.
        """
        if points is None or len(points) == 0:
            return float('inf')

        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        dist   = np.sqrt(x**2 + y**2 + z**2)
        xy_dist = np.sqrt(x**2 + y**2)

        yaw   = np.arctan2(y, x)
        pitch = np.arctan2(z, np.maximum(xy_dist, 1e-6))

        mask = (
            (yaw   >= self.yaw_min)   & (yaw   <= self.yaw_max) &
            (pitch >= self.pitch_min) & (pitch <= self.pitch_max) &
            (dist  >= self.min_range) & (dist  <= self.max_range)
        )

        valid = dist[mask]
        return float(np.min(valid)) if valid.size > 0 else float('inf')


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class ObstacleAvoidanceNode(Node):
    """
    Fuses LiDAR point cloud with the camera-lidar fused image to perform
    real-time obstacle avoidance for Donkeycar.

    Topics consumed:
      /livox/lidar          — raw PointCloud2 (Livox Mid-360)
      /sensorfusion_out2    — fused image from lidar_camera_projection_detection.py

    Topics published:
      /obstacle/cmd         — JSON-like string: throttle, steering, state
      /obstacle/debug_image — annotated camera image showing zone distances
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # ── Tunable parameters (expose as ROS params for live tuning) ────────
        self.declare_parameter('stop_distance',  0.50)   # meters — hard stop
        self.declare_parameter('steer_distance', 1.20)   # meters — begin steering
        self.declare_parameter('max_steer',      0.60)   # steering output [-1,1]
        self.declare_parameter('slow_throttle',  0.15)   # throttle when steering around
        self.declare_parameter('lidar_skip',     1)      # point skip rate (1=all)
        self.declare_parameter('lidar_topic',    '/livox/lidar')
        self.declare_parameter('image_topic',    '/sensorfusion_out2')

        self.stop_dist   = self.get_parameter('stop_distance').value
        self.steer_dist  = self.get_parameter('steer_distance').value
        self.max_steer   = self.get_parameter('max_steer').value
        self.slow_thr    = self.get_parameter('slow_throttle').value
        self.skip_rate   = self.get_parameter('lidar_skip').value
        lidar_topic      = self.get_parameter('lidar_topic').value
        image_topic      = self.get_parameter('image_topic').value

        # ── Zone definitions ──────────────────────────────────────────────────
        # Divide the forward hemisphere into three vertical slices.
        # Adjust yaw bounds based on your car's geometry and how the
        # Livox is physically mounted.
        #
        #        LEFT  |  CENTER  |  RIGHT
        #       +10..30 | -10..+10 | -30..-10   (degrees)
        #
        # Pitch bounds (-15 to +20 deg) exclude ground hits and ceiling.
        # Raise pitch_min if you still get ground returns at speed.
        self.zone_center = ObstacleZone(
            yaw_min_deg=-10, yaw_max_deg=10,
            pitch_min_deg=-12, pitch_max_deg=20,
            min_range=0.15, max_range=self.steer_dist + 0.5
        )
        self.zone_left = ObstacleZone(
            yaw_min_deg=10,  yaw_max_deg=35,
            pitch_min_deg=-12, pitch_max_deg=20,
            min_range=0.15, max_range=self.steer_dist + 0.5
        )
        self.zone_right = ObstacleZone(
            yaw_min_deg=-35, yaw_max_deg=-10,
            pitch_min_deg=-12, pitch_max_deg=20,
            min_range=0.15, max_range=self.steer_dist + 0.5
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.latest_points  = None   # most recent Nx3 LiDAR array
        self.avoidance_state = "CLEAR"
        self.bridge = CvBridge()

        # ── Subscribers ───────────────────────────────────────────────────────
        # LiDAR and fused image are time-synced within 100ms.
        # If sync is too strict and you drop messages, increase slop.
        self.lidar_sub = Subscriber(self, PointCloud2, lidar_topic)
        self.image_sub = Subscriber(self, Image,       image_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=5,
            slop=0.10
        )
        self.ts.registerCallback(self.sync_callback)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(String, '/obstacle/cmd',         1)
        self.debug_pub = self.create_publisher(Image,  '/obstacle/debug_image', 1)

        self.get_logger().info(
            f"ObstacleAvoidanceNode ready. "
            f"stop={self.stop_dist}m  steer={self.steer_dist}m  "
            f"max_steer={self.max_steer}"
        )

    # ── Main callback ─────────────────────────────────────────────────────────

    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):

        # 1. Parse point cloud
        points = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)

        # 2. Compute zone distances from LiDAR
        d_center = self.zone_center.nearest(points)
        d_left   = self.zone_left.nearest(points)
        d_right  = self.zone_right.nearest(points)

        # 3. Decision logic
        throttle, steering, state = self._decide(d_center, d_left, d_right)

        # 4. Publish command string (parse this in manage.py or a bridge node)
        cmd_str = (
            f"state={state},"
            f"throttle={throttle:.3f},"
            f"steering={steering:.3f},"
            f"d_center={d_center:.2f},"
            f"d_left={d_left:.2f},"
            f"d_right={d_right:.2f}"
        )
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)

        # 5. Debug image overlay
        try:
            cv_image = np.array(
                self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8'),
                dtype=np.uint8
            )
            debug = self._draw_debug(cv_image, d_center, d_left, d_right, state, steering)
            out_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out_msg.header = image_msg.header
            self.debug_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Debug image failed: {e}")

    # ── Decision logic ────────────────────────────────────────────────────────

    def _decide(self, d_center: float, d_left: float, d_right: float):
        """
        Returns (throttle, steering, state_string).

        STOP  — obstacle too close on all three zones
        STEER — obstacle in center, but one side has clearance
        CLEAR — nothing blocking forward path
        """

        # Hard stop — blocked everywhere
        if (d_center < self.stop_dist and
                d_left  < self.stop_dist and
                d_right < self.stop_dist):
            return 0.0, 0.0, "STOP"

        # Steer — center blocked, pick the clearer side
        if d_center < self.steer_dist:
            if d_left >= d_right:
                # More room on the left → steer left (positive steering)
                steer = self.max_steer
                side  = "STEER_LEFT"
            else:
                # More room on the right → steer right (negative steering)
                steer = -self.max_steer
                side  = "STEER_RIGHT"
            return self.slow_thr, steer, side

        # Clear — no intervention
        return 0.0, 0.0, "CLEAR"
        # NOTE: returning 0.0 throttle for CLEAR means the pilot retains control.
        # In your Donkeycar bridge, only apply the override when state != "CLEAR".

    # ── Debug visualization ───────────────────────────────────────────────────

    def _draw_debug(self, img: np.ndarray,
                    d_center: float, d_left: float, d_right: float,
                    state: str, steering: float) -> np.ndarray:
        """
        Draw three zone distance bars and state text on the fused camera image.
        Colors: green=safe, yellow=caution, red=danger
        """
        out = img.copy()
        h, w = out.shape[:2]

        def dist_color(d: float) -> tuple:
            if d < self.stop_dist:
                return (0, 0, 220)      # red — stop zone
            elif d < self.steer_dist:
                return (0, 180, 255)    # orange — steer zone
            else:
                return (0, 210, 60)     # green — clear

        # Draw three zone overlays as semi-transparent bars at the bottom
        overlay = out.copy()
        zone_h  = 60
        bar_y   = h - zone_h
        third   = w // 3

        # Left zone
        cv2.rectangle(overlay, (0, bar_y), (third, h),
                      dist_color(d_left), -1)
        # Center zone
        cv2.rectangle(overlay, (third, bar_y), (2 * third, h),
                      dist_color(d_center), -1)
        # Right zone
        cv2.rectangle(overlay, (2 * third, bar_y), (w, h),
                      dist_color(d_right), -1)

        cv2.addWeighted(overlay, 0.40, out, 0.60, 0, out)

        # Distance text inside each bar
        font = cv2.FONT_HERSHEY_SIMPLEX
        def fmt(d): return f"{d:.1f}m" if d < float('inf') else "--"

        cv2.putText(out, fmt(d_left),   (10,          h - 15), font, 0.65, (255,255,255), 2)
        cv2.putText(out, fmt(d_center), (third + 10,  h - 15), font, 0.65, (255,255,255), 2)
        cv2.putText(out, fmt(d_right),  (2*third + 10,h - 15), font, 0.65, (255,255,255), 2)

        # Zone labels
        cv2.putText(out, "LEFT",   (10,          h - 40), font, 0.45, (200,200,200), 1)
        cv2.putText(out, "CENTER", (third + 10,  h - 40), font, 0.45, (200,200,200), 1)
        cv2.putText(out, "RIGHT",  (2*third + 10,h - 40), font, 0.45, (200,200,200), 1)

        # State banner at top
        state_colors = {
            "CLEAR":       (0, 210, 60),
            "STEER_LEFT":  (0, 180, 255),
            "STEER_RIGHT": (0, 180, 255),
            "STOP":        (0, 0, 220),
        }
        color = state_colors.get(state, (200, 200, 200))
        cv2.rectangle(out, (0, 0), (w, 36), color, -1)
        cv2.putText(out, f"OBSTACLE: {state}  steer={steering:+.2f}",
                    (8, 26), font, 0.75, (255, 255, 255), 2)

        # Divider lines between zones
        cv2.line(out, (third,     bar_y), (third,     h), (255,255,255), 1)
        cv2.line(out, (2*third,   bar_y), (2*third,   h), (255,255,255), 1)

        return out


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()