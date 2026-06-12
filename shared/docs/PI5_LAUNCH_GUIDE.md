# Raspberry Pi 5 — Sensor Fusion Launch Guide

CPU-only pipeline: **OAK-D camera + Livox MID-360 LiDAR + fusion + Foxglove Studio**.

---

## 1. Prerequisites

| Requirement | Notes |
|-------------|-------|
| Raspberry Pi 5 (ARM64) | Tested on Pi 5 |
| Docker | `docker ps` must work (daemon running) |
| Docker images built | See Section 2 (one-time) |
| OAK-D on USB | `lsusb \| grep 03e7` should show `Intel Movidius` / `Luxonis` |
| Livox on Ethernet (`eth0`) | Dedicated cable to Pi Ethernet port |
| Foxglove Studio | Browser: [https://app.foxglove.dev](https://app.foxglove.dev) |

**ROS 2 on the Pi host is optional.** All nodes run inside Docker containers.

**Docker Compose is optional.** Launch scripts use plain `docker run`. Compose is only needed if you prefer `build_all.sh` with the compose plugin installed.

---

## 2. One-Time Setup

### 2.1 Build Docker images

If images already exist (`docker images | grep ece191`), skip this step.

```bash
bash ~/sensorfusion_ws/shared/build_all.sh
```

If `build_all.sh` fails with `unknown shorthand flag: 'f' in -f`, Docker Compose is not installed. Build manually:

```bash
# Camera
cd ~/sensorfusion_ws/camera
docker build \
  --build-arg ROS_DISTRO=humble \
  --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-ros-base-jammy \
  -t ece191/camera:humble \
  -f Dockerfile.camera .

# LiDAR
cd ~/sensorfusion_ws/lidar
docker build \
  --build-arg ROS_BASE_IMAGE=arm64v8/ros:humble-perception-jammy \
  -t ece191/lidar:humble \
  -f ../camera/Dockerfile.lidar .

# Fusion (Pi 5 / CPU-only)
bash ~/sensorfusion_ws/fusion/docker_rpi5/build.sh
```

Verify:

```bash
docker images | grep -E 'ece191|ros2_camera_lidar_fusion'
```

Expected tags: `ece191/camera:humble`, `ece191/lidar:humble`, `ros2_camera_lidar_fusion:rpi5`.

### 2.2 Configure LiDAR Ethernet (one-time)

With the Livox powered on and Ethernet connected to `eth0`:

```bash
sudo bash ~/sensorfusion_ws/shared/setup_livox_network.sh eth0 192.168.1.50
```

Verify:

```bash
ip -4 addr show dev eth0    # should show 192.168.1.50/24
ping -c 2 192.168.1.3       # your sensor IP (adjust if different)
```

---

## 3. Launch the Full Stack

### 3.1 Standard launch (camera + LiDAR + projection + Foxglove)

```bash
bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

Replace `192.168.1.3` with your Livox sensor IP, or use the serial suffix (e.g. `99` → `192.168.1.199`).

**Keep the terminal open.** Closing it or pressing Ctrl+C stops the entire stack.

### 3.2 Launch with YOLO + cone detection (Jetson `launch.sh 6` equivalent)

```bash
FUSION_MODE=detection bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

This runs `lidar_camera_projection_detection` and publishes `/sensorfusion_out2`.

When a **Hailo AI Hat** is attached (`/dev/hailo0` present), the launcher auto-selects the **NPU backend** and uses a pre-built `.hef` model (e.g. `/usr/share/hailo-models/yolov8s_h8.hef` on Hailo-8). Expect much higher FPS than CPU YOLO. HSV cone detection always runs.

Force CPU YOLO instead:

```bash
SENSORFUSION_DETECTION_BACKEND=cpu FUSION_MODE=detection bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

### 3.3 Auto-configure LiDAR network on launch

```bash
SETUP_LIVOX_NETWORK=1 bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

---

## 4. Connect Foxglove Studio

1. Open [https://app.foxglove.dev](https://app.foxglove.dev)
2. **Open connection** → **Foxglove WebSocket**
3. Enter the Pi **WiFi IP** (not the LiDAR Ethernet IP):

```bash
hostname -I
# Example: ws://192.168.139.223:8765
```

The launcher prints the correct URL when the stack starts. Use the **WiFi IP** when connecting from a laptop on the same wireless network.

| URL | When to use |
|-----|-------------|
| `ws://192.168.139.xxx:8765` | Laptop on WiFi (usual case) |
| `ws://192.168.1.50:8765` | Machine on the LiDAR Ethernet subnet only |

---

## 5. Foxglove Panel Setup

Topics appear in the left sidebar once connected, but you must **add panels** to visualize data.

**Detection mode (`FUSION_MODE=detection`):** use an **Image** panel on **`/sensorfusion_out2`**, not `/oak/rgb/image_raw`. The fused topic includes the camera feed, LiDAR point overlay, red YOLO boxes, and green cone boxes. `/oak/rgb/image_raw` is the raw camera only.

### Camera feed

| Setting | Value |
|---------|-------|
| Panel type | **Image** |
| Topic | `/oak/rgb/image_raw` |

You should see an Hz counter (~5–8 FPS on Pi 5) next to the topic name.

### LiDAR point cloud

| Setting | Value |
|---------|-------|
| Panel type | **3D** |
| Add topic | `/livox/lidar` (`sensor_msgs/PointCloud2`) |
| Fixed frame | `livox_frame` |
| Point size | 3–5 |

### Fusion output (projection mode)

| Setting | Value |
|---------|-------|
| Panel type | **Image** |
| Topic | `/sensorfusion_out` |

### Fusion + detection (detection mode)

| Setting | Value |
|---------|-------|
| Panel type | **Image** |
| Topic | `/sensorfusion_out2` |

---

## 6. Stop the Stack

```bash
bash ~/sensorfusion_ws/shared/stop_all.sh rpi5
```

Or press **Ctrl+C** in the terminal running `start_all_rpi5.sh`.

---

## 7. Manual / Component Launch (optional)

### Camera only

```bash
bash ~/sensorfusion_ws/shared/launch_camera.sh
```

### LiDAR only

```bash
export LIVOX_HOST_IP=192.168.1.50
bash ~/sensorfusion_ws/shared/launch_lidar.sh 192.168.1.3
```

### Fusion node inside container (Jetson-style `launch.sh`)

After the fusion container is running:

```bash
docker exec -it ros2_camera_lidar_fusion_rpi5 bash

# Inside container:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
bash /ros2_ws/src/ros2_camera_lidar_fusion/docker_rpi5/launch.sh 5   # projection
bash /ros2_ws/src/ros2_camera_lidar_fusion/docker_rpi5/launch.sh 6   # YOLO + cones
```

### Foxglove bridge only

```bash
bash ~/sensorfusion_ws/shared/foxglove_bridge.sh 8765
```

---

## 8. Verify Data Is Flowing

```bash
# Containers running?
docker ps

# Foxglove port open?
ss -tlnp | grep 8765

# Camera rate (inside camera container)
docker exec sensorfusion-camera bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   timeout 6 ros2 topic hz /oak/rgb/image_raw'

# LiDAR rate (inside lidar container)
docker exec $(docker ps --format "{{.Names}}" | grep "^lidar_foxy_host_" | head -1) bash -lc \
  'source /opt/ros/humble/setup.bash && source /home/devuser/livox_ws/install/setup.bash && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   timeout 6 ros2 topic hz /livox/lidar'
```

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Foxglove: "WebSocket server not reachable" | Stack not running or port 8765 closed | Relaunch `start_all_rpi5.sh`; check `ss -tlnp \| grep 8765` |
| Topics listed but no Hz / no image | Panel not subscribed to topic | Add topic to Image or 3D panel (Section 5) |
| `ws://192.168.1.50:8765` fails from WiFi laptop | Wrong IP for your network | Use WiFi IP from `hostname -I` |
| LiDAR bind failure | `eth0` missing `192.168.1.50` | Run `setup_livox_network.sh` |
| Camera: "No DepthAI devices" | OAK-D not on USB | Plug into USB 3.0; verify `lsusb \| grep 03e7`; run `bash shared/restart_camera.sh` |
| Camera: "Waiting for image messages" in Foxglove | No publisher on `/oak/rgb/image_raw` | Run `bash shared/verify_camera.sh` |
| `lsusb` has no `03e7:` device | Camera not connected / powered | None of the listed USB devices is the OAK-D — see Section 11 |
| `build_all.sh` fails on `-f` | Docker Compose not installed | Use manual `docker build` (Section 2.1) |
| Detection mode slow on Pi 5 | YOLO on CPU | Attach AI Hat; confirm `/dev/hailo0` and use Hailo backend (auto). Or force CPU with `SENSORFUSION_DETECTION_BACKEND=cpu` |

---

## 10. Quick Reference

```bash
# Full stack (projection)
bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3

# Full stack (YOLO + cones — Jetson launch.sh 6 equivalent)
FUSION_MODE=detection bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3

# Stop
bash ~/sensorfusion_ws/shared/stop_all.sh rpi5

# Foxglove
# → https://app.foxglove.dev
# → ws://<pi-wifi-ip>:8765
```

## 11. Identifying the OAK-D on USB

Run `lsusb`. The OAK-D / OAK-D Pro appears as **vendor ID `03e7`** (Luxonis / Intel Movidius):

```
Bus 003 Device 004: ID 03e7:2485 Intel Movidius MyriadX    ← this is the OAK-D
```

**Not the camera** (common on this robot):

| lsusb line | Device |
|------------|--------|
| `05e3:0610` / `05e3:0626` | USB hub |
| `0483:5740` | STMicro Virtual COM (often LiDAR-related serial) |
| `2341:8036` | Arduino Leonardo |

Verify camera health:

```bash
bash ~/sensorfusion_ws/shared/verify_camera.sh
```

Restart camera after plugging in USB:

```bash
bash ~/sensorfusion_ws/shared/restart_camera.sh
```

---

**Published topics:**

| Topic | Description |
|-------|-------------|
| `/oak/rgb/image_raw` | Camera RGB |
| `/livox/lidar` | LiDAR point cloud |
| `/livox/imu` | LiDAR IMU |
| `/sensorfusion_out` | LiDAR projected on image (mode 5) |
| `/sensorfusion_out2` | Projection + YOLO + cones (mode 6) |
