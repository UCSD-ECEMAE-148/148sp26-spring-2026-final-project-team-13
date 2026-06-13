# sensorfusion_ws — Sensor Fusion Workspace

<img width="1371" height="321" alt="image" src="https://github.com/user-attachments/assets/f02e7d83-d3de-40c9-b7c2-634e2478a389" />


<img width="1918" height="832" alt="image" src="https://github.com/user-attachments/assets/8862df8f-a33e-4b5a-be05-50b64fe6a98d" />


Unified workspace for the OAK-D Pro + Livox MID-360 sensor fusion pipeline.

The workspace includes a CPU-only Raspberry Pi 5 path under `fusion/docker_rpi5`
for running the fusion nodes without CUDA or Jetson L4T.

## Quick start (Raspberry Pi 5)

```bash
# 1. Build all Docker images
bash ~/sensorfusion_ws/shared/build_all.sh

# 2. Configure LiDAR Ethernet (one-time; requires LiDAR cable connected)
sudo bash ~/sensorfusion_ws/shared/setup_livox_network.sh eth0 192.168.1.50

# 3. Launch the full stack (camera + LiDAR + fusion + Foxglove)
bash ~/sensorfusion_ws/shared/start_all_rpi5.sh <sensor-id>
```

Replace `<sensor-id>` with the last two digits of your Livox MID-360 serial number
(e.g. `50` → sensor IP `192.168.1.150`).

Connect Foxglove Studio to `ws://<pi-wifi-ip>:8765`.

Full step-by-step guide: [shared/docs/PI5_LAUNCH_GUIDE.md](shared/docs/PI5_LAUNCH_GUIDE.md)

**YOLO + cone detection (Jetson `launch.sh 6` equivalent):**
```bash
# Example: MAE/ECE 148 Spring 2026 One Line launched all
SENSORFUSION_DETECTION_BACKEND=cpu FUSION_MODE=detection bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

**To STOP ALL working nodes cleanly (recommended):**
```
FUSION_MODE=detection bash ~/sensorfusion_ws/shared/stop_all_rpi5.sh 192.168.1.3
```
