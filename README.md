# sensorfusion_ws — Sensor Fusion Workspace

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
FUSION_MODE=detection bash ~/sensorfusion_ws/shared/start_all_rpi5.sh 192.168.1.3
```

## Directory layout
