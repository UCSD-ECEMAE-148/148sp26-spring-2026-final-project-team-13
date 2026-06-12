"""Python 3.10 client that delegates YOLO inference to a Python 3.11 Hailo worker."""

from __future__ import annotations

import os
import pickle
import struct
import subprocess
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

DetectionBox = Tuple[str, float, int, int, int, int]


def kill_stale_hailo_bridges():
    """Release /dev/hailo0 by terminating orphaned bridge workers."""
    subprocess.run(
        ["pkill", "-9", "-f", "hailo_detect_bridge"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(0.5)


class HailoYoloClient:
    """Spawn a persistent python3.11 bridge process for Hailo NPU inference."""

    def __init__(
        self,
        hef_path: str,
        conf_thresh: float = 0.4,
        python311: Optional[str] = None,
        logger=None,
    ):
        self.hef_path = hef_path
        self.conf_thresh = conf_thresh
        self._logger = logger
        self._proc: Optional[subprocess.Popen] = None
        self._python311 = python311 or os.environ.get("SENSORFUSION_HAILO_PYTHON", "python3.11")
        self._bridge_script = os.path.join(os.path.dirname(__file__), "hailo_detect_bridge.py")
        self._site_packages = os.path.dirname(os.path.dirname(__file__))
        self._start_bridge()

    def _start_bridge(self):
        kill_stale_hailo_bridges()
        env = os.environ.copy()
        env["PYTHONPATH"] = self._site_packages + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONNOUSERSITE"] = "1"
        proc = subprocess.Popen(
            [self._python311, self._bridge_script, self.hef_path, str(self.conf_thresh)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            env=env,
        )
        self._proc = proc
        try:
            ready = self._read_msg()
            if ready.get("status") != "ready":
                stderr = proc.stderr.read().decode() if proc.stderr else ""
                raise RuntimeError(f"Hailo bridge failed to start: {ready} {stderr}")
        except Exception:
            self.close()
            raise
        if self._logger:
            self._logger.info(
                f"Hailo NPU detection ready ({self.hef_path}) via {self._python311}"
            )

    def _restart_bridge_if_needed(self):
        if self._proc is None or self._proc.poll() is not None:
            if self._logger:
                self._logger.warn("Restarting Hailo bridge...")
            self.close()
            self._start_bridge()

    def _read_msg(self) -> dict:
        assert self._proc and self._proc.stdout
        header = self._proc.stdout.read(4)
        if not header:
            stderr = self._proc.stderr.read().decode() if self._proc.stderr else ""
            raise RuntimeError(f"Hailo bridge died: {stderr}")
        (length,) = struct.unpack(">I", header)
        return pickle.loads(self._proc.stdout.read(length))

    def _write_msg(self, obj: dict):
        assert self._proc and self._proc.stdin
        payload = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
        self._proc.stdin.write(struct.pack(">I", len(payload)))
        self._proc.stdin.write(payload)
        self._proc.stdin.flush()

    def detect(self, bgr_image: np.ndarray) -> List[DetectionBox]:
        self._restart_bridge_if_needed()
        h, w = bgr_image.shape[:2]
        try:
            self._write_msg({"cmd": "detect", "h": h, "w": w, "image": bgr_image.tobytes()})
            resp = self._read_msg()
        except (BrokenPipeError, RuntimeError):
            self._restart_bridge_if_needed()
            self._write_msg({"cmd": "detect", "h": h, "w": w, "image": bgr_image.tobytes()})
            resp = self._read_msg()
        if "error" in resp:
            raise RuntimeError(resp["error"])
        return resp.get("boxes", [])

    def close(self):
        proc = self._proc
        if proc is None:
            return
        self._proc = None
        if proc.poll() is None:
            try:
                payload = pickle.dumps({"cmd": "shutdown"}, protocol=pickle.HIGHEST_PROTOCOL)
                proc.stdin.write(struct.pack(">I", len(payload)))
                proc.stdin.write(payload)
                proc.stdin.flush()
            except Exception:
                pass
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
        kill_stale_hailo_bridges()

    def __del__(self):
        self.close()


def hailo_device_present() -> bool:
    return os.path.exists("/dev/hailo0")


def detect_hailo_arch() -> Optional[str]:
    """Return 'hailo8', 'hailo8l', or None."""
    try:
        result = subprocess.run(
            ["hailortcli", "fw-control", "identify"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        out = (result.stdout + result.stderr).lower()
        if "hailo-8l" in out or "hailo8l" in out:
            return "hailo8l"
        if "hailo-8" in out or "hailo8" in out:
            return "hailo8"
    except Exception:
        pass
    return None


def resolve_hailo_hef_model(arch: Optional[str] = None) -> Optional[str]:
    env_model = os.environ.get("SENSORFUSION_DETECTION_MODEL", "").strip()
    if env_model and os.path.isfile(env_model):
        return env_model

    model_dir = os.environ.get("SENSORFUSION_MODEL_DIR", "/usr/share/hailo-models")
    arch = arch or detect_hailo_arch() or "hailo8"

    candidates = []
    if arch == "hailo8l":
        candidates.extend([
            os.path.join(model_dir, "yolov8s_h8l.hef"),
            os.path.join(model_dir, "yolov8n_h8l.hef"),
        ])
    else:
        candidates.extend([
            os.path.join(model_dir, "yolov8s_h8.hef"),
            os.path.join(model_dir, "yolov6n_h8.hef"),
        ])

    for path in candidates:
        if os.path.isfile(path):
            return path
    return None
