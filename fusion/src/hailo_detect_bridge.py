#!/usr/bin/env python3.11
"""Subprocess worker: runs Hailo YOLO on Python 3.11 for ROS nodes on Python 3.10."""

from __future__ import annotations

import os
import pickle
import struct
import sys
import traceback

# Isolate the pickle IPC channel from HailoRT/library stdout noise.
_IPC_OUT = os.fdopen(os.dup(1), "wb", buffering=0)
os.dup2(sys.stderr.fileno(), 1)

# Allow imports from the colcon install tree when launched as a script.
_bridge_dir = os.path.dirname(os.path.abspath(__file__))
_site_packages = os.path.dirname(_bridge_dir)
if _site_packages not in sys.path:
    sys.path.insert(0, _site_packages)

import numpy as np

from ros2_camera_lidar_fusion.hailo_yolo_engine import HailoYoloEngine


def _read_msg():
    header = sys.stdin.buffer.read(4)
    if not header:
        return None
    (length,) = struct.unpack(">I", header)
    return pickle.loads(sys.stdin.buffer.read(length))


def _write_msg(obj):
    payload = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    _IPC_OUT.write(struct.pack(">I", len(payload)))
    _IPC_OUT.write(payload)
    _IPC_OUT.flush()


def main():
    if len(sys.argv) < 2:
        sys.stderr.write("usage: hailo_detect_bridge.py <model.hef> [conf_thresh]\n")
        sys.exit(1)

    hef_path = sys.argv[1]
    conf_thresh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4

    engine = HailoYoloEngine(hef_path, conf_thresh=conf_thresh)
    _write_msg({"status": "ready", "hef": hef_path})

    try:
        while True:
            msg = _read_msg()
            if msg is None:
                break
            if msg.get("cmd") == "shutdown":
                break
            if msg.get("cmd") != "detect":
                _write_msg({"error": f"unknown cmd: {msg.get('cmd')}"})
                continue

            h, w = int(msg["h"]), int(msg["w"])
            bgr = np.frombuffer(msg["image"], dtype=np.uint8).reshape(h, w, 3)
            boxes = engine.detect(bgr)
            _write_msg({"boxes": boxes})
    except Exception:
        _write_msg({"error": traceback.format_exc()})
        sys.exit(1)
    finally:
        engine.close()


if __name__ == "__main__":
    main()
