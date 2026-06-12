"""Hailo NPU YOLO inference engine (requires Python 3.11 + hailo_platform)."""

from __future__ import annotations

import os
from typing import List, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from hailo_platform import (
    ConfigureParams,
    FormatType,
    HEF,
    HailoStreamInterface,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

DetectionBox = Tuple[str, float, int, int, int, int]


class HailoYoloEngine:
    """Persistent Hailo-8 / Hailo-8L YOLO detector using a compiled .hef model."""

    def __init__(self, hef_path: str, conf_thresh: float = 0.25, imgsz: int = 640):
        if not os.path.isfile(hef_path):
            raise FileNotFoundError(f"Hailo model not found: {hef_path}")

        self.hef_path = hef_path
        self.conf_thresh = conf_thresh
        self.imgsz = imgsz

        hef = HEF(hef_path)
        self._input_name = hef.get_input_vstream_infos()[0].name
        self._output_name = hef.get_output_vstream_infos()[0].name

        params = VDevice.create_params()
        self._target = VDevice(params)
        configure_params = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe
        )
        self._network_group = self._target.configure(hef, configure_params)[0]
        self._network_group_params = self._network_group.create_params()

        input_params = InputVStreamParams.make(
            self._network_group, quantized=False, format_type=FormatType.FLOAT32
        )
        output_params = OutputVStreamParams.make(
            self._network_group, quantized=False, format_type=FormatType.FLOAT32
        )

        self._infer_ctx = InferVStreams(
            self._network_group, input_params, output_params, tf_nms_format=True
        )
        self._pipeline = self._infer_ctx.__enter__()
        self._pipeline.set_nms_score_threshold(conf_thresh)

    def close(self):
        if self._infer_ctx is not None:
            self._infer_ctx.__exit__(None, None, None)
            self._infer_ctx = None
            self._pipeline = None

    def detect(self, bgr_image: np.ndarray) -> List[DetectionBox]:
        """Run NPU inference. Returns (class_name, conf, x1, y1, x2, y2) in image coords."""
        h, w = bgr_image.shape[:2]
        rgb = bgr_image[:, :, ::-1]
        resized = self._resize_rgb(rgb, self.imgsz)
        inp = np.expand_dims(resized.astype(np.float32), axis=0)

        with self._network_group.activate(self._network_group_params):
            raw = self._pipeline.infer({self._input_name: inp})

        batch = np.array(raw[self._output_name])[0]  # (num_classes, 5, max_dets)
        boxes: List[DetectionBox] = []
        normalized = float(np.max(batch[:, :4, :])) <= 1.5

        for cls_idx in range(batch.shape[0]):
            cls_name = COCO_NAMES[cls_idx] if cls_idx < len(COCO_NAMES) else f"class_{cls_idx}"
            for det_i in range(batch.shape[2]):
                y1, x1, y2, x2, score = batch[cls_idx, :, det_i]
                score = float(score)
                if score < self.conf_thresh:
                    continue
                x1_i = self._to_pixel(x1, w, normalized)
                y1_i = self._to_pixel(y1, h, normalized)
                x2_i = self._to_pixel(x2, w, normalized)
                y2_i = self._to_pixel(y2, h, normalized)
                if x2_i <= x1_i or y2_i <= y1_i:
                    continue
                boxes.append((cls_name, score, x1_i, y1_i, x2_i, y2_i))

        return boxes

    def _to_pixel(self, value: float, dim: int, normalized: bool) -> int:
        if normalized:
            return int(max(0, min(dim, value * dim)))
        return int(max(0, min(dim, value * dim / self.imgsz)))

    @staticmethod
    def _resize_rgb(rgb: np.ndarray, size: int) -> np.ndarray:
        h, w = rgb.shape[:2]
        if h == size and w == size:
            return rgb
        if cv2 is not None:
            return cv2.resize(rgb, (size, size), interpolation=cv2.INTER_LINEAR)
        row_idx = (np.linspace(0, h - 1, size)).astype(np.int32)
        col_idx = (np.linspace(0, w - 1, size)).astype(np.int32)
        return rgb[row_idx][:, col_idx]
