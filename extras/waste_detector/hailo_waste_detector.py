#!/usr/bin/env python3
"""Hailo waste detector: a ROS 2 node for the behaviour tree.

Runs on the Raspberry Pi 5. The YOLO model runs on the Hailo AI HAT; the camera
is read locally from rpicam-vid (MJPEG on 127.0.0.1, never over WiFi).

Publishes
  /waste/class                  stable decision: metal, organic, plastic or none
  /waste/station                matching station colour: red, green, blue or none
  /waste/image/compressed       annotated JPEG preview (sensor_msgs/CompressedImage)
Services
  /waste/enable  (std_srvs/SetBool)  turn detection on (true) or off (false)
  /waste/reset   (std_srvs/Trigger)  clear the decision after offloading

Start (on the Pi, after rpicam-vid is running):
  python3 hailo_waste_detector.py --hef ~/models/waste.hef
"""
import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

CLASSES = ["metal", "organic", "plastic"]
NUM_CLASSES = len(CLASSES)
STATION = {"metal": "red", "organic": "green", "plastic": "blue"}
COLORS = {"metal": (0, 0, 255), "organic": (0, 200, 0), "plastic": (255, 0, 0)}  # BGR


# ---------------------------------------------------------------------------
# Pure image and YOLO post-processing helpers (no Hailo or ROS needed)
# ---------------------------------------------------------------------------
def letterbox(img, out_w, out_h):
    """Resize without stretching, pad with grey (114). Returns image + mapping."""
    h, w = img.shape[:2]
    scale = min(out_w / w, out_h / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((out_h, out_w, 3), 114, dtype=np.uint8)
    pad_x, pad_y = (out_w - nw) // 2, (out_h - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return canvas, scale, pad_x, pad_y


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _softmax(x, axis=-1):
    e = np.exp(x - x.max(axis=axis, keepdims=True))
    return e / e.sum(axis=axis, keepdims=True)


def describe_outputs(outputs):
    return ", ".join(f"{k}{tuple(v.shape)}" for k, v in sorted(outputs.items()))


def decode_yolo_outputs(outputs, in_w, in_h, num_classes=NUM_CLASSES):
    """Turn the six raw YOLO head outputs into boxes, scores and class ids.

    `outputs` maps output name -> array of shape (H, W, C), batch removed.
    Each scale gives one class map (C == num_classes) and one box map
    (C == 4 for distance boxes, as in YOLO26, or C == 64 for DFL boxes).
    Returns boxes (N, 4) as x1, y1, x2, y2 in model-input pixels,
    scores (N,) and class ids (N,).
    """
    by_size = {}
    for name, arr in outputs.items():
        arr = np.asarray(arr, dtype=np.float32)
        if arr.ndim != 3:
            raise ValueError(f"Unexpected output {name}{arr.shape}; outputs: "
                             f"{describe_outputs(outputs)}")
        by_size.setdefault(arr.shape[:2], {})[arr.shape[2]] = arr

    all_boxes, all_scores = [], []
    for (gh, gw), maps in by_size.items():
        cls_map = maps.get(num_classes)
        box_map = maps.get(4) if 4 in maps else maps.get(64)
        if cls_map is None or box_map is None:
            raise ValueError(f"Could not pair class/box outputs at grid {gh}x{gw}; "
                             f"outputs: {describe_outputs(outputs)}")
        stride_x, stride_y = in_w / gw, in_h / gh

        cls = cls_map.reshape(-1, num_classes)
        if cls.min() < 0.0 or cls.max() > 1.0:   # raw logits -> probabilities
            cls = _sigmoid(cls)

        if box_map.shape[2] == 64:               # DFL: 4 sides x 16 bins
            bins = _softmax(box_map.reshape(-1, 4, 16), axis=2)
            dist = (bins * np.arange(16, dtype=np.float32)).sum(axis=2)
        else:                                    # direct distances
            dist = box_map.reshape(-1, 4)

        ys, xs = np.mgrid[0:gh, 0:gw]
        ax = xs.reshape(-1).astype(np.float32) + 0.5
        ay = ys.reshape(-1).astype(np.float32) + 0.5
        x1 = (ax - dist[:, 0]) * stride_x
        y1 = (ay - dist[:, 1]) * stride_y
        x2 = (ax + dist[:, 2]) * stride_x
        y2 = (ay + dist[:, 3]) * stride_y
        all_boxes.append(np.stack([x1, y1, x2, y2], axis=1))
        all_scores.append(cls)

    boxes = np.concatenate(all_boxes)
    scores_all = np.concatenate(all_scores)
    class_ids = scores_all.argmax(axis=1)
    scores = scores_all[np.arange(len(scores_all)), class_ids]
    return boxes, scores, class_ids


def select_detections(boxes, scores, class_ids, mapping, conf_thr, nms_thr):
    """Threshold, NMS and map back to camera pixels.

    Returns a list of (label, confidence, (x, y, w, h)).
    """
    scale, pad_x, pad_y = mapping
    keep = scores > conf_thr
    boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]
    if len(scores) == 0:
        return []
    rects = []
    for x1, y1, x2, y2 in boxes:
        x = (x1 - pad_x) / scale
        y = (y1 - pad_y) / scale
        rects.append([int(x), int(y), int((x2 - x1) / scale), int((y2 - y1) / scale)])
    picked = cv2.dnn.NMSBoxes(rects, scores.tolist(), conf_thr, nms_thr)
    return [(CLASSES[class_ids[i]], float(scores[i]), tuple(rects[i]))
            for i in np.array(picked, dtype=int).flatten()]


class StableDecision:
    """Locks a class only after it wins `needed` frames in a row."""

    def __init__(self, needed):
        self.needed = needed
        self.reset()

    def reset(self):
        self.decided, self.candidate, self.count = None, None, 0

    def update(self, detections):
        """Returns the newly decided label (once), else None."""
        if self.decided is not None:
            return None
        if not detections:
            self.candidate, self.count = None, 0
            return None
        best = max(detections, key=lambda d: d[1])[0]
        if best == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = best, 1
        if self.count >= self.needed:
            self.decided = self.candidate
            return self.decided
        return None


# ---------------------------------------------------------------------------
# Hailo runner
# ---------------------------------------------------------------------------
class HailoYolo:
    def __init__(self, hef_path):
        from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams,
                                    ConfigureParams, InputVStreamParams,
                                    OutputVStreamParams, FormatType)
        self.hef = HEF(hef_path)
        self.device = VDevice()
        params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.device.configure(self.hef, params)[0]
        in_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)

        self.input_info = self.hef.get_input_vstream_infos()[0]
        self.output_infos = self.hef.get_output_vstream_infos()
        self.in_h, self.in_w = int(self.input_info.shape[0]), int(self.input_info.shape[1])

        self._pipe = InferVStreams(self.network_group, in_params, out_params)
        self._pipe.__enter__()
        self._active = self.network_group.activate(self.network_group.create_params())
        self._active.__enter__()

    def describe(self):
        lines = [f"input  {self.input_info.name} {tuple(self.input_info.shape)}"]
        lines += [f"output {o.name} {tuple(o.shape)}" for o in self.output_infos]
        return lines

    def infer(self, rgb):
        """rgb: uint8 array (in_h, in_w, 3). Returns {name: (H, W, C) float32}."""
        result = self._pipe.infer({self.input_info.name: np.ascontiguousarray(rgb[None])})
        return {name: np.asarray(arr)[0] for name, arr in result.items()}

    def close(self):
        for ctx in (self._active, self._pipe):
            try:
                ctx.__exit__(None, None, None)
            except Exception:
                pass
        try:
            self.device.release()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
class HailoWasteDetector(Node):
    def __init__(self, args):
        super().__init__("hailo_waste_detector")
        self.args = args
        self.enabled = not args.start_disabled
        self.decision = StableDecision(args.stable_frames)

        self.class_pub = self.create_publisher(String, "waste/class", 10)
        self.station_pub = self.create_publisher(String, "waste/station", 10)
        self.image_pub = self.create_publisher(CompressedImage, "waste/image/compressed", 5)
        self.create_service(SetBool, "waste/enable", self.on_enable)
        self.create_service(Trigger, "waste/reset", self.on_reset)

        self.model = HailoYolo(args.hef)
        for line in self.model.describe():
            self.get_logger().info(line)

        self.latest = None
        self.running = True
        self.frame_count = 0
        threading.Thread(target=self.grab_frames, daemon=True).start()
        self.create_timer(0.02, self.process)
        self.get_logger().info(
            f"Hailo waste detector ready ({'enabled' if self.enabled else 'disabled'}), "
            f"camera {args.source}")

    # ----- camera -----
    def grab_frames(self):
        while self.running:
            cap = cv2.VideoCapture(self.args.source, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().warn("Camera stream not available, retrying in 2 s "
                                       "(is rpicam-vid running?)")
                time.sleep(2)
                continue
            self.get_logger().info("Connected to the camera stream")
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn("Camera stream ended, reconnecting")
                    break
                self.latest = frame
            cap.release()
            time.sleep(1)

    # ----- services for the behaviour tree -----
    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if self.enabled:
            self.decision.reset()
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"Detection {response.message}")
        return response

    def on_reset(self, request, response):
        self.decision.reset()
        response.success = True
        response.message = "reset"
        self.get_logger().info("Decision reset, looking for the next item")
        return response

    # ----- main loop -----
    def process(self):
        frame = self.latest
        if frame is None:
            return
        self.latest = None
        start = time.time()

        detections = []
        if self.enabled:
            img, scale, pad_x, pad_y = letterbox(frame, self.model.in_w, self.model.in_h)
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            outputs = self.model.infer(rgb)
            boxes, scores, class_ids = decode_yolo_outputs(
                outputs, self.model.in_w, self.model.in_h)
            detections = select_detections(boxes, scores, class_ids,
                                           (scale, pad_x, pad_y),
                                           self.args.conf, self.args.nms)
            newly = self.decision.update(detections)
            if newly:
                self.get_logger().info(f"DECIDED: {newly} -> {STATION[newly]} station")

        decided = self.decision.decided
        self.class_pub.publish(String(data=decided or "none"))
        self.station_pub.publish(String(data=STATION.get(decided, "none")))

        self.frame_count += 1
        if self.frame_count % self.args.preview_every == 0:
            self.publish_preview(frame, detections, decided, time.time() - start)

    def publish_preview(self, frame, detections, decided, elapsed):
        for label, conf, (x, y, w, h) in detections:
            cv2.rectangle(frame, (x, y), (x + w, y + h), COLORS[label], 2)
            cv2.putText(frame, f"{label} {conf:.0%}", (x, max(y - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS[label], 2)
        if not self.enabled:
            status, colour = "Detection OFF", (160, 160, 160)
        elif decided:
            status = f"DECIDED: {decided.upper()} -> {STATION[decided].upper()}"
            colour = COLORS[decided]
        else:
            status = f"Looking... ({self.decision.count}/{self.decision.needed})"
            colour = (255, 255, 255)
        cv2.putText(frame, status, (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
        cv2.putText(frame, f"FPS: {1 / max(elapsed, 1e-6):.1f}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data = jpg.tobytes()
            self.image_pub.publish(msg)

    def shutdown(self):
        self.running = False
        self.model.close()


def parse_args():
    p = argparse.ArgumentParser(description="Hailo waste detector ROS 2 node")
    p.add_argument("--hef", required=True, help="path to the .hef model")
    p.add_argument("--source", default="tcp://127.0.0.1:8888",
                   help="camera stream from rpicam-vid (default: local MJPEG)")
    p.add_argument("--conf", type=float, default=0.6, help="confidence threshold")
    p.add_argument("--nms", type=float, default=0.45, help="NMS overlap threshold")
    p.add_argument("--stable-frames", type=int, default=5,
                   help="frames in a row before a class is decided")
    p.add_argument("--preview-every", type=int, default=2,
                   help="publish the preview image every N frames")
    p.add_argument("--start-disabled", action="store_true",
                   help="start with detection off until /waste/enable true")
    args, _ = p.parse_known_args()   # ignore ROS arguments
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = HailoWasteDetector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
