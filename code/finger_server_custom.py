#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import numpy as np

from esp_link import FrameReader, open_serial, quiesce, send_coords, send_stream
from fingertip_model import FingertipDetector, DEFAULT_CONF_THRESHOLD
MAX_FLOW_STREAK = 20

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
)


class HybridTracker:

    def __init__(self, detector: FingertipDetector):
        self.detector = detector
        self.prev_gray = None
        self.prev_pt = None
        self.flow_streak = 0

    def process(self, img_bgr: np.ndarray):
        h, w = img_bgr.shape[:2]
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        found, px, py, confidence = self.detector.predict(img_bgr)

        if found:
            self.prev_pt = np.array([[[px, py]]], dtype=np.float32)
            self.flow_streak = 0
            self.prev_gray = gray
            return True, px, py, "model", confidence

        if (self.prev_gray is not None and self.prev_pt is not None
                and self.flow_streak < MAX_FLOW_STREAK):
            new_pts, status, _err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pt, None, **LK_PARAMS)
            if new_pts is not None and status is not None and status[0][0] == 1:
                px, py = new_pts[0][0]
                px, py = int(px), int(py)
                if 0 <= px < w and 0 <= py < h:
                    self.prev_pt = new_pts
                    self.flow_streak += 1
                    self.prev_gray = gray
                    return True, px, py, "flow", confidence

        self.prev_pt = None
        self.flow_streak = 0
        self.prev_gray = gray
        return False, 0, 0, None, confidence


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="port (e.g. COM11, /dev/ttyACM0)")
    ap.add_argument("--show", action="store_true", help="preview window")
    ap.add_argument("--model", default="fingertip_model.pt", help="model path")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF_THRESHOLD,
                     help="confidence threshold")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        raise SystemExit(
            f"Can't find {args.model}. Train your model:\n"
            f"  python collect_frames.py --out dataset/images\n"
            f"  python label_tool.py --images dataset/images --labels dataset/labels.json\n"
            f"  python train_heatmap_model.py --dataset dataset")

    detector = FingertipDetector(args.model, conf_threshold=args.conf)
    ser = open_serial(args.port)
    tracker = HybridTracker(detector)
    reader = FrameReader(ser)
    quiesce(ser, reader)
    send_stream(ser, True)

    frames = 0
    stats = {"model": 0, "flow": 0, "lost": 0}
    corrupt_seen = 0
    t0 = time.time()

    print("Czekam na klatki... (Ctrl+C aby zakonczyc)")
    try:
        while True:
            frame_id, jpeg = reader.read_frame()
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue

            found, px, py, source, confidence = tracker.process(img)
            send_coords(ser, frame_id, found, px, py)
            frames += 1
            stats[source if source else "lost"] += 1

            if args.show:
                if found:
                    color = (0, 0, 255) if source == "model" else (0, 200, 255)
                    cv2.circle(img, (px, py), 6, color, -1)
                    cv2.circle(img, (px, py), 10, (255, 255, 255), 2)
                    label = f"model {confidence:.2f}" if source == "model" else "flow"
                    cv2.putText(img, label, (px + 14, py - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.imshow("ESP32 finger tracker (custom model)", img)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

            if frames % 60 == 0:
                dt = time.time() - t0
                t0 = time.time()
                corrupt = reader.corrupt_frames - corrupt_seen
                corrupt_seen = reader.corrupt_frames
                warn = "  <- sprawdz naswietlanie/MAX_FPS" if corrupt > 3 else ""
                print(f"FPS: {60 / dt:5.1f} | model={stats['model']} "
                      f"flow={stats['flow']} lost={stats['lost']} "
                      f"| uszkodzone klatki: {corrupt}{warn}")
                stats = {"model": 0, "flow": 0, "lost": 0}
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send_stream(ser, False)
        except Exception:
            pass
        ser.close()
        if args.show:
            cv2.destroyAllWindows()
        print("Zakonczono.")


if __name__ == "__main__":
    main()
