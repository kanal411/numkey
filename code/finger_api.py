#!/usr/bin/env python3
import argparse
import collections
import contextlib
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from esp_link import (AUTO_EXPOSURE, ST_FAILED, ST_HOMED, STATE_NAMES, FrameReader,
                      describe_switches, open_serial, quiesce, send_coords,
                      send_exposure, send_home, send_stream, send_target, send_text)
from fingertip_model import DEFAULT_CONF_THRESHOLD, FingertipDetector

MAX_X = 7750
MAX_Y = 12700

HOME_POSITION = (MAX_X, MAX_Y)

FRAME_BUFFER = 8

DEFAULT_SAVE_DIR = "debug_frames"

@dataclass
class FingerResult:

    found: bool
    x: int
    y: int
    confidence: float
    frame_id: int
    image: Optional[np.ndarray] = None
    saved_path: Optional[str] = None

    def __bool__(self) -> bool:
        return self.found

    @property
    def xy(self) -> Tuple[int, int]:
        return self.x, self.y


class FingerRig:
    def __init__(self, port: str = None, model: str = "fingertip_model.pt",
                 conf: float = DEFAULT_CONF_THRESHOLD, device: str = "cpu",
                 exposure=None, gain: int = 12, verbose: bool = True,
                 save_dir: str = DEFAULT_SAVE_DIR, on_demand: bool = False):
        self.verbose = verbose
        self.detector = None
        if model is not None:
            self.detector = FingertipDetector(model, conf_threshold=conf, device=device)

        self.ser = open_serial(port, timeout=0.02)
        self.reader = FrameReader(self.ser)

        self._frames = collections.deque(maxlen=FRAME_BUFFER)
        self._streaming = False
        self._homed = False
        self._state = None
        self._switches = 0
        self._arrived = False
        self._pos = None
        self._button_presses = 0
        self._last_button_pos = None
        self._button_callback = None
        self._in_button_callback = False
        self.displayed_text = ""
        self.busy = False
        self._target_gen = 0

        self.save_dir = save_dir
        self._save_counter = None
        self.on_demand = on_demand

        quiesce(self.ser, self.reader)
        if exposure is not None:
            self.set_exposure(exposure, gain)
        
    def move_to(self, x: int, y: int, wait: bool = True, timeout: float = 30.0) -> bool:
        if not self._homed:
            raise RuntimeError(
                pass

        tx = int(max(0, min(MAX_X, x)))
        ty = int(max(0, min(MAX_Y, y)))

        self._pump(0.0)
        self._arrived = False

        send_target(self.ser, tx, ty)
        self._target_gen += 1
        mojaGen = self._target_gen
        if not wait:
            return True

        if self._pos == (tx, ty):
            return True

        ok = self._pump(timeout, until=lambda: self._arrived or not self._homed
                        or self._target_gen != mojaGen)

        if self._target_gen != mojaGen:
            if self.verbose:
                print(f"move_to({tx}, {ty}): cel zastapiony w trakcie przejazdu "
                      f"(pozycja: {self._pos})")
            return False

        if not self._homed:
            raise RuntimeError(
                "ESP zglosil, ze nie jest zhomowany (stan: "
                f"{STATE_NAMES.get(self._state, self._state)}). "
                "Prawdopodobnie plytka sie zresetowala - powtorz home().")
        if not ok and self._pos == (tx, ty):
            ok = True
        if not ok and self.verbose:
            print(f"move_to({tx}, {ty}): brak potwierdzenia dojazdu w {timeout:.0f}s "
                  f"(ostatnia znana pozycja: {self._pos})")
        return ok

    def read_finger(self, latest: bool = True, timeout: float = 3.0,
                    with_image: bool = False, save: bool = False,
                    fresh: bool = True) -> FingerResult:
                      
        if self.detector is None:
            raise RuntimeError("FingerRig utworzony z model=None - detekcja niedostepna.")

        if self.on_demand:
            self._begin_shot()
        elif fresh:
            self.resync()
        elif not self._streaming:
            self.stream(True)

        try:
            return self._read_and_predict(latest, timeout, with_image, save)
        finally:
            if self.on_demand:
                self.stream(False)

    def _begin_shot(self):
        self._pump(0.0)
        self._frames.clear()
        self.reader.buf.clear()
        self.ser.reset_input_buffer()
        self.stream(True)

    def _read_and_predict(self, latest, timeout, with_image, save) -> FingerResult:
        deadline = time.monotonic() + timeout
        while True:
            self._pump(0.0)
            if not self._frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Brak klatki z ESP w ciagu {timeout:.1f}s. Sprawdz polaczenie "
                        f"USB i czy strumien jest wlaczony (uszkodzone klatki: "
                        f"{self.reader.corrupt_frames}).")
                if self.on_demand and not self._streaming:
                    self._begin_shot()
                self._pump(remaining, until=lambda: bool(self._frames))
                continue

            if latest:
                frame_id, jpeg = self._frames[-1]
                self._frames.clear()
            else:
                frame_id, jpeg = self._frames.popleft()

            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue

            found, px, py, confidence = self.detector.predict(img)
            path = None
            if save:
                path = self._save_annotated(img, found, px, py, confidence, frame_id)
            return FingerResult(found, px, py, confidence, frame_id,
                                img if with_image else None, path)

    def _save_annotated(self, img, found, px, py, confidence, frame_id) -> str:
        if self._save_counter is None:
            os.makedirs(self.save_dir, exist_ok=True)
            self._save_counter = len([f for f in os.listdir(self.save_dir)
                                      if f.endswith(".jpg")])

        vis = img.copy()
        if found:
            cv2.circle(vis, (px, py), 6, (0, 0, 255), -1)
            cv2.circle(vis, (px, py), 10, (255, 255, 255), 2)
            label = f"{confidence:.2f}  ({px}, {py})"
            color = (0, 0, 255)
        else:
            label = f"brak palca  (max {confidence:.2f})"
            color = (0, 200, 255)
        cv2.putText(vis, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        name = (f"{self._save_counter:05d}_id{frame_id}_"
                f"{'ok' if found else 'brak'}_conf{confidence:.2f}.jpg")
        path = os.path.join(self.save_dir, name)
        cv2.imwrite(path, vis)
        self._save_counter += 1
        return path

    def send_result(self, result: FingerResult):
        send_coords(self.ser, result.frame_id, result.found, result.x, result.y)


    def home(self, timeout: float = 180.0) -> bool:
        was_streaming = self._streaming
        if was_streaming:
            self.stream(False)
            self._pump(0.3)
            self._frames.clear()

        self._state = None
        self._arrived = False
        send_home(self.ser)

        ok = self._pump(timeout, until=lambda: self._state in (ST_HOMED, ST_FAILED))

        if not ok:
            print(f"TIMEOUT: brak odpowiedzi o stanie homingu w ciagu {timeout:.0f}s.")
        elif self._state == ST_FAILED:
            print("\n--- HOMING NIEUDANY ---")
            print("Maszyna nie wykonala ruchu szukania zera: krancowki od razu")
            print("raportuja stan koncowy, wiec petle homingu zakonczyly sie od razu.")
            print(f"Aktualny odczyt: {describe_switches(self._switches)}")
            print("Sprawdz podlaczenie krancowek, ich logike (NO/NC) oraz czy")
            print("toolhead nie stoi juz dosuniety do krancowki.")
            print("Maszyna NIE pojedzie do zadnego celu dopoki homing sie nie powiedzie.")

        success = self._homed and self._state == ST_HOMED
        if success:
            self._pump(0.5)
            if self._pos is None:
                self._pos = HOME_POSITION
            if self.verbose:
                print(f"Homing OK, pozycja: {self._pos}")

        if was_streaming:
            self.stream(True)
        return success

    def displayText(self, text: str):
        self.displayed_text = text
        send_text(self.ser, text)

    @property
    def position(self) -> Optional[Tuple[int, int]]:
        self._pump(0.0)
        return self._pos

    @property
    def is_homed(self) -> bool:
        self._pump(0.0)
        return self._homed

    @property
    def corrupt_frames(self) -> int:
        return self.reader.corrupt_frames

    def stream(self, on: bool):
        send_stream(self.ser, on)
        self._streaming = on
        if not on:
            self._frames.clear()

    @contextlib.contextmanager
    def take_control(self):
        self.busy = True
        try:
            yield self
        finally:
            self.busy = False

    def on_button(self, callback):
        self._button_callback = callback

    def button_pressed(self):

        self._pump(0.0)
        if self._button_presses > 0:
            self._button_presses -= 1
            return self._last_button_pos
        return None

    def clear_button_presses(self):
        self._pump(0.0)
        self._button_presses = 0

    def wait(self, seconds: float):
        self._pump(max(0.0, seconds))

    def resync(self, settle: float = 0.2):
        self._pump(0.0)
        quiesce(self.ser, self.reader, settle)
        self._streaming = False
        self._frames.clear()
        self.stream(True)

    def set_exposure(self, aec, gain: int = 12):
        """aec: 0..1200 (wiecej = jasniej i wiecej rozmycia) albo "auto"."""
        value = AUTO_EXPOSURE if str(aec).lower() == "auto" else int(aec)
        send_exposure(self.ser, value, gain)
        time.sleep(0.3)

    def close(self):
        try:
            send_stream(self.ser, False)
        except Exception:
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _dispatch(self, event):
        kind = event[0]
        if kind == "frame":
            self._frames.append((event[1], event[2]))
        elif kind == "arrived":
            self._pos = (event[1], event[2])
            self._arrived = True
        elif kind == "status":
            self._state, self._switches = event[1], event[2]
            self._homed = (event[1] == ST_HOMED)
        elif kind == "button":
            press = (event[1], event[2])
            self._last_button_pos = press
            if self._button_callback is not None and not self._in_button_callback:
                self._in_button_callback = True
                try:
                    self._button_callback(*press)
                finally:
                    self._in_button_callback = False
            else:
                self._button_presses += 1

    def _pump(self, timeout: float = 0.0, until=None) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            event = self.reader.poll_event()
            if event is not None:
                self._dispatch(event)
                if until is not None and until():
                    return True
                continue
            if until is not None and until():
                return True
            if self.busy and not self._in_button_callback:
                return False
            if time.monotonic() >= deadline:
                return until is None
            time.sleep(0.001)

def _demo():
    ap = argparse.ArgumentParser(description="Test dymny finger_api")
    ap.add_argument("--port", help="np. COM11; domyslnie autodetekcja")
    ap.add_argument("--model", default="fingertip_model.pt")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF_THRESHOLD)
    ap.add_argument("--exposure", default=None, help="0..1200 albo 'auto'")
    ap.add_argument("--gain", type=int, default=12)
    ap.add_argument("--reads", type=int, default=30, help="ile odczytow palca")
    ap.add_argument("--no-move", action="store_true", help="pomin test przejazdu")
    ap.add_argument("--save", action="store_true",
                    help="zapisuj klatki z zaznaczonym punktem")
    ap.add_argument("--save-dir", default=DEFAULT_SAVE_DIR, help="folder na te klatki")
    args = ap.parse_args()

    with FingerRig(port=args.port, model=args.model, conf=args.conf,
                   exposure=args.exposure, gain=args.gain,
                   save_dir=args.save_dir) as rig:

        print("\n[1/3] homing...")
        if not rig.home():
            raise SystemExit("Homing nieudany - przerywam.")

        if not args.no_move:
            print("\n[2/3] przejazd do srodka obszaru...")
            t0 = time.monotonic()
            ok = rig.move_to(MAX_X // 2, MAX_Y // 2)
            print(f"  dojazd: {ok}, czas {time.monotonic() - t0:.2f}s, "
                  f"pozycja {rig.position}")

        print(f"\n[3/3] {args.reads} odczytow palca...")
        hits = 0
        t0 = time.monotonic()
        for _ in range(args.reads):
            r = rig.read_finger(save=args.save)
            if r.found:
                hits += 1
                print(f"  x={r.x:4d} y={r.y:4d}  pewnosc={r.confidence:.2f}")
            else:
                print(f"  brak palca      (pewnosc={r.confidence:.2f})")
        dt = time.monotonic() - t0
        print(f"\nTrafienia: {hits}/{args.reads} | {args.reads / dt:.1f} odczytow/s "
              f"| uszkodzone klatki: {rig.corrupt_frames}")
        if args.save:
            print(f"Podglad zapisany w: {os.path.abspath(rig.save_dir)}")


if __name__ == "__main__":
    _demo()
