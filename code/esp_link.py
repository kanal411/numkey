import struct
import sys
import time

import serial
import serial.tools.list_ports

SYNC = b"\xA5\x5A"
TYPE_FRAME   = 0x01
TYPE_COORDS  = 0x02
TYPE_TARGET  = 0x03
TYPE_ARRIVED = 0x04
TYPE_HOME    = 0x05
TYPE_STREAM  = 0x06
TYPE_STATUS  = 0x07
TYPE_EXPOSURE = 0x08
TYPE_TEXT     = 0x09
TYPE_BUTTON   = 0x0A

AUTO_EXPOSURE = 0xFFFF

MAX_TEXT_LEN = 128

FRAME_HEADER_LEN = 11
ARRIVED_LEN = 8
STATUS_LEN = 6
BUTTON_LEN = 8
MAX_JPEG = 300_000

ST_NOT_HOMED = 0
ST_HOMING    = 1
ST_HOMED     = 2
ST_FAILED    = 3

STATE_NAMES = {
    ST_NOT_HOMED:,
    ST_HOMING:
    ST_HOMED:
    ST_FAILED:
}

ESPRESSIF_VID = 0x303A


def crc8(data: bytes) -> int:
    c = 0
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    for p in ports:
        if "USB" in (p.description or "").upper():
            return p.device
    names = ", ".join(p.device for p in ports) or "brak"
    sys.exit(f"Can't find ESP32-S3. Avaible ports: {names}. Use --port.")


def open_serial(port: str = None, timeout: float = 0.05) -> serial.Serial:
    port = port or find_port()
    ser = serial.Serial(port, 115200, timeout=timeout)
    ser.reset_input_buffer()
    print(f"Connected to {port}")
    return ser

def send_coords(ser: serial.Serial, frame_id: int, found: bool, x: int, y: int):
    ser.write(SYNC + struct.pack("<BIBHH", TYPE_COORDS, frame_id, 1 if found else 0,
                                 max(0, min(65535, x)), max(0, min(65535, y))))


def send_target(ser: serial.Serial, x: int, y: int):
    ser.write(SYNC + struct.pack("<BHH", TYPE_TARGET,
                                 max(0, min(65535, int(x))), max(0, min(65535, int(y)))))


def send_home(ser: serial.Serial):
    ser.write(SYNC + bytes([TYPE_HOME]))


def send_stream(ser: serial.Serial, on: bool):
    ser.write(SYNC + struct.pack("<BB", TYPE_STREAM, 1 if on else 0))


def send_exposure(ser: serial.Serial, aec: int, gain: int):
    if aec != AUTO_EXPOSURE:
        aec = max(0, min(1200, int(aec)))
    ser.write(SYNC + struct.pack("<BHB", TYPE_EXPOSURE, aec, max(0, min(30, int(gain)))))

_ASCII_FOLD = str.maketrans({
    "ą": "a", "ć": "c", "ę": "e", "ł": "l", "ń": "n",
    "ó": "o", "ś": "s", "ź": "z", "ż": "z",
    "Ą": "A", "Ć": "C", "Ę": "E", "Ł": "L", "Ń": "N",
    "Ó": "O", "Ś": "S", "Ź": "Z", "Ż": "Z",
})


def encode_text(text: str) -> bytes:
    folded = str(text).translate(_ASCII_FOLD)
    raw = folded.encode("ascii", errors="replace")
    return raw[:MAX_TEXT_LEN]


def send_text(ser: serial.Serial, text: str):
    payload = encode_text(text)
    ser.write(SYNC + struct.pack("<BB", TYPE_TEXT, len(payload)) + payload)

class FrameReader:

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = bytearray()
        self.corrupt_frames = 0

    def poll_event(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        return self._try_parse()

    def read_event(self, timeout: float = None):
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            if deadline is not None and time.monotonic() > deadline:
                return None
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n > 0 else 1)
            if chunk:
                self.buf += chunk
            event = self._try_parse()
            if event is not None:
                return event

    def _try_parse(self):
        while True:
            idx = self.buf.find(SYNC)
            if idx < 0:
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return None
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 3:
                return None

            msg_type = self.buf[2]

            if msg_type == TYPE_FRAME:
                if len(self.buf) < FRAME_HEADER_LEN:
                    return None
                frame_id, length = struct.unpack_from("<II", self.buf, 3)
                if length == 0 or length > MAX_JPEG:
                    del self.buf[:2]
                    continue
                if len(self.buf) < FRAME_HEADER_LEN + length:
                    return None
                jpeg = bytes(self.buf[FRAME_HEADER_LEN:FRAME_HEADER_LEN + length])
                del self.buf[:FRAME_HEADER_LEN + length]
                if not jpeg.startswith(b"\xFF\xD8"):
                    continue
                if b"\xFF\xD9" not in jpeg[-16:]:
                    self.corrupt_frames += 1
                    continue
                return ("frame", frame_id, jpeg)

            elif msg_type == TYPE_ARRIVED:
                if len(self.buf) < ARRIVED_LEN:
                    return None
                body = bytes(self.buf[2:7])
                if crc8(body) != self.buf[7]:
                    del self.buf[:2]
                    continue
                x, y = struct.unpack_from("<HH", self.buf, 3)
                del self.buf[:ARRIVED_LEN]
                return ("arrived", x, y)

            elif msg_type == TYPE_BUTTON:
                if len(self.buf) < BUTTON_LEN:
                    return None
                body = bytes(self.buf[2:7])
                if crc8(body) != self.buf[7]:
                    del self.buf[:2]
                    continue
                x, y = struct.unpack_from("<HH", self.buf, 3)
                del self.buf[:BUTTON_LEN]
                return ("button", x, y)

            elif msg_type == TYPE_STATUS:
                if len(self.buf) < STATUS_LEN:
                    return None
                body = bytes(self.buf[2:5])
                if crc8(body) != self.buf[5]:
                    del self.buf[:2]
                    continue
                state, switches = self.buf[3], self.buf[4]
                del self.buf[:STATUS_LEN]
                return ("status", state, switches)

            else:
                del self.buf[:2]
                continue

    def read_frame(self):
        while True:
            event = self.read_event()
            if event[0] == "frame":
                return event[1], event[2]

def quiesce(ser: serial.Serial, reader: "FrameReader", settle: float = 0.6):
    send_stream(ser, False)
    time.sleep(settle)
    ser.reset_input_buffer()
    reader.buf.clear()
    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        if ser.in_waiting:
            ser.read(ser.in_waiting)
            deadline = time.monotonic() + settle
        else:
            time.sleep(0.05)
    reader.buf.clear()


def describe_switches(switches: int) -> str:
    return f"buttonX={(switches >> 0) & 1} buttonY={(switches >> 1) & 1}"


def home_and_wait(ser: serial.Serial, reader: "FrameReader", timeout: float = 180.0):
    send_home(ser)
    deadline = time.monotonic() + timeout
    last_state = None

    while time.monotonic() < deadline:
        event = reader.read_event(timeout=1.0)
        if event is None:
            continue
        if event[0] != "status":
            continue

        state, switches = event[1], event[2]
        if state != last_state:
            last_state = state

        if state == ST_HOMED:
            return True
        if state == ST_FAILED:
            print("\nHOMING FAILED")
            return False

    print("TIMEOUT")
    return False
