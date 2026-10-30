"""Local web interface for the sensor-driven motor.

Talks to the line_motor sketch over serial and serves a control page at
http://127.0.0.1:8767: live sensor readings, per-sensor thresholds, the line
state (lost / centre / drift / turn), the motor speed used for each state, and
a mirror of what the OLED shows, and the speaker tone. In manual input each
sensor's black/white is set from the page instead of read from the board.

Thresholds, enables and state speeds set here are re-sent if the ESP32 resets,
but auto mode never is: it always comes back off.

Learning: while a label (white or black) is set from the page, every reading
is saved with it to recordings/<time>_learn.csv, and the page fits each
sensor's threshold from the labelled readings.

Usage: python line_web.py <port>
Requires pyserial (pip install pyserial).
"""

import csv
import json
import queue
import re
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import serial

HOST, PORT = "127.0.0.1", 8767
ROOT = Path(__file__).resolve().parent.parent
RECORDINGS = ROOT / "recordings"
PAGE = Path(__file__).resolve().parent / "line_web.html"
LABELS = ("white", "black")
SENSORS = ["VOUT1 (GPIO2)", "VOUT2 (GPIO3)", "VOUT3 (GPIO4)", "VOUT4 (GPIO5)"]
STATES = ["lost", "centre", "drift", "turn"]
DATA_LINE = re.compile(r"^D (\d+) (\d+) (\d+) (\d+) black=([01]{4}) state=(\w+) speed=(-?\d+) duty=(-?\d+) "
                       r"mode=(\w+) auto=([01]) oled=([01]) tone=(\d+)")
CFG_LINE = re.compile(r"^CFG auto=([01]) sp=(\d+),(\d+),(\d+),(\d+) th=(\d+),(\d+),(\d+),(\d+) en=([01]{4}) snd=([01]) hz=(\d+),(\d+) man=([01]) frc=([01]{4})")


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.ser = None
        self.connected = False
        self.listeners = []
        self.config = None    # last CFG reported by the ESP32
        self.settings = {}    # command -> value set from the page, replayed after a reset
        self.label = None     # "white" / "black" while learning, else None
        self.file = None
        self.writer = None
        self.rec_path = None
        self.rec_start = 0.0
        self.rec_rows = 0

    def learn_status(self):
        return {"type": "learn", "label": self.label, "rows": self.rec_rows,
                "file": self.rec_path.name if self.rec_path else None}

    def set_label(self, label):
        with self.lock:
            if label and not self.file:
                RECORDINGS.mkdir(exist_ok=True)
                self.rec_path = RECORDINGS / f"{datetime.now():%Y%m%d_%H%M%S}_learn.csv"
                self.file = open(self.rec_path, "w", newline="")
                self.writer = csv.writer(self.file)
                self.writer.writerow(["t_s", "label", *(f"VOUT{i + 1}" for i in range(4))])
                self.rec_start = time.time()
                self.rec_rows = 0
            self.label = label
            if self.file:
                self.file.flush()
        self.broadcast(self.learn_status())

    def reset_learning(self):
        with self.lock:
            if self.file:
                self.file.close()
            self.label = self.file = self.writer = self.rec_path = None
            self.rec_rows = 0
        self.broadcast(self.learn_status())

    def record(self, mv):
        with self.lock:
            if not (self.label and self.writer):
                return None
            self.writer.writerow([f"{time.time() - self.rec_start:.3f}", self.label, *mv])
            self.file.flush()
            self.rec_rows += 1
            return self.label

    def broadcast(self, msg):
        data = json.dumps(msg)
        with self.lock:
            for q in self.listeners:
                q.put(data)

    def status(self):
        return {"type": "status", "connected": self.connected}

    def send(self, cmd):
        with self.lock:
            ser = self.ser
        if not ser:
            return False
        try:
            ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError):
            return False
        self.broadcast({"type": "log", "dir": "out", "text": cmd})
        return True

    def setting(self, key, cmd):
        self.settings[key] = cmd
        return self.send(cmd)

    def replay_settings(self):
        for cmd in list(self.settings.values()):
            self.send(cmd)


state = State()


def open_port(port):
    while True:
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = port, 115200, 0.2
        # Asserting DTR/RTS on the C6's native USB port holds the chip in reset.
        ser.dtr = ser.rts = False
        try:
            ser.open()
            return ser
        except (serial.SerialException, OSError):
            time.sleep(0.5)


def handle_line(text):
    m = DATA_LINE.match(text)
    if m:
        g = m.groups()
        mv = [int(v) for v in g[:4]]
        state.broadcast({"type": "data", "mv": mv, "black": [b == "1" for b in g[4]], "state": g[5],
                         "speed": int(g[6]), "duty": int(g[7]), "mode": g[8], "auto": g[9] == "1",
                         "oled": g[10] == "1", "tone": int(g[11]),
                         "label": state.record(mv)})
        return
    m = CFG_LINE.match(text)
    if m:
        g = m.groups()
        state.config = {"type": "config", "auto": g[0] == "1", "speeds": [int(v) for v in g[1:5]],
                        "thresholds": [int(v) for v in g[5:9]], "enabled": [b == "1" for b in g[9]],
                        "sound": g[10] == "1", "tones": [int(g[11]), int(g[12])],
                        "manual": g[13] == "1", "forced": [b == "1" for b in g[14]]}
        state.broadcast(state.config)
    elif text.startswith("READY"):
        # The ESP32 (re)booted with default settings.
        state.replay_settings()
    state.broadcast({"type": "log", "dir": "in", "text": text})


def serial_loop(port):
    while True:
        ser = open_port(port)
        with state.lock:
            state.ser = ser
        state.connected = True
        state.broadcast(state.status())
        state.replay_settings()
        state.send("?")
        buf = b""
        try:
            while True:
                buf += ser.read(256)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode(errors="replace").strip()
                    if text:
                        handle_line(text)
        except (serial.SerialException, OSError):
            with state.lock:
                state.ser = None
            state.connected = False
            state.broadcast(state.status())
            ser.close()


def int_arg(q, key, lo, hi):
    return max(lo, min(hi, int(q.get(key, ""))))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send_json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        try:
            self.route(url.path, q)
        except ValueError:
            self.send_error(400)

    def route(self, path, q):
        if path == "/":
            body = PAGE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/config":
            self.send_json({"sensors": SENSORS, "states": STATES, "connected": state.connected,
                            "config": state.config})
        elif path == "/auto":
            self.send_json({"ok": state.send(f"A {int_arg(q, 'on', 0, 1)}")})
        elif path == "/statespeed":
            s = int_arg(q, "s", 0, len(STATES) - 1)
            self.send_json({"ok": state.setting(f"V{s}", f"V {s} {int_arg(q, 'v', 0, 255)}")})
        elif path == "/threshold":
            ch = int_arg(q, "ch", 1, 4)
            self.send_json({"ok": state.setting(f"T{ch}", f"T {ch} {int_arg(q, 'mv', 0, 3300)}")})
        elif path == "/enable":
            ch = int_arg(q, "ch", 1, 4)
            self.send_json({"ok": state.setting(f"E{ch}", f"E {ch} {int_arg(q, 'on', 0, 1)}")})
        elif path == "/speed":
            self.send_json({"ok": state.send(f"M {int_arg(q, 'v', -255, 255)}")})
        elif path == "/stop":
            self.send_json({"ok": state.send("M 0")})   # M also turns auto off
        elif path == "/tones":
            w, b = int_arg(q, "white", 20, 20000), int_arg(q, "black", 20, 20000)
            self.send_json({"ok": state.setting("F", f"F {w} {b}")})
        elif path == "/sound":
            self.send_json({"ok": state.setting("U", f"U {int_arg(q, 'on', 0, 1)}")})
        elif path == "/manual":
            self.send_json({"ok": state.setting("S", f"S {int_arg(q, 'on', 0, 1)}")})
        elif path == "/force":
            ch = int_arg(q, "ch", 1, 4)
            self.send_json({"ok": state.setting(f"K{ch}", f"K {ch} {int_arg(q, 'black', 0, 1)}")})
        elif path == "/brake":
            self.send_json({"ok": state.send("B")})
        elif path == "/label":
            label = q.get("l")
            state.set_label(label if label in LABELS else None)
            self.send_json(state.learn_status())
        elif path == "/learnreset":
            state.reset_learning()
            self.send_json(state.learn_status())
        elif path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with state.lock:
                state.listeners.append(q)
            q.put(json.dumps(state.status()))
            q.put(json.dumps(state.learn_status()))
            if state.config:
                q.put(json.dumps(state.config))
            try:
                while True:
                    self.wfile.write(f"data: {q.get()}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with state.lock:
                    state.listeners.remove(q)
        else:
            self.send_error(404)


def main():
    threading.Thread(target=serial_loop, args=(sys.argv[1],), daemon=True).start()
    print(f"Line motor web interface: http://{HOST}:{PORT}")
    try:
        ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        state.send("M 0")


if __name__ == "__main__":
    main()
