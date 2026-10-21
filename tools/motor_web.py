"""Local web interface for driving the motor.

Sends commands to the motor_control sketch over serial and serves a control
page at http://127.0.0.1:8766. Preset sequences run here, step by step; any
manual command cancels a running sequence.

Usage: python motor_web.py <port>
Requires pyserial (pip install pyserial).
"""

import json
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import serial

HOST, PORT = "127.0.0.1", 8766
PAGE = Path(__file__).resolve().parent / "motor_web.html"
STATE_LINE = re.compile(r"STATE speed=(-?\d+) duty=(-?\d+) mode=(\w+)")

# Each step is (label, command, seconds). Commands are motor_control lines.
SEQUENCES = {
    "test": ("Bench test (fwd 50 / rev 50 / fwd 100)", [
        ("Forward 50%", "M 128", 2.0), ("Stop", "M 0", 1.5),
        ("Reverse 50%", "M -128", 2.0), ("Stop", "M 0", 1.5),
        ("Forward 100%", "M 255", 2.0), ("Stop", "M 0", 0.0),
    ]),
    "ramp": ("Ramp up and down (forward)", [
        *((f"Forward {round(d / 2.55)}%", f"M {d}", 0.5) for d in range(0, 256, 32)),
        ("Forward 100%", "M 255", 1.0),
        *((f"Forward {round(d / 2.55)}%", f"M {d}", 0.5) for d in range(224, -1, -32)),
        ("Stop", "M 0", 0.0),
    ]),
    # Raw duty (R), bypassing the firmware's MIN_DUTY scaling, to re-measure it.
    "deadband": ("Find minimum start duty (raw)", [
        *((f"Raw duty {d} ({round(d / 2.55)}%)", f"R {d}", 1.0) for d in range(120, 256, 8)),
        ("Stop", "M 0", 0.0),
    ]),
    "brake_vs_coast": ("Brake vs coast from full speed", [
        ("Forward 100%", "M 255", 2.0), ("Coast", "M 0", 2.0),
        ("Forward 100%", "M 255", 2.0), ("Brake", "B", 1.0),
        ("Stop", "M 0", 0.0),
    ]),
}


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.ser = None
        self.connected = False
        self.listeners = []
        self.speed = 0
        self.duty = 0
        self.mode = "coast"
        self.seq_cancel = threading.Event()
        self.seq_thread = None

    def broadcast(self, msg):
        data = json.dumps(msg)
        with self.lock:
            for q in self.listeners:
                q.put(data)

    def snapshot(self):
        return {"type": "state", "connected": self.connected, "speed": self.speed, "duty": self.duty,
                "mode": self.mode}

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

    def manual(self, cmd):
        self.cancel_sequence()
        return self.send(cmd)

    def cancel_sequence(self):
        t = self.seq_thread
        if t and t.is_alive():
            self.seq_cancel.set()
            t.join()

    def run_sequence(self, name):
        self.cancel_sequence()
        title, steps = SEQUENCES[name]
        self.seq_cancel = threading.Event()
        self.seq_thread = threading.Thread(target=self._sequence, args=(name, title, steps, self.seq_cancel),
                                           daemon=True)
        self.seq_thread.start()

    def _sequence(self, name, title, steps, cancel):
        for i, (label, cmd, secs) in enumerate(steps):
            self.broadcast({"type": "seq", "name": name, "title": title, "step": i + 1,
                            "steps": len(steps), "label": label})
            if not self.send(cmd) or cancel.wait(secs):
                self.send("M 0")
                self.broadcast({"type": "seq", "name": None, "ended": "cancelled"})
                return
        self.broadcast({"type": "seq", "name": None, "ended": "finished"})


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


def serial_loop(port):
    while True:
        ser = open_port(port)
        with state.lock:
            state.ser = ser
        state.connected = True
        state.send("?")
        state.broadcast(state.snapshot())
        buf = b""
        try:
            while True:
                buf += ser.read(256)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode(errors="replace").strip()
                    if not text:
                        continue
                    m = STATE_LINE.search(text)
                    if m:
                        state.speed, state.duty, state.mode = int(m.group(1)), int(m.group(2)), m.group(3)
                        state.broadcast(state.snapshot())
                    state.broadcast({"type": "log", "dir": "in", "text": text})
        except (serial.SerialException, OSError):
            with state.lock:
                state.ser = None
            state.connected = False
            state.broadcast(state.snapshot())
            ser.close()


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
        if url.path == "/":
            body = PAGE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif url.path == "/config":
            self.send_json({**state.snapshot(),
                            "sequences": {k: {"title": t, "steps": len(s)} for k, (t, s) in SEQUENCES.items()}})
        elif url.path == "/speed":
            try:
                speed = max(-255, min(255, int(q.get("v", "0"))))
            except ValueError:
                return self.send_error(400)
            self.send_json({"ok": state.manual(f"M {speed}")})
        elif url.path == "/brake":
            self.send_json({"ok": state.manual("B")})
        elif url.path == "/stop":
            self.send_json({"ok": state.manual("M 0")})
        elif url.path == "/run":
            if q.get("seq") not in SEQUENCES:
                return self.send_error(404)
            state.run_sequence(q["seq"])
            self.send_json({"ok": True})
        elif url.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with state.lock:
                state.listeners.append(q)
            q.put(json.dumps(state.snapshot()))
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
    print(f"Motor web interface: http://{HOST}:{PORT}")
    try:
        ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        state.manual("M 0")


if __name__ == "__main__":
    main()
