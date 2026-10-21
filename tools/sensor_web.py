"""Local web interface for the IR sensor array.

Reads the sensor_stream sketch's serial output, serves a live page at
http://127.0.0.1:8765, and saves each recording as a CSV in recordings/.

Usage: python sensor_web.py <port>
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

HOST, PORT = "127.0.0.1", 8765
ROOT = Path(__file__).resolve().parent.parent
RECORDINGS = ROOT / "recordings"
PAGE = Path(__file__).resolve().parent / "sensor_web.html"

# GPIO -> display name, in VOUT order. Mapping from the 2026-10-16 tape test.
CHANNELS = {"G3": "VOUT1 (GPIO3)", "G4": "VOUT2 (GPIO4)", "G5": "VOUT3 (GPIO5)", "G0": "VOUT4 (GPIO0)"}
FIELD = re.compile(r"(G\d+)=(\d+)")


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.connected = False
        self.listeners = []
        self.writer = None
        self.file = None
        self.rec_path = None
        self.rec_label = ""
        self.rec_start = 0.0
        self.rec_rows = 0

    def broadcast(self, msg):
        data = json.dumps(msg)
        with self.lock:
            for q in self.listeners:
                q.put(data)

    def start_recording(self, label):
        with self.lock:
            self._stop_locked()
            RECORDINGS.mkdir(exist_ok=True)
            safe = re.sub(r"[^A-Za-z0-9_-]+", "_", label).strip("_") or "recording"
            self.rec_path = RECORDINGS / f"{datetime.now():%Y%m%d_%H%M%S}_{safe}.csv"
            self.file = open(self.rec_path, "w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow(["t_s", "label", *CHANNELS])
            self.rec_label = label
            self.rec_start = time.time()
            self.rec_rows = 0
            return self.rec_path.name

    def stop_recording(self):
        with self.lock:
            return self._stop_locked()

    def _stop_locked(self):
        if not self.file:
            return None
        self.file.close()
        name, rows = self.rec_path.name, self.rec_rows
        self.file = self.writer = None
        return {"file": name, "rows": rows}

    def set_label(self, label):
        with self.lock:
            self.rec_label = label

    def record(self, values):
        with self.lock:
            if not self.writer:
                return None
            t = time.time() - self.rec_start
            self.writer.writerow([f"{t:.3f}", self.rec_label, *(values.get(k, "") for k in CHANNELS)])
            self.rec_rows += 1
            return {"t": round(t, 2), "rows": self.rec_rows, "label": self.rec_label}


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
    ser = open_port(port)
    state.connected = True
    buf = b""
    while True:
        try:
            buf += ser.read(256)
        except (serial.SerialException, OSError):
            state.connected = False
            state.broadcast({"type": "status", "connected": False})
            ser.close()
            ser = open_port(port)
            state.connected = True
            buf = b""
            continue
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            values = {k: int(v) for k, v in FIELD.findall(raw.decode(errors="replace"))}
            if not all(k in values for k in CHANNELS):
                continue
            rec = state.record(values)
            state.broadcast({"type": "reading", "values": values, "rec": rec})


def list_recordings():
    if not RECORDINGS.exists():
        return []
    return [p.name for p in sorted(RECORDINGS.glob("*.csv"), reverse=True)]


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
            self.send_json({"channels": CHANNELS, "connected": state.connected,
                            "recordings": list_recordings()})
        elif url.path == "/start":
            self.send_json({"file": state.start_recording(q.get("label", ""))})
        elif url.path == "/label":
            state.set_label(q.get("label", ""))
            self.send_json({"ok": True})
        elif url.path == "/stop":
            self.send_json({"stopped": state.stop_recording(), "recordings": list_recordings()})
        elif url.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with state.lock:
                state.listeners.append(q)
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
    print(f"Sensor web interface: http://{HOST}:{PORT}")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
