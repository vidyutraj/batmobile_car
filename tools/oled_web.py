"""Local web page for cueing the OLED and piezo white or black, and testing the motors.

Talks to the oled_cue sketch over serial and serves a page at
http://127.0.0.1:8768: White / Black cue buttons, the message shown for each,
a mirror of the screen, the piezo tone for each, and drive buttons for the two
motors. Messages, tones and the cue are re-sent if the ESP32 resets; motor
commands never are. While a page is open the server sends a P keepalive every
0.5 s; the sketch stops the motors 1.5 s after the last one.

Usage: python oled_web.py [port]   (default: the first USB serial port found -
usbserial*, SLAB_USBtoUART*, wchusbserial*, usbmodem* - re-found whenever the
board reconnects). Plug into the DevKit's UART port: the sketch turns off the
native USB port because GPIO12 (USB D-) drives motor IN1.
Requires pyserial (pip install pyserial).
"""

import glob
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

HOST, PORT = "127.0.0.1", 8768
PAGE = Path(__file__).resolve().parent / "oled_web.html"
# The DevKit's UART USB port (CP210x / CH34x bridge); usbmodem kept for CH343 bridges and older sketches.
PORT_GLOBS = ("/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*", "/dev/cu.wchusbserial*", "/dev/cu.usbmodem*")
STATE_LINE = re.compile(r"^STATE cue=(white|black) oled=([01]) sda=(\d+) scl=(\d+) snd=([01]) "
                        r"hz=(\d+),(\d+) tone=(\d+)(?: ma=(-?\d+) mb=(-?\d+) brake=([01]))?")
MSG_LINE = re.compile(r"^MSG ([WK]) ?(.*)$")
MAX_MSG = 40


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.ser = None
        self.port = None
        self.listeners = []
        self.board = {"type": "state", "connected": False, "cue": None, "oled": False, "sda": 0, "scl": 0,
                      "sound": None, "tones": None, "tone": 0, "ma": 0, "mb": 0, "brake": False,
                      "white": None, "black": None}
        self.settings = {}    # command letter -> command, replayed after a reset

    def broadcast(self, msg):
        data = json.dumps(msg)
        with self.lock:
            for q in self.listeners:
                q.put(data)

    def update(self, **kw):
        self.board.update(kw)
        self.broadcast(self.board)

    def send(self, cmd, quiet=False):
        with self.lock:
            ser = self.ser
        if not ser:
            return False
        try:
            ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError):
            return False
        if not quiet:
            self.broadcast({"type": "log", "dir": "out", "text": cmd})
        return True

    def setting(self, key, cmd):
        self.settings[key] = cmd
        return self.send(cmd)

    def replay_settings(self):
        for cmd in list(self.settings.values()):
            self.send(cmd)


state = State()


def open_port(fixed):
    while True:
        candidates = [fixed] if fixed else sorted(
            p for pat in PORT_GLOBS for p in glob.glob(pat))
        for port in candidates:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = port, 115200, 0.2
            # Asserting DTR/RTS on the C6's native USB port holds the chip in reset.
            ser.dtr = ser.rts = False
            try:
                ser.open()
                state.port = port
                return ser
            except (serial.SerialException, OSError):
                pass
        time.sleep(0.5)


def handle_line(text):
    m = STATE_LINE.match(text)
    if m:
        state.update(cue=m[1], oled=m[2] == "1", sda=int(m[3]), scl=int(m[4]), sound=m[5] == "1",
                     tones=[int(m[6]), int(m[7])], tone=int(m[8]))
        if m[9] is not None:
            state.update(ma=int(m[9]), mb=int(m[10]), brake=m[11] == "1")
    m = MSG_LINE.match(text)
    if m:
        state.update(**{"white" if m[1] == "W" else "black": m[2]})
    if text.startswith("READY"):
        # The ESP32 (re)booted with the default messages.
        state.replay_settings()
    state.broadcast({"type": "log", "dir": "in", "text": text})


def serial_loop(fixed):
    while True:
        ser = open_port(fixed)
        with state.lock:
            state.ser = ser
        state.update(connected=True, port=state.port)
        state.replay_settings()
        state.send("?")
        buf = b""
        last_rx = time.time()
        reset_tried = False
        try:
            while True:
                chunk = ser.read(256)
                if chunk:
                    last_rx = time.time()
                elif not reset_tried and time.time() - last_rx > 2:
                    # Silent after opening the port: pulse RTS to reset the chip.
                    reset_tried = True
                    ser.rts = True
                    time.sleep(0.1)
                    ser.rts = False
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode(errors="replace").strip()
                    if text:
                        handle_line(text)
        except (serial.SerialException, OSError):
            with state.lock:
                state.ser = None
            state.update(connected=False, oled=False)
            ser.close()


def keepalive_loop():
    # Only while a page is open, so closing it lets the sketch's motor timeout stop the motors.
    while True:
        time.sleep(0.5)
        with state.lock:
            open_pages = len(state.listeners)
        if open_pages:
            state.send("P", quiet=True)


def clean(text):
    return " ".join(text.split())[:MAX_MSG]


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
        q = {k: v[0] for k, v in parse_qs(url.query, keep_blank_values=True).items()}
        if url.path == "/":
            body = PAGE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif url.path == "/cue":
            self.send_json({"ok": state.setting("C", f"C {1 if q.get('c') == 'black' else 0}")})
        elif url.path == "/message":
            key = "K" if q.get("for") == "black" else "W"
            self.send_json({"ok": state.setting(key, f"{key} {clean(q.get('text', ''))}")})
        elif url.path == "/sound":
            self.send_json({"ok": state.setting("U", f"U {1 if q.get('on') == '1' else 0}")})
        elif url.path == "/tones":
            try:
                w, b = (max(20, min(20000, int(q.get(k, "")))) for k in ("white", "black"))
            except ValueError:
                self.send_error(400)
                return
            self.send_json({"ok": state.setting("F", f"F {w} {b}")})
        elif url.path == "/motor":
            try:
                a, b = (max(-255, min(255, int(q.get(k, "")))) for k in ("a", "b"))
            except ValueError:
                self.send_error(400)
                return
            self.send_json({"ok": state.send(f"D {a} {b}")})
        elif url.path == "/brake":
            self.send_json({"ok": state.send("X")})
        elif url.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue()
            with state.lock:
                state.listeners.append(q)
            q.put(json.dumps(state.board))
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

    def do_POST(self):
        # navigator.sendBeacon on page close posts /motor?a=0&b=0.
        if urlparse(self.path).path == "/motor":
            self.do_GET()
        else:
            self.send_error(404)


def main():
    fixed = sys.argv[1] if len(sys.argv) > 1 else None
    threading.Thread(target=serial_loop, args=(fixed,), daemon=True).start()
    threading.Thread(target=keepalive_loop, daemon=True).start()
    print(f"OLED / piezo / motor page: http://{HOST}:{PORT}")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
