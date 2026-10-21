"""Live terminal view of the IR sensor sketch's serial output.

Shows each reading with a bar graph scaled to the lowest and highest
readings seen so far, and logs every line with a timestamp.

Keys: w = calibrate WHITE, b = calibrate BLACK, r = reset calibration,
      c = clear the bar's min/max, q = quit.

Usage: python live_view.py <port> [log_file]
Requires pyserial (pip install pyserial).
"""

import re
import select
import sys
import termios
import time
import tty

import serial

BAR_WIDTH = 40
READING = re.compile(r"(-?\d+(?:\.\d+)?) mV")


def open_port(port):
    """Opens the port, waiting for it to reappear if the board is resetting."""
    while True:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 0.05
        # Asserting DTR/RTS on the C6's native USB port holds the chip in reset.
        ser.dtr = False
        ser.rts = False
        try:
            ser.open()
            return ser
        except (serial.SerialException, OSError):
            time.sleep(0.5)


def main():
    port = sys.argv[1]
    log_path = sys.argv[2] if len(sys.argv) > 2 else None

    ser = open_port(port)

    log = open(log_path, "a", buffering=1) if log_path else None
    lo = hi = None
    t0 = time.time()
    buf = b""

    old_term = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("Keys: w=calibrate WHITE  b=calibrate BLACK  r=reset cal  c=clear bar range  q=quit\n")
    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "q":
                    break
                if key == "c":
                    lo = hi = None
                    print("-- bar range cleared --")
                elif key in "wbr":
                    ser.write(key.encode())
                    print({"w": "-- calibrating WHITE, hold still --",
                           "b": "-- calibrating BLACK, hold still --",
                           "r": "-- calibration reset --"}[key])

            try:
                buf += ser.read(256)
            except (serial.SerialException, OSError):
                # The board reset (rewiring, a brief short) and its USB port
                # dropped out; reconnect instead of exiting.
                print("-- board disconnected, reconnecting --")
                ser.close()
                ser = open_port(port)
                buf = b""
                print("-- reconnected --")
                continue
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                t = time.time() - t0
                if log:
                    log.write(f"{t:7.2f} {line}\n")

                m = READING.search(line)
                if not m:
                    print(line)
                    continue
                mv = float(m.group(1))
                lo = mv if lo is None else min(lo, mv)
                hi = mv if hi is None else max(hi, mv)
                span = hi - lo
                filled = int(round((mv - lo) / span * BAR_WIDTH)) if span > 0 else 0
                bar = "█" * filled + "·" * (BAR_WIDTH - filled)
                print(f"{t:6.1f}s  {line}  [{bar}]  range {lo:.1f}-{hi:.1f}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        ser.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
