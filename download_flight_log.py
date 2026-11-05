"""Download the flight log from the Pico over USB serial into "flight logs/flight_log_X.csv".

Usage: python download_flight_log.py [COM_PORT]
Requires: pip install pyserial
"""
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

PICO_VID = 0x2E8A
BEGIN_MARK = "--- FLIGHT LOG BEGIN"
END_MARK = "--- FLIGHT LOG END"
LOG_DIR = Path(__file__).parent / "flight logs"
START_TIMEOUT_S = 3.0    # the firmware dumps by itself shortly after the port opens
FINISH_TIMEOUT_S = 30.0


def find_port():
    ports = [p.device for p in list_ports.comports() if p.vid == PICO_VID]
    if len(ports) != 1:
        sys.exit(f"Expected one Pico serial port, found {len(ports)}; pass the port name as an argument.")
    return ports[0]


def read_log(port):
    lines = []
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        ser.dtr = True
        recording = False
        nudged = False
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            if not recording and not nudged and elapsed > START_TIMEOUT_S:
                ser.write(b"\n")   # any keypress makes the firmware dump again
                nudged = True
            if elapsed > START_TIMEOUT_S + FINISH_TIMEOUT_S:
                sys.exit("Timed out waiting for the flight log.")

            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line.startswith(BEGIN_MARK):
                recording = True
                lines = []
            elif line.startswith(END_MARK) and recording:
                return lines
            elif recording and line:
                lines.append(line)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    LOG_DIR.mkdir(exist_ok=True)
    out = LOG_DIR / f"flight_log_{len(list(LOG_DIR.glob('flight_log_*.csv'))) + 1}.csv"

    print(f"Reading from {port}...")
    lines = read_log(port)
    out.write_text("\n".join(lines) + "\n")
    print(f"Saved {len(lines) - 1} samples to {out}")


if __name__ == "__main__":
    main()
