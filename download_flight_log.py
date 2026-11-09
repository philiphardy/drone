"""Download the flight log from the Pico over USB serial into "flight logs/flight_log_X.csv".

Usage: python download_flight_log.py [COM_PORT]
Requires: pip install pyserial
"""
import re
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

PICO_VID = 0x2E8A
BEGIN_RE = re.compile(r"--- FLIGHT LOG BEGIN \((\d+) samples, (\d+) i2c failures, (\d+) i2c recoveries\)")
END_MARK = "--- FLIGHT LOG END"
N_FIELDS = 15
LOG_DIR = Path(__file__).parent / "flight logs"
START_TIMEOUT_S = 3.0    # the firmware dumps by itself shortly after the port opens
IDLE_TIMEOUT_S = 2.0     # no data for this long => ask for the log again
MAX_REQUESTS = 4
TOTAL_TIMEOUT_S = 120.0


def find_port():
    ports = [p.device for p in list_ports.comports() if p.vid == PICO_VID]
    if len(ports) != 1:
        sys.exit(f"Expected one Pico serial port, found {len(ports)}; pass the port name as an argument.")
    return ports[0]


def bad_rows(rows):
    bad = 0
    for row in rows:
        fields = row.split(",")
        try:
            if len(fields) != N_FIELDS:
                raise ValueError
            [float(f) for f in fields]
        except ValueError:
            bad += 1
    return bad


def read_log(ser):
    """Return (lines, ok, (i2c_failures, i2c_recoveries)). Retries until a dump matches its advertised sample count."""
    buf = b""
    lines = None          # header + rows of the dump in progress
    expected = 0
    stats = (0, 0)
    best, best_good, best_stats = None, -1, stats
    requests = 0
    start = time.monotonic()
    request_at = start + START_TIMEOUT_S

    while time.monotonic() - start < TOTAL_TIMEOUT_S:
        if time.monotonic() >= request_at:
            if requests == MAX_REQUESTS:
                break
            requests += 1
            print(f"No complete log yet, requesting dump ({requests}/{MAX_REQUESTS})...")
            lines = None
            try:
                ser.write(b"\n")   # any keypress makes the firmware dump again
            except serial.SerialTimeoutException:
                print("  write timed out (firmware not reading USB?)")
            request_at = time.monotonic() + IDLE_TIMEOUT_S

        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        request_at = time.monotonic() + IDLE_TIMEOUT_S
        buf += chunk
        *complete, buf = buf.split(b"\n")

        for raw in complete:
            line = raw.decode("utf-8", errors="replace").strip()
            m = BEGIN_RE.match(line)
            if m:
                lines, expected = [], int(m.group(1))
                stats = (int(m.group(2)), int(m.group(3)))
            elif line.startswith(END_MARK) and lines is not None:
                rows = lines[1:]
                bad = bad_rows(rows)
                if len(rows) == expected and bad == 0:
                    return lines, True, stats
                print(f"  incomplete dump: {len(rows)}/{expected} rows, {bad} malformed")
                if len(rows) - bad > best_good:
                    best, best_good, best_stats = lines, len(rows) - bad, stats
                lines = None
                request_at = time.monotonic()
            elif lines is not None and line:
                lines.append(line)

    if best is None:
        sys.exit("Timed out waiting for the flight log. Is the drone in failsafe (motors stopped) "
                 "and is no other program holding the port?")
    return best, False, best_stats


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    LOG_DIR.mkdir(exist_ok=True)
    out = LOG_DIR / f"flight_log_{len(list(LOG_DIR.glob('flight_log_*.csv'))) + 1}.csv"

    print(f"Reading from {port}...")
    try:
        with serial.Serial(port, 115200, timeout=0.2, write_timeout=1.0) as ser:
            ser.dtr = True
            lines, ok, (i2c_failures, i2c_recoveries) = read_log(ser)
    except serial.SerialException as e:
        sys.exit(f"Serial error: {e}")

    out.write_text("\n".join(lines) + "\n")
    print(f"Saved {len(lines) - 1} samples to {out}")
    print(f"IMU I2C failures: {i2c_failures}, bus recoveries: {i2c_recoveries}")
    if not ok:
        print("WARNING: no clean dump received; saved the best one (has missing/garbled rows).")


if __name__ == "__main__":
    main()
