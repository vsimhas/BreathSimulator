#!/usr/bin/env python3
"""Sensor bring-up check for the breath simulator board.

Connects to the board's USB-CDC telemetry port, asks the SFM3300 flow
sensor and the AMS5935 pressure sensor for status, then streams live
readings so you can confirm both respond and react before anything is
wired into the control loop.

Typical session:

    python flow_check.py --list
    python flow_check.py --port COM7 --zero

Blow gently through the sensor and watch the bar move. Positive is flow in
the direction of the arrow on the sensor body.

Only needs pyserial.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required:  pip install pyserial")


BAR_WIDTH = 31          # odd, so there is a exact centre column for zero flow
DEFAULT_SPAN_SLM = 50.0  # full-scale of the ASCII bar


def find_ports() -> list[str]:
    return [p.device for p in list_ports.comports()]


def print_ports() -> None:
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        print(f"  {p.device:10s}  {p.description}")


def bar(value: float, span: float) -> str:
    """Centre-zero ASCII bar; '|' marks zero, '#' fills toward the reading."""
    half = BAR_WIDTH // 2
    cells = ["-"] * BAR_WIDTH
    cells[half] = "|"
    n = int(round((value / span) * half))
    n = max(-half, min(half, n))
    if n > 0:
        for i in range(half + 1, half + 1 + n):
            cells[i] = "#"
    elif n < 0:
        for i in range(half + n, half):
            cells[i] = "#"
    return "".join(cells)


class FlowMonitor:
    def __init__(self, port: str, baud: int, span: float) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.span = span
        self.samples = 0
        self.bad = 0
        self.lo = float("inf")
        self.hi = float("-inf")
        self.total = 0.0
        self.last_err_count: int | None = None
        self.dropped_by_device = 0
        self.press_cmh2o: float | None = None
        self.press_samples = 0
        self.press_lo = float("inf")
        self.press_hi = float("-inf")

    def send(self, cmd: str) -> None:
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def drain_comments(self, seconds: float = 1.0) -> None:
        """Print '#' lines (status/ack/err) for a short while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if text.startswith("#"):
                print("  " + text)

    def run(self, duration: float | None) -> int:
        end = None if duration is None else time.monotonic() + duration
        last_draw = 0.0

        while end is None or time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()

            if text.startswith("#"):
                print("\n  " + text)
                continue

            if text.startswith("V,"):
                # V, breath, tidal_ml, p_min, p_max, p_mean, q_peak, flags
                v = text.split(",")
                if len(v) == 8:
                    try:
                        print(f"\n  breath {int(v[1]):4d}: Vt {float(v[2]):6.1f} mL"
                              f"   P {float(v[3]):5.2f} / {float(v[4]):5.2f}"
                              f" (mean {float(v[5]):5.2f}) cmH2O"
                              f"   flags {v[7]}")
                    except ValueError:
                        pass
                continue

            if text.startswith("P,"):
                # P, tick, raw, mbar, cmH2O, temp, held, status, baro
                pparts = text.split(",")
                if len(pparts) == 9:
                    try:
                        self.press_cmh2o = float(pparts[4])
                        held = int(pparts[6])
                    except ValueError:
                        continue
                    if not held:
                        self.press_samples += 1
                        self.press_lo = min(self.press_lo, self.press_cmh2o)
                        self.press_hi = max(self.press_hi, self.press_cmh2o)
                continue

            if not text.startswith("F,"):
                continue

            parts = text.split(",")
            # F, tick_ms, raw, flow_slm, flow_filt_slm, sample_ok, error_count
            if len(parts) != 7:
                continue
            try:
                counts = int(parts[2])
                flow = float(parts[3])
                filt = float(parts[4])
                ok = int(parts[5])
                errs = int(parts[6])
            except ValueError:
                continue

            if self.last_err_count is not None and errs > self.last_err_count:
                self.dropped_by_device += errs - self.last_err_count
            self.last_err_count = errs

            if ok:
                self.samples += 1
                self.total += flow
                self.lo = min(self.lo, flow)
                self.hi = max(self.hi, flow)
            else:
                self.bad += 1

            now = time.monotonic()
            if now - last_draw >= 0.05:   # ~20 Hz refresh, easy on the console
                last_draw = now
                press = ("     ---" if self.press_cmh2o is None
                         else f"{self.press_cmh2o:+8.2f}")
                sys.stdout.write(
                    f"\r{bar(filt, self.span)}  {filt:+7.2f} slm  "
                    f"{press} cmH2O  n={self.samples:6d}  err={errs:4d}   "
                )
                sys.stdout.flush()

        return 0

    def summary(self) -> None:
        print("\n\n--- summary ---")
        print(f"  good samples : {self.samples}")
        print(f"  bad samples  : {self.bad}")
        print(f"  device errors: {self.dropped_by_device}")
        if self.press_samples:
            print(f"  press samples: {self.press_samples}")
            print(f"  press min    : {self.press_lo:+.2f} cmH2O")
            print(f"  press max    : {self.press_hi:+.2f} cmH2O")
        else:
            print("  press samples: 0  (no valid P lines - check 'press status')")
        if self.samples:
            print(f"  flow min     : {self.lo:+.2f} slm")
            print(f"  flow max     : {self.hi:+.2f} slm")
            print(f"  flow mean    : {self.total / self.samples:+.2f} slm")
            print()
            if abs(self.hi - self.lo) < 0.5:
                print("  Reading never moved. If you blew through the sensor, check")
                print("  that it is plumbed in the right place; if you did not, do")
                print("  that now and re-run.")
            else:
                print("  Sensor responds to flow.")
        else:
            print("\n  No valid samples. Check:")
            print("   - 'flow status' output above: present=1 and a non-zero serial")
            print("     mean the sensor answered; present=0 means it did not.")
            print("   - I2C1 wiring and pull-ups, sensor address 0x40.")
            print("   - 3.3 V supply to the sensor.")


def main() -> int:
    ap = argparse.ArgumentParser(description="Flow + pressure sensor bring-up check")
    ap.add_argument("--port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--zero", action="store_true",
                    help="tare the sensor before streaming (no flow, please)")
    ap.add_argument("--span", type=float, default=DEFAULT_SPAN_SLM,
                    help=f"bar full-scale in slm (default {DEFAULT_SPAN_SLM})")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop after this many seconds (default: until Ctrl-C)")
    args = ap.parse_args()

    if args.list:
        print_ports()
        return 0

    port = args.port
    if port is None:
        ports = find_ports()
        if len(ports) != 1:
            print_ports()
            print("\nSpecify --port (more than one port, or none, was found).")
            return 1
        port = ports[0]
        print(f"Using {port}")

    mon = FlowMonitor(port, args.baud, args.span)

    print("\nSensor status:")
    mon.send("flow status")
    mon.drain_comments(1.0)
    mon.send("press status")
    mon.drain_comments(1.0)

    if args.zero:
        print("\nTaring (make sure there is no flow and no pressure):")
        mon.send("flow zero")
        mon.send("press zero")
        mon.drain_comments(1.5)

    print("\nStreaming. Blow gently through the sensor. Ctrl-C to stop.\n")
    mon.send("flow on")
    mon.send("press on")

    try:
        mon.run(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            mon.send("flow off")
            mon.send("press off")
        except Exception:
            pass
        mon.summary()
        mon.ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
