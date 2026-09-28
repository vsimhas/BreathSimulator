#!/usr/bin/env python3
"""Measure the blower's pressure-flow (P-Q) curve.

Holds the blower at ONE constant speed and logs (airway pressure, flow) while
you step the CPAP pressure by hand. Constant speed is the point: a P-Q curve is
only meaningful at a fixed impeller speed, so this uses rpm_base with amplitude
zero rather than any breath waveform.

    python pq_curve.py --port COM15 --rpm 20000

Set the CPAP to 4, then 8, 12, 16, 20 cmH2O, pausing a few seconds at each.
Ctrl-C when done. The script bins the samples by pressure, prints the curve,
and fits the shutoff head - the pressure at which the blower can no longer
move any gas.

Because shutoff head scales with the square of impeller speed (fan affinity
law), a curve measured at one speed predicts the others. Run it at two speeds
to confirm the scaling holds before trusting the extrapolation.

!! READ THIS BEFORE USING IT !!

This tool does NOT work on the normal rig configuration, and the reason is
physics rather than a bug. With the test lung connected the circuit is CLOSED:
at a constant blower speed the lung reaches equilibrium and net flow goes to
zero, at every pressure. Measured 2026-09-28 - 0.00 slm at both 4 and 10
cmH2O, 2500+ samples each - which looked like a dead blower and was not.

Disconnecting the lung gives a flow path, but then the CPAP leaks and can no
longer hold the pressure you set, so the x-axis is meaningless.

A steady-state P-Q curve therefore cannot be measured on this rig at all
without a calibrated orifice to flow through. Use tidal_point.py instead: in a
closed circuit the meaningful quantity is delivered TIDAL VOLUME per breath,
not steady flow.

Kept only for use with a deliberate calibrated leak fitted.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required:  pip install pyserial")

ABORT_FLAGS = 0x0008 | 0x0400          # MOTOR_FAULT, FLOW_NO_RESPONSE


class Rig:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.flow = None
        self.press = None
        self.flags = 0
        self.state = "?"

    def send(self, cmd: str) -> None:
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def read_once(self, echo: bool = False) -> None:
        raw = self.ser.readline()
        if not raw:
            return
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            return
        if line.startswith("#") or line.startswith("["):
            if echo:
                print("    " + line)
            return
        p = line.split(",")
        try:
            if line.startswith("F,") and len(p) == 7:
                self.flow = float(p[4])
            elif line.startswith("P,") and len(p) == 9:
                self.press = float(p[4])
            elif line.startswith("B,") and len(p) >= 9:
                self.flags = int(p[5], 16)
            elif line.startswith("S,") and len(p) >= 3:
                self.state = p[2]
        except (ValueError, IndexError):
            pass

    def pump(self, seconds: float, echo: bool = False) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.read_once(echo)

    def stop(self) -> None:
        try:
            self.send("sim stop")
            self.send("flow off")
            self.send("press off")
            self.pump(0.5)
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


def fit_shutoff(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    """Least-squares line through (P, Q); return (Q at P=0, P at Q=0)."""
    n = len(points)
    if n < 3:
        return None
    sx = sum(p for p, _ in points)
    sy = sum(q for _, q in points)
    sxx = sum(p * p for p, _ in points)
    sxy = sum(p * q for p, q in points)
    den = (n * sxx) - (sx * sx)
    if abs(den) < 1e-9:
        return None
    slope = ((n * sxy) - (sx * sy)) / den
    intercept = (sy - (slope * sx)) / n
    if slope >= 0:
        return None                      # not a blower curve
    return intercept, -intercept / slope


def main() -> int:
    ap = argparse.ArgumentParser(description="Blower pressure-flow curve")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rpm", type=int, default=20000,
                    help="constant blower speed (2000..20000, via rpm_base)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="stop automatically after this long (default: Ctrl-C)")
    ap.add_argument("--bin", type=float, default=1.0,
                    help="pressure bin width, cmH2O")
    ap.add_argument("--full-speed", type=int, default=38500,
                    help="speed to extrapolate the shutoff head to")
    ap.add_argument("--csv", default=None,
                    help="append this run's point(s) here and fit the "
                         "accumulated curve (one pressure per run)")
    args = ap.parse_args()

    if not (2000 <= args.rpm <= 20000):
        print("--rpm must be 2000..20000: this holds a constant speed via")
        print("rpm_base, and BREATH_RPM_BASE_MAX is 20000.")
        return 1

    rig = Rig(args.port, args.baud)
    samples: list[tuple[float, float]] = []

    try:
        rig.stop()
        rig.pump(1.0)

        print(f"Holding blower at a constant {args.rpm} rpm "
              "(open loop, zero amplitude).")
        for cmd in ("sim set control openrpm",
                    "sim set waveform square",
                    "sim set amplitude 0",
                    f"sim set rpm_base {args.rpm}",
                    "sim set jitter 0"):
            rig.send(cmd)
            rig.pump(0.3)

        # Clear any zero left over from a previous session before doing
        # anything else. A stale offset is invisible in the data - it just
        # shifts every reading - and one taken during a lung-disconnected run
        # (where the CPAP blows 64 slm straight through) silently made an
        # entire capture read -64 slm.
        rig.send("flow on")
        rig.send("press on")
        rig.send("flow clearzero")
        rig.pump(2.0)
        resting = rig.flow
        if resting is None:
            print("!! no flow samples - check 'flow status'")
            return 1
        print(f"Zero cleared. At rest, blower stopped: {resting:+.2f} slm.")
        if abs(resting) > 2.0:
            print("!! That is not still air. Something is flowing with the")
            print("!! blower off, so the flow axis will be biased by it.")
        else:
            rig.send("flow zero")
            rig.pump(1.5)
            print("    tared.")

        rig.send("sim start")
        print("Starting (rev-up plus any coast hold-off)...")
        rig.pump(20.0, echo=True)

        if rig.flags & ABORT_FLAGS or rig.state.lower() == "fault":
            print(f"\n!! aborted at startup: flags=0x{rig.flags:04X} "
                  f"state={rig.state}")
            return 1

        print("\nNow step the CPAP: 4, 8, 12, 16, 20 cmH2O.")
        print("Hold each for ~5 s. Ctrl-C when finished.\n")
        print(f"  {'P cmH2O':>9} {'Q slm':>8}   samples")

        end = None if args.seconds is None else time.monotonic() + args.seconds
        last_print = 0.0
        while end is None or time.monotonic() < end:
            rig.read_once()
            if rig.flow is not None and rig.press is not None:
                samples.append((rig.press, rig.flow))
            if rig.flags & ABORT_FLAGS or rig.state.lower() == "fault":
                print(f"\n!! aborted: flags=0x{rig.flags:04X}")
                break
            # One line every few seconds, not a \r overwrite: this output is
            # usually captured to a log rather than watched on a terminal,
            # and \r turns the whole capture into a single unreadable line.
            now = time.monotonic()
            if now - last_print >= 5.0:
                last_print = now
                if rig.press is not None and rig.flow is not None:
                    print(f"  {rig.press:9.2f} {rig.flow:8.2f}   "
                          f"{len(samples):6d}", flush=True)

    except KeyboardInterrupt:
        print("\n\nstopped by user")
    finally:
        rig.stop()
        rig.close()

    if len(samples) < 20:
        print("\nToo few samples to form a curve.")
        return 1

    # ---- bin by pressure --------------------------------------------------
    bins: dict[int, list[float]] = {}
    for p, q in samples:
        k = int(round(p / args.bin))
        bins.setdefault(k, []).append(q)

    print(f"\n\n--- P-Q curve at {args.rpm} rpm ---")
    print(f"  {'P cmH2O':>9} {'Q slm':>9} {'n':>6}")
    curve: list[tuple[float, float]] = []
    for k in sorted(bins):
        qs = sorted(bins[k])
        if len(qs) < 5:
            continue                      # too few to trust
        median = qs[len(qs) // 2]
        p_mid = k * args.bin
        curve.append((p_mid, median))
        bar = "#" * max(1, int(median / 3))
        print(f"  {p_mid:9.1f} {median:9.2f} {len(qs):6d}  {bar}")

    # Accumulate across runs. Taking one pressure per run avoids having to
    # step the CPAP inside a fixed capture window, but it means the curve is
    # built up over several invocations - so persist it rather than expecting
    # anyone to collate numbers by hand.
    if args.csv:
        import csv
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["rpm", "p_cmh2o", "q_slm", "samples"])
            for p_mid, median in curve:
                n = len(bins[int(round(p_mid / args.bin))])
                w.writerow([args.rpm, f"{p_mid:.2f}", f"{median:.3f}", n])
        print(f"\n  appended {len(curve)} point(s) to {args.csv}")

        rows: list[tuple[float, float]] = []
        with open(args.csv, newline="") as fh:
            for r in csv.DictReader(fh):
                if int(r["rpm"]) == args.rpm:
                    rows.append((float(r["p_cmh2o"]), float(r["q_slm"])))
        if len(rows) > len(curve):
            rows.sort()
            print(f"\n--- accumulated curve at {args.rpm} rpm "
                  f"({len(rows)} points) ---")
            print(f"  {'P cmH2O':>9} {'Q slm':>9}")
            for p_v, q_v in rows:
                bar = "#" * max(0, int(q_v / 3))
                print(f"  {p_v:9.1f} {q_v:9.2f}  {bar}")
            curve = rows

    fit = fit_shutoff(curve)
    if fit is None:
        print("\nNot enough spread yet to fit a falling curve - needs points")
        print("at three or more pressures, with flow falling as pressure rises.")
        return 0

    q0, p_shutoff = fit
    print(f"\n  free-delivery flow (P=0):  {q0:.1f} slm")
    print(f"  shutoff head   (Q=0):      {p_shutoff:.1f} cmH2O")

    scale = (args.full_speed / args.rpm) ** 2
    print(f"\n  Fan affinity law: head scales with speed squared, so at "
          f"{args.full_speed} rpm")
    print(f"  the shutoff head should be about "
          f"{p_shutoff * scale:.1f} cmH2O "
          f"(x{scale:.2f}).")
    print("  Confirm by re-running at a second speed before relying on it.")
    print("\n  For therapy testing you need useful flow AT the test pressure,")
    print("  so the shutoff head must sit well above your highest set point.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
