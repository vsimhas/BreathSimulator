#!/usr/bin/env python3
"""Blower pressure-flow characterisation against a calibrated orifice.

This is the measurement the closed rig could not make. With the sealed test
lung replaced by an orifice there is a real flow path, so a constant blower
speed produces a steady operating point instead of equilibrating to zero flow
(which is what made pq_curve.py read 0.00 slm at every pressure).

What it answers: whether this blower can hold CPAP therapeutic pressure while
supplying patient peak inspiratory flow plus vent flow - roughly 80-100 L/min
at 20 cmH2O. The simulator's own duty (cyclic flow into a sealed lung) tells
you almost nothing about that, because the limits there are acceleration and
regen rather than pressure capability.

Method: amplitude 0 so the blower holds ONE speed, then step the speed and
record the settled (pressure, flow) at each. With a fixed orifice this traces
the SYSTEM curve - the orifice's load line, P roughly proportional to Q^2 -
not a constant-speed fan curve. To get the fan curve family, repeat with two
or three different orifices and pass a different --orifice label each time;
all runs accumulate in one CSV.

    python orifice_sweep.py --port COM15 --orifice 3mm --csv orifice.csv

SETUP
  - Test lung removed, calibrated orifice fitted in its place.
  - CPAP disconnected or switched off, so the blower alone sets the pressure.
    Leaving it running makes the pressure axis meaningless.
  - Needs the build where BREATH_RPM_BASE_MAX is BREATH_RPM_ABS_MAX,
    otherwise constant speed is capped at 20000 rpm.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required:  pip install pyserial")

FLAG_NAMES = {
    0x0001: "SLEW_LIMITED", 0x0002: "TRACK_ERROR", 0x0004: "BELOW_OBS_MIN",
    0x0008: "MOTOR_FAULT", 0x0010: "TICK_OVERRUN", 0x0020: "VBUS_DERATE",
    0x0040: "FLOW_INVALID", 0x0080: "FLOW_SAT", 0x0100: "PRESSURE_LIMIT",
    0x0200: "VOLUME_SHORT", 0x0400: "FLOW_NO_RESPONSE",
}


def decode(f: int) -> str:
    hits = [n for b, n in sorted(FLAG_NAMES.items()) if f & b]
    return "|".join(hits) if hits else "none"


def main() -> int:
    ap = argparse.ArgumentParser(description="Blower P-Q against an orifice")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--orifice", required=True,
                    help="label for this orifice, e.g. 3mm - kept in the CSV")
    ap.add_argument("--start", type=int, default=10000)
    ap.add_argument("--stop", type=int, default=38500)
    ap.add_argument("--step", type=int, default=2500)
    ap.add_argument("--dwell", type=float, default=8.0)
    ap.add_argument("--target-p", type=float, default=20.0,
                    help="CPAP pressure to assess, cmH2O")
    ap.add_argument("--target-q", type=float, default=100.0,
                    help="flow needed at that pressure, L/min")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    st = {"press": None, "flow": None, "flags": 0, "state": "?", "act": 0}
    ps: list[float] = []
    qs: list[float] = []
    acts: list[int] = []

    def send(c: str) -> None:
        ser.write((c + "\r\n").encode("ascii"))
        ser.flush()

    def pump(sec: float, collect: bool = False, echo: bool = False) -> None:
        end = time.monotonic() + sec
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#") or line.startswith("["):
                if echo:
                    print("    " + line)
                continue
            p = line.split(",")
            try:
                if line.startswith("P,") and len(p) == 9:
                    st["press"] = float(p[4])
                    if collect:
                        ps.append(st["press"])
                elif line.startswith("F,") and len(p) == 7:
                    st["flow"] = float(p[4])
                    if collect:
                        qs.append(st["flow"])
                elif line.startswith("B,") and len(p) >= 9:
                    st["flags"] = int(p[5], 16)
                    st["act"] = int(p[7])
                    if collect:
                        acts.append(st["act"])
                elif line.startswith("S,") and len(p) >= 3:
                    st["state"] = p[2]
            except (ValueError, IndexError):
                pass

    rows: list[tuple[int, int, float, float, int]] = []
    try:
        send("sim stop")
        send("press on")
        send("flow on")
        send("flow clearzero")
        pump(2.5)

        rest_q = st["flow"]
        rest_p = st["press"]
        print(f"At rest, blower stopped: "
              f"{rest_q if rest_q is not None else float('nan'):+.2f} slm, "
              f"{rest_p if rest_p is not None else float('nan'):.2f} cmH2O")
        if rest_q is None or rest_p is None:
            print("!! no sensor data - check 'flow status' / 'press status'")
            return 1
        if abs(rest_q) > 2.0:
            print("!! air is moving with the blower stopped. Is the CPAP still")
            print("!! running? It must be off for this measurement.")
        else:
            send("flow zero")
            pump(1.0)
        if abs(rest_p) > 1.0:
            print(f"!! {rest_p:.2f} cmH2O with the blower stopped - the CPAP")
            print("!! appears to be pressurising the circuit. Switch it off.")

        print(f"\nConstant speed (amplitude 0), orifice '{args.orifice}', "
              f"{args.start} -> {args.stop} rpm\n")
        for c in ("sim set control openrpm", "sim set waveform square",
                  "sim set amplitude 0", "sim set jitter 0",
                  "sim set insp_pause 0", "sim set exp_pause 0",
                  "sim set rate 6", "sim set ie 1",
                  f"sim set rpm_base {args.start}"):
            send(c)
            pump(0.3)

        send("sim start")
        print("Starting (rev-up plus any coast hold-off)...")
        pump(22.0, echo=True)
        if (st["flags"] & 0x0008) or st["state"].lower() == "fault":
            print(f"!! faulted at startup: {decode(st['flags'])}")
            return 1

        print(f"  {'rpm':>7} {'act':>7} {'P cmH2O':>9} {'Q slm':>8} "
              f"{'n':>5}   flags")
        print("  " + "-" * 62)

        rpm = args.start
        while rpm <= args.stop:
            send(f"sim set rpm_base {rpm}")
            pump(2.5)                      # latch at the breath boundary
            ps.clear(); qs.clear(); acts.clear()
            pump(args.dwell, collect=True)

            if (st["flags"] & 0x0008) or st["state"].lower() == "fault":
                print(f"\n  !! MOTOR FAULT at {rpm} rpm: {decode(st['flags'])}")
                break
            if len(ps) < 5 or len(qs) < 5:
                print(f"  {rpm:7d} {'':>7} {'--':>9} {'--':>8} "
                      f"{min(len(ps), len(qs)):5d}   too few samples")
                rpm += args.step
                continue

            p_med = statistics.median(ps)
            q_med = statistics.median(qs)
            a_med = int(statistics.median(acts)) if acts else 0
            rows.append((rpm, a_med, p_med, q_med, st["flags"]))
            print(f"  {rpm:7d} {a_med:7d} {p_med:9.2f} {q_med:8.2f} "
                  f"{len(ps):5d}   {decode(st['flags'])}")
            rpm += args.step
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        send("sim stop")
        send("press off")
        send("flow off")
        time.sleep(0.5)
        ser.close()

    if not rows:
        print("\nNo usable points.")
        return 1

    best_p = max(r[2] for r in rows)
    best_q = max(r[3] for r in rows)
    print(f"\n--- orifice '{args.orifice}' ---")
    print(f"  highest pressure reached : {best_p:.2f} cmH2O")
    print(f"  highest flow reached     : {best_q:.2f} slm")

    met = [r for r in rows if r[2] >= args.target_p and r[3] >= args.target_q]
    print(f"\n  CPAP-style requirement: >= {args.target_q:.0f} L/min "
          f"at >= {args.target_p:.0f} cmH2O")
    if met:
        r = met[0]
        print(f"  MET at {r[0]} rpm: {r[2]:.2f} cmH2O, {r[3]:.2f} slm")
    else:
        print("  NOT met on this orifice. That is not conclusive on its own -")
        print("  a fixed orifice traces one load line, and the requirement")
        print("  point may simply lie off it. Repeat with a smaller orifice")
        print("  (higher pressure, lower flow) and a larger one before")
        print("  concluding anything about the blower.")

    if args.csv:
        import csv
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["orifice", "rpm_cmd", "rpm_act",
                            "p_cmh2o", "q_slm", "flags"])
            for rpm, a, p_v, q_v, fl in rows:
                w.writerow([args.orifice, rpm, a, f"{p_v:.2f}",
                            f"{q_v:.2f}", f"0x{fl:04X}"])
        print(f"\n  appended {len(rows)} rows to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
