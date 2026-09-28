#!/usr/bin/env python3
"""Measure delivered TIDAL VOLUME against the CPAP pressure in use.

Why not flow: the rig's circuit is closed (CPAP - hose - chamber - blower -
sealed test lung). At a constant blower speed the lung reaches equilibrium and
net flow goes to zero, whatever the pressure - there is nowhere for continuous
flow to go. Steady-state flow is therefore not a measure of anything here, and
a P-Q curve cannot be taken in this circuit at all.

What does matter is how much gas the blower can move into the lung per breath,
and how that falls as the CPAP pre-inflates the lung onto the stiffer part of
its compliance curve. That is tidal volume, and the firmware already measures
it: BreathSim integrates positive inspiratory flow and reports it on the V
line, in open loop as well as closed.

One CPAP pressure per run; appends to a CSV so the curve builds up.

    python tidal_point.py --port COM15 --amp 24000 --csv tidal.csv
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
    ap = argparse.ArgumentParser(description="Tidal volume vs CPAP pressure")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--base", type=int, default=14500)
    ap.add_argument("--amp", type=int, default=24000,
                    help="open-loop amplitude; full drive by default")
    ap.add_argument("--rate", type=int, default=12)
    ap.add_argument("--ie", type=int, default=2)
    ap.add_argument("--breaths", type=int, default=6)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    breaths: list[tuple[float, float, float, int]] = []   # vt, pmean, qpeak, flags
    flow = press = None
    peak_meas = [0.0]          # list so the nested pump() can update it

    def send(c: str) -> None:
        ser.write((c + "\r\n").encode("ascii"))
        ser.flush()

    def pump(sec: float, echo: bool = False) -> None:
        nonlocal flow, press
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
                if line.startswith("F,") and len(p) == 7:
                    flow = float(p[4])
                    # Real measured peak. The V line's q_peak_lpm is the
                    # COMMANDED peak derived from the tidal parameter, so in
                    # open loop it says nothing about what was delivered.
                    if flow > peak_meas[0]:
                        peak_meas[0] = flow
                elif line.startswith("P,") and len(p) == 9:
                    press = float(p[4])
                elif line.startswith("V,") and len(p) >= 8:
                    vt = float(p[2])
                    pmean = float(p[5])
                    qpeak = float(p[6])
                    fl = int(p[7], 16)
                    breaths.append((vt, pmean, qpeak, fl))
                    print(f"    breath {int(p[1]):3d}:  Vt {vt:6.1f} mL   "
                          f"P {pmean:5.2f} cmH2O   Qpk {qpeak:5.1f}   "
                          f"{decode(fl)}")
            except (ValueError, IndexError):
                pass

    try:
        send("sim stop")
        send("flow on")
        send("press on")
        send("flow clearzero")
        pump(2.0)
        if flow is None:
            print("!! no flow samples - check 'flow status'")
            return 1
        print(f"At rest, blower stopped: {flow:+.2f} slm, "
              f"{press if press is not None else float('nan'):.2f} cmH2O")
        if abs(flow) > 2.0:
            print("!! not still air - the volume figures will be biased")
        else:
            send("flow zero")
            pump(1.0)

        print(f"\nOpen loop, sine, base {args.base} + amp {args.amp}, "
              f"{args.rate} bpm I:E 1:{args.ie}")
        for c in ("sim set control openrpm",
                  "sim set waveform sine",
                  "sim set jitter 0",
                  # Zero the pauses explicitly. They are sticky, and a
                  # leftover pause shortens the inspiratory time, which
                  # changes delivered volume for reasons that have nothing
                  # to do with what is being measured. This silently
                  # confounded the first over-modulation comparison: Ti was
                  # 1.0 s in one run and 1.667 s in the next.
                  "sim set insp_pause 0",
                  "sim set exp_pause 0",
                  f"sim set rate {args.rate}",
                  f"sim set ie {args.ie}",
                  f"sim set rpm_base {args.base}",
                  f"sim set amplitude {args.amp}"):
            send(c)
            pump(0.3)

        send("sim start")
        print("Starting (rev-up plus any coast hold-off)...\n")
        pump(22.0)

        period = 60.0 / args.rate
        pump(period * (args.breaths + 1))
    finally:
        send("sim stop")
        send("flow off")
        send("press off")
        time.sleep(0.4)
        ser.close()

    good = [b for b in breaths if not (b[3] & 0x0008)]
    if len(good) < 2:
        print("\nToo few complete breaths to report.")
        return 1

    # Drop the first: the lung is still settling toward its working volume.
    use = good[1:] if len(good) > 2 else good
    vt = statistics.median(v for v, _, _, _ in use)
    pm = statistics.median(p for _, p, _, _ in use)
    qp = statistics.median(q for _, _, q, _ in use)
    fl = 0
    for _, _, _, f in use:
        fl |= f

    print(f"\n--- {len(use)} breaths at {pm:.2f} cmH2O ---")
    print(f"  delivered tidal volume    : {vt:.1f} mL   (measured, integrated)")
    print(f"  peak inspiratory flow     : {peak_meas[0]:.1f} slm   (measured)")
    print(f"  commanded peak (Vt param) : {qp:.1f} slm   (ignore in open loop)")
    print(f"  flags                     : {decode(fl)}")

    if args.csv:
        import csv
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(["p_cmh2o", "tidal_ml", "qpeak_slm",
                            "base_rpm", "amp_rpm", "rate_bpm", "flags"])
            w.writerow([f"{pm:.2f}", f"{vt:.1f}", f"{qp:.1f}",
                        args.base, args.amp, args.rate, f"0x{fl:04X}"])
        print(f"\n  appended to {args.csv}")

        rows = []
        with open(args.csv, newline="") as fh:
            for r in csv.DictReader(fh):
                rows.append((float(r["p_cmh2o"]), float(r["tidal_ml"])))
        if len(rows) > 1:
            rows.sort()
            print(f"\n--- tidal volume vs CPAP pressure ({len(rows)} points) ---")
            print(f"  {'P cmH2O':>9} {'Vt mL':>8}")
            for p_v, v_v in rows:
                print(f"  {p_v:9.1f} {v_v:8.1f}  {'#' * max(0, int(v_v / 20))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
