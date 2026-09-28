#!/usr/bin/env python3
"""Measure the blower's steady-state RPM -> flow curve, and answer whether a
given peak inspiratory flow is reachable.

Method: drive the rig OPEN LOOP with a square waveform at a slow rate, so each
"inspiration" is a multi-second hold at a constant commanded RPM. Step the
amplitude up, record the settled flow at each step, then fit the curve. No
controller in the loop, so nothing here depends on PI tuning or feed-forward.

Safety: aborts on any motor fault, on the no-response flag, or on a stall in
telemetry, and always sends `sim stop` on the way out - including on Ctrl-C
or an exception.

    python blower_map.py --port COM15
    python blower_map.py --port COM15 --target 50 --max-amp 24000
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("pyserial is required:  pip install pyserial")


# BREATH_FLAG_* from breath_sim.h
FLAG_NAMES = {
    0x0001: "SLEW_LIMITED", 0x0002: "TRACK_ERROR", 0x0004: "BELOW_OBS_MIN",
    0x0008: "MOTOR_FAULT", 0x0010: "TICK_OVERRUN", 0x0020: "VBUS_DERATE",
    0x0040: "FLOW_INVALID", 0x0080: "FLOW_SAT", 0x0100: "PRESSURE_LIMIT",
    0x0200: "VOLUME_SHORT", 0x0400: "FLOW_NO_RESPONSE",
}
ABORT_FLAGS = 0x0008 | 0x0400          # MOTOR_FAULT, FLOW_NO_RESPONSE


def decode(flags: int) -> str:
    hits = [n for b, n in sorted(FLAG_NAMES.items()) if flags & b]
    return "|".join(hits) if hits else "none"


class Rig:
    def __init__(self, port: str, baud: int = 115200, verbose: bool = False):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.verbose = verbose
        self.flags = 0
        self.state = "?"
        self.segment = "?"
        self.rpm_cmd = 0
        self.rpm_act = 0
        self.flow = None
        self.press = None

    # -- plumbing ---------------------------------------------------------
    def send(self, cmd: str) -> None:
        if self.verbose:
            print(f"    > {cmd}")
        self.ser.write((cmd + "\r\n").encode("ascii"))
        self.ser.flush()

    def pump(self, seconds: float, collect: list | None = None,
             echo_comments: bool = False) -> None:
        """Read for `seconds`, updating state; append flow samples to collect."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                if echo_comments:
                    print("    " + line)
                continue
            p = line.split(",")
            try:
                if line.startswith("B,") and len(p) >= 9:
                    self.segment = p[3]
                    self.flags = int(p[5], 16)
                    self.rpm_cmd = int(p[6])
                    self.rpm_act = int(p[7])
                elif line.startswith("F,") and len(p) == 7:
                    self.flow = float(p[4])          # filtered slm
                    if collect is not None and self.segment == "Insp":
                        collect.append(self.flow)
                elif line.startswith("P,") and len(p) == 9:
                    self.press = float(p[4])         # cmH2O
                elif line.startswith("S,") and len(p) >= 3:
                    self.state = p[2]
            except (ValueError, IndexError):
                continue

    def check_abort(self) -> str | None:
        if self.flags & ABORT_FLAGS:
            return decode(self.flags & ABORT_FLAGS)
        if self.state.lower() == "fault":
            return "state=Fault"
        return None

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


def main() -> int:
    ap = argparse.ArgumentParser(description="Blower RPM -> flow characterisation")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--base", type=int, default=14500, help="rpm_base for the run")
    ap.add_argument("--max-amp", type=int, default=24000)
    ap.add_argument("--step", type=int, default=3000)
    ap.add_argument("--rate", type=int, default=6, help="bpm; sets the hold length")
    ap.add_argument("--target", type=float, default=50.0, help="peak slm to assess")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    period = 60.0 / args.rate
    dwell = period + 1.0            # one full breath per point

    rig = Rig(args.port, args.baud, args.verbose)
    results: list[tuple[int, float, int]] = []

    try:
        print(f"Connected to {args.port}.  Sending stop first.")
        rig.stop()
        rig.pump(1.0)

        print("\nSensor status:")
        rig.send("flow status")
        rig.send("press status")
        rig.pump(2.0, echo_comments=True)

        # Tare ONLY if the sensor is actually seeing still air. With the test
        # lung disconnected and a CPAP still running, the CPAP blows straight
        # through the open port - on 2026-09-28 that was 64 slm, and taring it
        # away silently offset every reading in the run by -63.87 slm. A zero
        # taken against a standing flow is worse than no zero at all, because
        # it looks like it worked.
        print("\nChecking flow is at rest before taring...")
        rig.send("flow on")
        rig.pump(1.5)
        resting = rig.flow
        if resting is None:
            print("    no flow samples arrived - skipping tare")
        elif abs(resting) > 2.0:
            print(f"    !! {resting:+.2f} slm is flowing with the blower stopped.")
            print("    !! NOT taring - that would bake this into every reading.")
            print("    !! Stop the CPAP, or accept that readings are relative.")
        else:
            print(f"    {resting:+.2f} slm at rest - taring.")
            rig.send("flow zero")
            rig.pump(1.5, echo_comments=True)

        print("\nConfiguring open-loop square hold "
              f"(base {args.base}, {args.rate} bpm, I:E 1:1)...")
        for cmd in ("sim set control openrpm",
                    "sim set waveform square",
                    f"sim set rate {args.rate}",
                    "sim set ie 1",
                    "sim set jitter 0",
                    f"sim set rpm_base {args.base}",
                    "sim set amplitude 0"):
            rig.send(cmd)
            rig.pump(0.3)

        rig.send("flow on")
        rig.send("press on")
        rig.send("sim start")
        print("Starting (allow for CM4 rev-up and any coast hold-off)...")
        rig.pump(20.0, echo_comments=True)

        abort = rig.check_abort()
        if abort:
            print(f"\n!! aborted before stepping: {abort}")
            return 1

        print(f"\n{'amp':>6} {'rpm_cmd':>8} {'rpm_act':>8} {'flow slm':>9} "
              f"{'P cmH2O':>8}  flags")
        print("-" * 62)

        amp = 0
        while amp <= args.max_amp:
            rig.send(f"sim set amplitude {amp}")
            rig.pump(0.5)
            samples: list[float] = []
            rig.pump(dwell, collect=samples)

            abort = rig.check_abort()
            if abort:
                print(f"\n!! aborted at amplitude {amp}: {abort}")
                break

            if samples:
                samples.sort()
                # 90th percentile of the inspiratory samples = the plateau,
                # ignoring the rise at the start of the hold.
                plateau = samples[int(0.9 * (len(samples) - 1))]
            else:
                plateau = float("nan")

            results.append((amp, plateau, rig.rpm_act))
            print(f"{amp:6d} {rig.rpm_cmd:8d} {rig.rpm_act:8d} {plateau:9.2f} "
                  f"{(rig.press if rig.press is not None else float('nan')):8.2f}"
                  f"  {decode(rig.flags)}")
            amp += args.step

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        print("\nStopping blower.")
        rig.stop()
        rig.close()

    # -- verdict ----------------------------------------------------------
    good = [(a, f, r) for a, f, r in results
            if f == f and r > 0]                     # drop NaN
    if len(good) < 2:
        print("\nNot enough points to fit. Was the blower running?")
        return 1

    print("\n--- measured ---")
    peak_amp, peak_flow, peak_rpm = max(good, key=lambda t: t[1])
    print(f"  highest flow seen: {peak_flow:.2f} slm "
          f"at {peak_rpm} rpm (amplitude {peak_amp})")

    # slope over the top half of the range, where the curve is most relevant
    top = good[len(good) // 2:]
    if len(top) >= 2:
        (a0, f0, r0), (a1, f1, r1) = top[0], top[-1]
        if r1 != r0:
            slope = (f1 - f0) / (r1 - r0)            # slm per rpm
            print(f"  slope near the top: {slope*1000:.2f} slm per 1000 rpm")
            if slope > 0:
                need = r1 + (args.target - f1) / slope
                print(f"  linear extrapolation to {args.target:.0f} slm: "
                      f"~{need:.0f} rpm")
                print(f"  firmware ceiling is 38500 rpm", end="")
                print("  -> REACHABLE" if need <= 38500 else
                      f"  -> NOT reachable (needs {need-38500:.0f} rpm more)")
    print("\nNote: a blower's P-Q curve flattens near its ceiling, so a linear")
    print("extrapolation is optimistic. Treat it as an upper bound.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
