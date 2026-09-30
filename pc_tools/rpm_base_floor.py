#!/usr/bin/env python3
"""Find the lowest usable rpm_base, and what it does to the pressure offset.

Why this matters: the lung sees ~9.4 cmH2O more than the CPAP is set to, at
every pressure, because rpm_base never stops pushing. That offset is what
drives the circuit into the blower's own stall region above CPAP 8. Lowering
rpm_base is the single biggest lever on this rig.

The obstacle is the sensorless observer: back-EMF falls with speed, so below
some point the STO/PLL loses the rotor and MC_SPEED_FDBK fires.
OBS_MINIMUM_SPEED_RPM (12965) is the ST profiler's conservative guess at that
point, but in BreathSim it only raises BREATH_FLAG_BELOW_OBS_MIN - it is not a
clamp - so the real floor can be measured without rebuilding anything.

Method: hold a constant speed (amplitude 0), step rpm_base down, and at each
step record whether the drive held and what the airway pressure settled at.
Stops at the first motor fault, and always stops the blower on the way out.

    python rpm_base_floor.py --port COM15
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
    ap = argparse.ArgumentParser(description="Lowest usable rpm_base")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--start", type=int, default=14500)
    ap.add_argument("--floor", type=int, default=6000)
    ap.add_argument("--step", type=int, default=1000)
    ap.add_argument("--dwell", type=float, default=8.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    st = {"press": None, "flags": 0, "state": "?", "rpm_cmd": 0, "rpm_act": 0}
    samples: list[float] = []
    rpms: list[int] = []

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
                    if collect and st["press"] is not None:
                        samples.append(st["press"])
                elif line.startswith("B,") and len(p) >= 9:
                    st["flags"] = int(p[5], 16)
                    st["rpm_cmd"] = int(p[6])
                    st["rpm_act"] = int(p[7])
                    if collect:
                        rpms.append(st["rpm_act"])
                elif line.startswith("S,") and len(p) >= 3:
                    st["state"] = p[2]
            except (ValueError, IndexError):
                pass

    results: list[tuple[int, float, int, int]] = []
    baseline = None
    try:
        send("sim stop")
        send("press on")
        send("flow on")
        pump(2.0)
        baseline = st["press"]
        print(f"CPAP pressure with blower stopped: "
              f"{baseline if baseline is not None else float('nan'):.2f} cmH2O")

        print(f"\nConstant speed (amplitude 0), stepping rpm_base "
              f"{args.start} -> {args.floor} in {args.step}\n")
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

        print(f"  {'rpm_base':>9} {'rpm_act':>8} {'P cmH2O':>9} "
              f"{'offset':>7}   flags")
        print("  " + "-" * 60)

        rpm = args.start
        while rpm >= args.floor:
            send(f"sim set rpm_base {rpm}")
            pump(2.0)                      # let the new base latch and settle
            samples.clear()
            rpms.clear()
            pump(args.dwell, collect=True)

            if (st["flags"] & 0x0008) or st["state"].lower() == "fault":
                print(f"\n  !! MOTOR FAULT at rpm_base {rpm}: "
                      f"{decode(st['flags'])}")
                break

            p = statistics.median(samples) if samples else float("nan")
            a = int(statistics.median(rpms)) if rpms else 0
            off = (p - baseline) if (baseline is not None and p == p) else float("nan")
            results.append((rpm, p, a, st["flags"]))
            print(f"  {rpm:9d} {a:8d} {p:9.2f} {off:7.2f}   "
                  f"{decode(st['flags'])}")
            rpm -= args.step
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        send("sim stop")
        send("press off")
        send("flow off")
        time.sleep(0.5)
        ser.close()

    if not results:
        print("\nNo stable points.")
        return 1

    lowest = results[-1]
    print(f"\n--- lowest rpm_base that held: {lowest[0]} ---")
    if baseline is not None:
        print(f"  airway pressure there : {lowest[1]:.2f} cmH2O")
        print(f"  offset above CPAP     : {lowest[1] - baseline:.2f} cmH2O "
              f"(was {results[0][1] - baseline:.2f} at {results[0][0]})")
    print("\n  BELOW_OBS_MIN is only a warning flag - what matters is whether")
    print("  MOTOR_FAULT / MC_SPEED_FDBK appeared. If the drive held well")
    print("  below 12965, OBS_MINIMUM_SPEED_RPM is simply conservative and")
    print("  rpm_base can be lowered with no observer retune at all.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
