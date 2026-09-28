#!/usr/bin/env python3
"""Ask the rig for status and print everything it says. Motor is not touched.

    python rig_status.py --port COM15
"""
from __future__ import annotations
import argparse, sys, time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cmds", default="sim stop,telem off,sim status,flow status,press status")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    try:
        for cmd in args.cmds.split(","):
            cmd = cmd.strip()
            if not cmd:
                continue
            print(f"\n>>> {cmd}")
            ser.write((cmd + "\r\n").encode("ascii"))
            ser.flush()
            end = time.monotonic() + 1.8
            while time.monotonic() < end:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").rstrip()
                # With telem off only comments, acks and status lines remain.
                if line:
                    print("    " + line)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
