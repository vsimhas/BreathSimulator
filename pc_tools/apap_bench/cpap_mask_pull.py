#!/usr/bin/env python3
"""#16 / #64: real mask pulls on our CPAP (COM13), cued by beeps.

    python cpap_mask_pull.py [pressures] [pulls]     default 4,8,12  2

Circuit as normal (hose -> vent -> lung), simulator stopped (ff static).
At each pressure therapy starts and settles 25 s, then per pull:
    ONE HIGH BEEP     open the circuit after the vent, keep it open
    (15 s later)
    TWO LOW BEEPS     reconnect
    (30 s to settle - the mask-on is held 20 s before it is logged)
The script polls `therapy mask` ~3x/s and reports, per pull, how long after
the cue mask-off was detected, which detector path fired, how long after the
reconnect cue mask-on came back, and the episode the firmware logged.
"""
import sys
import time

import serial

try:
    import winsound

    def beep(f, ms):
        winsound.Beep(f, ms)
except Exception:                                  # pragma: no cover
    def beep(f, ms):
        print("\a", end="", flush=True)

PRESSURES = [float(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "4,8,12").split(",")]
PULLS = int(sys.argv[2]) if len(sys.argv) > 2 else 2
STAMP = time.strftime("%Y%m%d_%H%M%S")
LOG = "runs/cpap_mask_pull_%s.log" % STAMP
KEEP = ("#", "S,")


class Cpap:
    def __init__(self):
        self.s = serial.Serial("COM13", 115200, timeout=0.02)
        self.cmd("telem off", 0.4)

    def cmd(self, c, dur=0.25):
        self.s.reset_input_buffer()
        self.s.write((c + "\r\n").encode("ascii"))
        out, buf, t0 = [], b"", time.time()
        while time.time() - t0 < dur:
            buf += self.s.read(4096)
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                ln = ln.decode("ascii", "replace").strip()
                if ln.startswith(KEEP):
                    out.append(ln)
        return out

    def line(self, c, prefix, dur=0.3):
        for ln in self.cmd(c, dur):
            if ln.startswith(prefix):
                return ln
        return ""


def f(line, key):
    for tok in line.split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


def say(log, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


def watch(c, secs, want_open):
    """Poll until the mask state matches want_open; (seconds, line) or None."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        ln = c.line("therapy mask", "# MASK")
        if f(ln, "open") == ("1" if want_open else "0"):
            return round(time.monotonic() - t0, 2), ln
    return None, c.line("therapy mask", "# MASK")


def state(c):
    return f(c.line("therapy ramp status", "# RAMP", 0.4), "st")


def main():
    c = Cpap()
    with open(LOG, "w", encoding="utf-8") as log:
        say(log, "mask pulls at %s cmH2O, %d each" % (PRESSURES, PULLS))
        try:
            for p in PRESSURES:
                for x in ("therapy set mode cpap", "therapy set ramp 0", "therapy set epr 0",
                          "therapy set pressure %.1f" % p):
                    c.cmd(x)
                c.cmd("therapy start", 0.6)
                say(log, "\n== %g cmH2O: settling 25 s" % p)
                time.sleep(25.0)
                say(log, "   state %s, %s" % (state(c), c.line("therapy mask", "# MASK")))
                for k in range(PULLS):
                    beep(1500, 400)                       # OPEN
                    say(log, "   pull %d: OPEN cue" % (k + 1))
                    t_off, ln_off = watch(c, 15.0, True)
                    say(log, "     mask-off detected %s s after the cue" % t_off)
                    rest = 15.0 - (t_off or 15.0)
                    time.sleep(max(0.0, rest))
                    beep(600, 250)                        # RECONNECT
                    time.sleep(0.1)
                    beep(600, 250)
                    say(log, "     RECONNECT cue (state %s)" % state(c))
                    t_on, ln_on = watch(c, 20.0, False)
                    say(log, "     mask-on detected %s s after the cue" % t_on)
                    time.sleep(24.0)                      # past the 20 s hold
                    ln = c.line("therapy mask", "# MASK")
                    say(log, "     logged: n=%s off_s=%s on_s=%s dur=%s path=%s end=%s rows=%s" % (
                        f(ln, "n"), f(ln, "last_off_s"), f(ln, "last_on_s"), f(ln, "dur"),
                        f(ln, "path"), f(ln, "end"), f(ln, "rows")))
                c.cmd("therapy set pressure 4", 0.3)
                time.sleep(6.0)
                c.cmd("therapy stop", 0.5)
                time.sleep(4.0)
                ln = c.line("therapy mask", "# MASK")
                say(log, "   after stop: n=%s rows=%s err=%s" % (f(ln, "n"), f(ln, "rows"), f(ln, "err")))
            for ln in c.cmd("therapy edf status", 0.8):
                if ln.startswith("# EDF"):
                    say(log, "edf: eve=%s everec=%s eveerr=%s evedrop=%s" % (
                        f(ln, "eve"), f(ln, "everec"), f(ln, "eveerr"), f(ln, "evedrop")))
        finally:
            c.cmd("therapy set pressure 4", 0.3)
            time.sleep(3.0)
            c.cmd("therapy stop", 0.5)
            c.s.close()
            for _ in range(3):
                beep(1000, 120)


if __name__ == "__main__":
    main()
