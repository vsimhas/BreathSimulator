#!/usr/bin/env python3
"""#85 learn-circuit bench check on our CPAP (COM13).

    python cpap_circuit_learn.py closed   # circuit connected: must refuse
                                          # (not_open), plus the busy / abort /
                                          # start-interlock paths
    python cpap_circuit_learn.py open     # tube end open: learn, and compare
                                          # with the external sensor (COM4) if
                                          # it is fitted downstream
    python cpap_circuit_learn.py status | reset

The firmware owns the speed steps and their limits (<= 26000 rpm, 10 cmH2O,
140 L/min, 0.8 A); this script only starts the run and watches it. Stop the
simulator (ff static) first: nothing here drives it.
"""
import sys
import time

import serial

PORT = "COM13"
KEEP = ("#", "ACK", "ERR", "ack", "err", "S,")
LOG = "runs/cpap_circuit_learn_%s.log"


class Cpap:
    def __init__(self):
        self.s = serial.Serial(PORT, 115200, timeout=0.05)
        self.cmd("telem off", 0.5)

    def cmd(self, c, dur=1.0):
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

    def circuit(self):
        """(fields of the # CIRCUIT line, list of point lines)."""
        head, pts = {}, []
        for ln in self.cmd("circuit status", 0.6):
            if ln.startswith("# CIRCUIT pt="):
                pts.append(ln)
            elif ln.startswith("# CIRCUIT "):
                for tok in ln[10:].split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        head[k] = v
        return head, pts

    def state(self):
        for ln in self.cmd("therapy ramp status", 0.6):
            if ln.startswith("# RAMP"):
                return [t for t in ln.split() if t.startswith("st=")][0][3:]
        return "?"


def say(log, *a):
    line = " ".join(str(x) for x in a)
    print(line)
    log.write(line + "\n")
    log.flush()


def watch(c, log, max_s=40.0, abort_at=None, try_start_at=None):
    """Poll until done; return (head, points, step start times)."""
    t0 = time.monotonic()                  # same clock as ext_sensor.py
    last, marks = None, []
    while time.monotonic() - t0 < max_s:
        el = time.monotonic() - t0
        if (abort_at is not None) and el >= abort_at:
            say(log, "   %.1f s: circuit abort" % el, c.cmd("circuit abort", 0.4))
            abort_at = None
        if (try_start_at is not None) and el >= try_start_at:
            c.cmd("therapy start", 0.6)
            say(log, "   %.1f s: therapy start during learn -> state %s" % (el, c.state()))
            try_start_at = None
        h, pts = c.circuit()
        key = (h.get("ph"), h.get("step"))
        if key != last:
            marks.append((time.monotonic(), key))
            say(log, "   %.1f s  ph=%s step=%s" % (el, key[0], key[1]))
            last = key
        if h.get("ph") in ("done", "idle") and el > 1.0:
            return h, pts, marks
        time.sleep(0.25)
    return h, pts, marks


def show(log, h, pts):
    say(log, "   result %s warn=%s applied=%s n=%s stop=%s" % (
        h.get("res"), h.get("warn"), h.get("applied"), h.get("n"), h.get("stop")))
    say(log, "   fit R_lam %s R_turb %s  d60 %s (was %s) d100 %s rms %s  p0 %s q0 %s" % (
        h.get("r_lam"), h.get("r_turb"), h.get("d60"), h.get("prev_d60"),
        h.get("d100"), h.get("rms"), h.get("p0"), h.get("q0")))
    say(log, "   in use: %s %s / %s  csv rows %s err %s" % (
        h.get("use"), h.get("use_lam"), h.get("use_turb"), h.get("csv"), h.get("csverr")))
    for p in pts:
        say(log, "   ", p)


def closed(c, log):
    say(log, "== 1. circuit connected: learn must stop at the first step (not_open)")
    say(log, "  ", c.cmd("circuit learn", 0.6))
    h, pts, _ = watch(c, log, try_start_at=1.0)
    show(log, h, pts)
    time.sleep(3.0)

    say(log, "== 2. abort during the first step")
    say(log, "  ", c.cmd("circuit learn", 0.6))
    h, pts, _ = watch(c, log, abort_at=2.5)
    show(log, h, pts)
    time.sleep(3.0)

    say(log, "== 3. learn while therapy runs (4 cmH2O): must be refused (busy)")
    for x in ("therapy set ramp 0", "therapy set epr 0", "therapy set pressure 4"):
        c.cmd(x, 0.4)
    c.cmd("therapy start", 0.8)
    time.sleep(8.0)
    say(log, "   state", c.state())
    say(log, "  ", c.cmd("circuit learn", 0.8))
    c.cmd("therapy stop", 0.8)
    time.sleep(3.0)
    say(log, "   state after stop", c.state())


def open_run(c, log):
    ext = None
    try:
        sys.path.insert(0, ".")
        from ext_sensor import ExtSensorLogger
        ext = ExtSensorLogger(LOG.replace(".log", "_ext.csv") % stamp)
        ext.start()
        time.sleep(1.0)
        say(log, "   external sensor logging (COM4)")
    except Exception as e:                       # sensor not fitted / port busy
        say(log, "   no external sensor:", e)
        ext = None
    say(log, "== learn, tube end open")
    say(log, "  ", c.cmd("circuit learn", 0.6))
    h, pts, marks = watch(c, log)
    show(log, h, pts)
    if ext is None:
        return
    time.sleep(1.0)
    ext.stop()
    s = ext.samples
    # Each step samples for the last 1.0 s before the next one starts; take
    # 0.8 s of that (host time of the status change, +-0.25 s poll).
    ends = [t for t, k in marks if k[0] in ("step", "rampdown")][1:]
    say(log, "   external sensor at each step (P_dev - P_ext is the drop to it):")
    for i, p in enumerate(pts):
        if i >= len(ends):
            break
        t1 = ends[i] - 0.3
        win = [x for x in s if t1 - 0.8 <= x[0] <= t1]
        if not win:
            continue
        pe = sum(x[1] for x in win) / len(win)
        qe = sum(x[2] for x in win) / len(win)
        f = dict(t.split("=") for t in p[10:].split())
        say(log, "   pt %s q %s ext_q %.1f  p_dev %s  p_ext %.3f  drop %.3f  fit %s" % (
            f["pt"], f["q"], qe, f["p"], pe, float(f["p"]) - pe, f["fit"]))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "status"
    stamp = time.strftime("%Y%m%d_%H%M%S")
    c = Cpap()
    with open(LOG % (what + "_" + stamp), "w", encoding="utf-8") as log:
        try:
            if what == "closed":
                closed(c, log)
            elif what == "open":
                open_run(c, log)
            elif what == "reset":
                say(log, c.cmd("circuit reset", 0.8))
            h, pts = c.circuit()
            say(log, "== final")
            show(log, h, pts)
        finally:
            c.cmd("circuit abort", 0.4)
            c.s.close()
