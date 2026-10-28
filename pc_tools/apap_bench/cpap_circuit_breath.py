#!/usr/bin/env python3
"""#85: does the learned tube make the mask-pressure estimate more right?

Circuit connected (vent + external sensor COM4 + lung), simulator breathing,
CPAP 12 cmH2O. Three segments, tube coefficients switched live:
    A  learned  (what `circuit learn` applied; read back from `circuit status`)
    B  built-in (0.00353 / 0.000118)
    A2 learned again (drift check)
Each: settle 15 s, capture 40 s. The device's P_mask (T line) is compared with
the external pressure by external-flow band, after aligning the two clocks
(arrival-time offset, refined by correlation).

    python cpap_circuit_breath.py [pressure] [tidal_ml]
"""
import statistics
import sys
import threading
import time

ARGS = sys.argv[1:]
del sys.argv[1:]                    # phase1 takes argv[1] as its port
from phase1 import send, ramp_down  # noqa: E402
import serial                       # noqa: E402
from ext_sensor import ExtSensorLogger, align, looks_frozen  # noqa: E402

P = float(ARGS[0]) if ARGS else 12.0
TIDAL = int(ARGS[1]) if len(ARGS) > 1 else 700
DEFAULT = (0.00353, 0.000118)
STAMP = time.strftime("%Y%m%d_%H%M%S")
LOG = "runs/cpap_circuit_breath_%s.log" % STAMP
SETTLE, DUR = 15.0, 40.0


class Dev(threading.Thread):
    """Reads COM13 continuously: T lines with host arrival time, # lines."""

    def __init__(self):
        super().__init__(daemon=True)
        self.s = serial.Serial("COM13", 115200, timeout=0.05)
        self.t = []                  # (host_s, dev_ms, p_sensor, q_patient, p_mask)
        self.info = []
        self.raw = None              # optional file: every line with host time
        self.stop_ev = threading.Event()

    def run(self):
        buf = b""
        while not self.stop_ev.is_set():
            buf += self.s.read(8192)
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                now = time.monotonic()
                ln = ln.decode("ascii", "replace").strip()
                if self.raw is not None:
                    self.raw.write("%.3f %s\n" % (now, ln))
                if ln.startswith("T,"):
                    f = ln.split(",")
                    try:
                        self.t.append((now, float(f[1]), float(f[2]), float(f[3]), float(f[6])))
                    except (ValueError, IndexError):
                        pass
                elif ln.startswith("#"):
                    self.info.append((now, ln))

    def cmd(self, c, wait=0.4):
        self.s.write((c + "\r\n").encode("ascii"))
        time.sleep(wait)

    def last(self, prefix, since):
        for t, ln in reversed(self.info):
            if t >= since and ln.startswith(prefix):
                return ln
        return ""


def field(line, key):
    for tok in line.split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


def say(log, *a):
    line = " ".join(str(x) for x in a)
    print(line, flush=True)
    log.write(line + "\n")
    log.flush()


def analyse(log, name, dev, ext, t0, t1):
    rows = [r for r in dev if t0 - 3.0 <= r[0] <= t1 + 3.0]
    ex = [e for e in ext if t0 <= e[0] <= t1]
    if len(rows) < 500 or len(ex) < 500:
        say(log, "  %s: not enough data (dev %d, ext %d)" % (name, len(rows), len(ex)))
        return None
    offs = sorted(r[0] - r[1] / 1000.0 for r in rows)
    coarse = offs[len(offs) // 20]
    shifted = [(r[1] / 1000.0 + coarse, r[4]) for r in rows]
    lag, corr = align(shifted, [(e[0], e[1]) for e in ex])
    off = coarse + lag
    import bisect
    et = [e[0] for e in ex]

    def at(t, k):
        i = bisect.bisect_left(et, t)
        if i <= 0 or i >= len(et) or et[i] - et[i - 1] > 0.2:
            return None
        a, b = ex[i - 1], ex[i]
        return a[k] + (b[k] - a[k]) * (t - a[0]) / (b[0] - a[0])

    pairs = []
    for r in rows:
        t = r[1] / 1000.0 + off
        if not (t0 <= t <= t1):
            continue
        pe, fe = at(t, 1), at(t, 2)
        if pe is not None:
            pairs.append((fe, r[2], r[4], pe))
    say(log, "  %s: %d samples, lag %+.2f s, r %.3f" % (name, len(pairs), lag, corr))
    bands = [(-999, -20, "exp < -20"), (-20, -5, "exp -20..-5"), (-5, 5, "|Q| < 5"),
             (5, 25, "insp 5..25"), (25, 45, "insp 25..45"), (45, 999, "insp > 45")]
    out = {}
    say(log, "    %-12s %6s %9s %14s %11s %9s" % ("ext flow", "n", "Pmask-ext", "p10..p90",
                                                  "Psens-ext", "ext P"))
    for lo, hi, nm in bands:
        g = [x for x in pairs if lo <= x[0] < hi]
        if len(g) < 20:
            continue
        d = sorted(x[2] - x[3] for x in g)
        ds = statistics.median(x[1] - x[3] for x in g)
        pe = statistics.median(x[3] for x in g)
        out[nm] = statistics.median(d)
        say(log, "    %-12s %6d %+9.3f %6.2f..%-6.2f %+11.3f %9.2f" % (
            nm, len(g), out[nm], d[len(d) // 10], d[len(d) * 9 // 10], ds, pe))
    qpk = max(e[2] for e in ex)
    say(log, "    ext peak flow %.0f L/min; ext P mean %.2f" % (
        qpk, statistics.fmean(e[1] for e in ex)))
    return out


def main():
    log = open(LOG, "w", encoding="utf-8")
    ext = ExtSensorLogger(LOG.replace(".log", "_ext.csv"))
    ext.start()
    time.sleep(1.5)
    if looks_frozen(list(ext.samples)):
        ext.stop()
        sys.exit("external sensor frozen - power-cycle it")
    dev = Dev()
    dev.start()
    segs = []
    learned = None
    try:
        t = time.monotonic()
        dev.cmd("telem off", 0.3)
        dev.cmd("circuit status", 0.8)
        cl = dev.last("# CIRCUIT ph", t)
        learned = (float(field(cl, "use_lam")), float(field(cl, "use_turb")))
        say(log, "in use at start: %s %s / %s (res %s d60 %s)" % (
            field(cl, "use"), learned[0], learned[1], field(cl, "res"), field(cl, "d60")))
        if field(cl, "use") not in ("learned", "card"):
            say(log, "!! no learned tube in use - run `circuit learn` first")
            return
        for c in ("sim set waveform sine", "sim set timing rate_ie", "sim set rate 12",
                  "sim set ie 1", "sim set insp_pause 0", "sim set exp_pause 0",
                  "sim set jitter 0", "sim set flattening 0", "sim set exp_tau 0.4",
                  "sim set rpm_base 8000", "sim set tidal %d" % TIDAL, "sim set recovery 0",
                  "sim set snore_hz 0", "sim set plimit 40", "sim set control flow",
                  "sim set ff static", "sim script clear", "telem on", "sim start"):
            send(c)
        say(log, "simulator breathing, tidal %d mL, 12/min; priming 15 s" % TIDAL)
        time.sleep(15.0)
        for c in ("therapy set mode cpap", "therapy set ramp 0", "therapy set epr 0",
                  "therapy set pressure %.1f" % P):
            dev.cmd(c, 0.3)
        dev.cmd("telem on", 0.3)
        dev.cmd("therapy start", 0.5)
        say(log, "CPAP %.1f cmH2O started" % P)
        time.sleep(7.0)
        send("sim set ff pressure")
        plan = [("A learned", learned), ("B built-in", DEFAULT), ("A2 learned", learned)]
        for name, (rl, rt) in plan:
            dev.cmd("tubecomp set R_lam %.6f" % rl, 0.2)
            dev.cmd("tubecomp set R_turb %.7f" % rt, 0.2)
            say(log, "== %s: R_lam %.5f R_turb %.6f" % (name, rl, rt))
            time.sleep(SETTLE)
            t0 = time.monotonic()
            time.sleep(DUR)
            t1 = time.monotonic()
            ts = time.monotonic()
            dev.cmd("therapy ramp status", 0.5)
            say(log, "   state after: %s" % field(dev.last("# RAMP", ts), "st"))
            segs.append((name, t0, t1))
    finally:
        try:
            ramp_down()
            send("sim set ff static")
            send("sim set plimit 0")
        except Exception as e:
            say(log, "sim stop: %s" % e)
        if learned is not None:                  # leave the learned tube in use
            dev.cmd("tubecomp set R_lam %.6f" % learned[0], 0.2)
            dev.cmd("tubecomp set R_turb %.7f" % learned[1], 0.2)
        dev.cmd("therapy set pressure 4", 0.3)
        time.sleep(8.0)
        dev.cmd("therapy stop", 0.5)
        time.sleep(3.0)
        dev.cmd("telem off", 0.3)
        ts = time.monotonic()
        dev.cmd("circuit status", 0.8)
        say(log, "after: " + dev.last("# CIRCUIT ph", ts))
        dev.stop_ev.set()
        ext.stop()
    say(log, "\nP_mask estimate minus external pressure (median by external flow band)")
    res = {}
    for name, t0, t1 in segs:
        res[name] = analyse(log, name, dev.t, list(ext.samples), t0, t1)
    log.close()


if __name__ == "__main__":
    main()
