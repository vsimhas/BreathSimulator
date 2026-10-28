#!/usr/bin/env python3
"""#25: is a flattened breath still flat in OUR patient-flow signal?

Flatness (inspiratory mean / peak, the FL detector's measure) of the same
breaths from two signals: the external sensor at the lung (COM4) and the
CPAP's patient flow (T line). Simulator flattening 0.9 (plateau), full-size
breaths and 55 % hypopnea breaths, at each pressure.

    python cpap_flat_ext.py [pressures]        default 4,8
"""
import statistics
import sys
import time

ARGS = sys.argv[1:]
del sys.argv[1:]
from phase1 import send, ramp_down  # noqa: E402
from ext_sensor import ExtSensorLogger, looks_frozen  # noqa: E402
from cpap_circuit_breath import Dev  # noqa: E402

PRESSURES = [float(x) for x in (ARGS[0] if ARGS else "4,8").split(",")]
STAMP = time.strftime("%Y%m%d_%H%M%S")
LOG = "runs/cpap_flat_ext_%s.log" % STAMP


def say(log, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


def breaths(ts, qs, min_ti=0.8, floor=0.0):
    """Inspirations as runs of q > floor lasting >= min_ti; (t0, mean/peak, peak)."""
    out, start = [], None
    for i, q in enumerate(qs):
        if q > floor and start is None:
            start = i
        elif q <= floor and start is not None:
            seg = qs[start:i]
            if ts[i - 1] - ts[start] >= min_ti and max(seg) > 3.0:
                out.append((ts[start], statistics.fmean(seg) / max(seg), max(seg)))
            start = None
    return out


def report(log, name, dev, ext, t0, t1):
    # device: T lines on device time (s) - flatness needs no alignment
    d = [r for r in dev if t0 + 2.0 <= r[0] <= t1 + 2.5]      # arrival lags ~2 s
    e = [x for x in ext if t0 <= x[0] <= t1]
    bd = breaths([r[1] / 1000.0 for r in d], [r[3] for r in d])
    be = breaths([x[0] for x in e], [x[2] for x in e])
    def stats(b):
        if not b:
            return "none"
        f = sorted(x[1] for x in b)
        return "n %d  flat med %.3f (%.3f..%.3f)  peak med %.1f" % (
            len(f), f[len(f) // 2], f[0], f[-1], sorted(x[2] for x in b)[len(b) // 2])
    say(log, "   %-22s ext (lung): %s" % (name, stats(be)))
    say(log, "   %-22s CPAP pat. : %s" % ("", stats(bd)))


def main():
    log = open(LOG, "w", encoding="utf-8")
    ext = ExtSensorLogger(LOG.replace(".log", "_ext.csv"))
    ext.start()
    time.sleep(1.5)
    if looks_frozen(list(ext.samples)):
        ext.stop()
        sys.exit("external sensor frozen - power-cycle it")
    dev = Dev()
    dev.raw = open(LOG.replace(".log", "_raw.log"), "w", encoding="utf-8")
    dev.start()
    try:
        dev.cmd("telem off", 0.3)
        for p in PRESSURES:
            say(log, "\n== %g cmH2O" % p)
            send("sim event cancel")
            for c in ("sim set waveform sine", "sim set timing rate_ie", "sim set rate 12",
                      "sim set ie 1", "sim set insp_pause 0", "sim set exp_pause 0",
                      "sim set jitter 0", "sim set flattening 0", "sim set fl_shape plateau",
                      "sim set exp_tau 0.4", "sim set rpm_base 8000", "sim set tidal 500",
                      "sim set recovery 0", "sim set snore_hz 0", "sim set plimit 40",
                      "sim set control flow", "sim set ff static", "sim script clear",
                      "telem on", "sim start"):
                send(c)
            time.sleep(12.0)
            for c in ("therapy set mode cpap", "therapy set ramp 0", "therapy set epr 0",
                      "therapy set pressure %.1f" % p):
                dev.cmd(c, 0.3)
            dev.cmd("telem on", 0.3)
            dev.cmd("therapy start", 0.5)
            time.sleep(7.0)
            send("sim set ff pressure")
            time.sleep(15.0)
            a = time.monotonic(); time.sleep(25.0)
            report(log, "round, full", dev.t, list(ext.samples), a, time.monotonic())
            send("sim set flattening 0.9")
            time.sleep(8.0)
            a = time.monotonic(); time.sleep(25.0)
            report(log, "flattened, full", dev.t, list(ext.samples), a, time.monotonic())
            send("sim event hypopnea 30 55")
            time.sleep(6.0)
            a = time.monotonic(); time.sleep(22.0)
            report(log, "flattened, 55 % hyp", dev.t, list(ext.samples), a, time.monotonic())
            send("sim set flattening 0")
            time.sleep(4.0)
            ramp_down()
            send("sim set ff static")
            dev.cmd("therapy set pressure 4", 0.3)
            time.sleep(6.0)
            dev.cmd("therapy stop", 0.5)
            time.sleep(4.0)
    finally:
        try:
            send("sim event cancel")
            send("sim set flattening 0")
            ramp_down()
            send("sim set ff static")
            send("sim set plimit 0")
        except Exception as ex:
            say(log, "sim stop:", ex)
        dev.cmd("therapy set pressure 4", 0.3)
        time.sleep(4.0)
        dev.cmd("therapy stop", 0.5)
        dev.cmd("telem off", 0.3)
        dev.stop_ev.set()
        ext.stop()
        log.close()


if __name__ == "__main__":
    main()
