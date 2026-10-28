#!/usr/bin/env python3
"""#25: flattened (obstructive-shaped) 55 % hypopneas, classed by pressure
and breath jitter. CPAP mode with scoring, simulator breathing.

    python cpap_hyp_lowp.py [cases] [cpap settings]
        cases          e.g. 4:25,4:0,8:0,8:25 (default)
        cpap settings  ';'-separated commands sent before each case, e.g.
                       "apap set rel_baseline 1"

Each case: therapy at P cmH2O, simulator jitter J %, 40 s of normal breathing,
then two 25 s hypopneas at 55 % with flattening 0.9 (40 s apart). The
detector's own per-breath log (`apap breaths`) is collected throughout, so
the report shows what each hypopnea breath looked like and why it was or was
not called flow-limited.
"""
import sys
import threading
import time

import serial

CASES = sys.argv[1] if len(sys.argv) > 1 else "4:25,4:0,8:0,8:25"
SETTINGS = [c.strip() for c in (sys.argv[2] if len(sys.argv) > 2 else "").split(";") if c.strip()]
del sys.argv[1:]
from phase1 import send, ramp_down, ser  # noqa: E402

SIM_GUARD = 36000
STAMP = time.strftime("%Y%m%d_%H%M%S")
LOG = "runs/cpap_hyp_lowp_%s.log" % STAMP


class Cpap:
    def __init__(self):
        self.s = serial.Serial("COM13", 115200, timeout=0.1)
        self.lock = threading.Lock()
        self.lines = {}
        self.breaths = {}
        self.run = True
        self.th = threading.Thread(target=self._rx, daemon=True)
        self.th.start()

    def _rx(self):
        buf = b""
        while self.run:
            try:
                buf += self.s.read(4096)
            except serial.SerialException:
                break
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                t = ln.decode("ascii", "replace").strip()
                if t.startswith("# BR "):
                    d = dict(x.split("=", 1) for x in t[5:].split() if "=" in x)
                    with self.lock:
                        self.breaths[int(d["seq"])] = d
                for key in ("# APAP ", "# RAMP "):
                    if t.startswith(key):
                        with self.lock:
                            self.lines[key] = t

    def cmd(self, c):
        self.s.write((c + "\r\n").encode("ascii"))
        time.sleep(0.15)

    def query(self, c, key):
        with self.lock:
            self.lines.pop(key, None)
        self.cmd(c)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            with self.lock:
                if key in self.lines:
                    return self.lines[key]
            time.sleep(0.05)
        return ""

    def close(self):
        self.run = False
        time.sleep(0.3)
        self.s.close()


def field(line, key):
    for tok in (line or "").split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


SIM_BREATHS = set()
SIM_RPM_MAX = [0]          # highest measured simulator rpm since the last clear


def drain_sim(seconds, cp=None, poll_s=8.0):
    end = time.monotonic() + seconds
    next_poll = time.monotonic() + poll_s
    while time.monotonic() < end:
        raw = ser.readline()
        if raw and raw.startswith(b"B,"):
            p = raw.decode("ascii", "replace").split(",")
            try:
                if len(p) >= 8 and int(p[6]) > SIM_GUARD:
                    raise RuntimeError("simulator commanded %s rpm" % p[6])
                if len(p) >= 8:
                    SIM_RPM_MAX[0] = max(SIM_RPM_MAX[0], int(p[7]))
                if len(p) >= 4 and p[3] == "Insp":
                    SIM_BREATHS.add(int(p[2]))
            except ValueError:
                pass
        if cp is not None and time.monotonic() >= next_poll:
            cp.cmd("apap breaths")
            next_poll = time.monotonic() + poll_s


def say(log, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


def run_case(cp, log, p, jitter):
    say(log, "\n== %g cmH2O, jitter %d %%" % (p, jitter))
    # ramp_down() leaves a 120 s apnea latched; it fires on the next start.
    send("sim event cancel")
    for c in ("sim set waveform sine", "sim set timing rate_ie", "sim set rate 12",
              "sim set ie 1", "sim set insp_pause 0", "sim set exp_pause 0",
              "sim set jitter %d" % jitter, "sim set flattening 0",
              "sim set fl_shape plateau", "sim set exp_tau 0.4", "sim set rpm_base 8000",
              "sim set tidal 500", "sim set recovery 0", "sim set snore_hz 0",
              "sim set plimit 40", "sim set control flow", "sim set ff static",
              "sim script clear", "telem on", "sim start"):
        send(c)
    SIM_BREATHS.clear()
    SIM_RPM_MAX[0] = 0
    drain_sim(15.0)
    # The segment names step through Insp even when the motor is not turning
    # (2026-10-28: cmd 8000, measured 0) - require real rotation too.
    if len(SIM_BREATHS) < 2 or SIM_RPM_MAX[0] < 3000:
        raise RuntimeError("simulator not breathing (inspirations %d, max measured rpm %d)"
                           % (len(SIM_BREATHS), SIM_RPM_MAX[0]))
    say(log, "   simulator breathing (%d inspirations in 15 s, up to %d rpm)"
        % (len(SIM_BREATHS), SIM_RPM_MAX[0]))
    for c in ("therapy set mode cpap", "therapy set ramp 0", "therapy set epr 0",
              "therapy set pressure %.1f" % p) + tuple(SETTINGS):
        cp.cmd(c)
    if SETTINGS:
        say(log, "   settings: %s" % "; ".join(SETTINGS))
    # The firmware's breath log survives across sessions: note where this
    # session's breaths start, and count only those.
    cp.cmd("apap breaths")
    time.sleep(1.0)
    with cp.lock:
        seq_start = max(cp.breaths) if cp.breaths else 0
        cp.breaths.clear()
    cp.cmd("therapy start")
    drain_sim(7.0, cp)
    send("sim set ff pressure")
    drain_sim(33.0, cp)
    seq0 = max(cp.breaths) if cp.breaths else 0
    windows = []
    for k in range(2):
        send("sim set flattening 0.9")
        drain_sim(2.0, cp)
        t0 = time.monotonic()
        send("sim event hypopnea 25 55")
        drain_sim(27.0, cp)
        send("sim set flattening 0")
        windows.append((t0, time.monotonic()))
        drain_sim(40.0, cp)
    cp.cmd("apap breaths")
    time.sleep(1.0)
    a = cp.query("therapy apap events", "# APAP ")
    say(log, "   events: Hyp=%s FL=%s CA=%s OA=%s" % (field(a, "Hyp"), field(a, "FL"),
                                                    field(a, "CA"), field(a, "OA")))
    with cp.lock:
        br = [cp.breaths[k] for k in sorted(cp.breaths) if k > seq_start]
    lim = [b for b in br if b["hyp"] == "1"]
    norm = [b for b in br if int(b["seq"]) > seq0 and b["hyp"] == "0" and b["small"] == "0"]

    def summary(name, g):
        if not g:
            say(log, "   %-14s none" % name)
            return
        n = len(g)
        ok = sum(b["ok"] == "1" for b in g)
        fl = sum(b["fl"] == "1" for b in g)
        fl_ok = [float(b["flat"]) for b in g if b["ok"] == "1"]
        say(log, "   %-14s n=%d  clean segment %d  (from floor %d, overflow %d)  FL-shape %d  "
            "flat(clean) %s  dip med %.2f" % (
                name, n, ok, sum(b["floor"] == "1" for b in g),
                sum(b["ovf"] == "1" for b in g), fl,
                ("%.2f..%.2f" % (min(fl_ok), max(fl_ok))) if fl_ok else "-",
                sorted(float(b["dip"]) for b in g)[n // 2]))
    summary("normal", norm)
    summary("hyp-limited", lim)
    for b in lim:
        say(log, "     seq %s ti %s T %s pk %s base %s flat %s dip %s ok %s floor %s ovf %s fl %s" % (
            b["seq"], b["ti"], b["T"], b["pk"], b["base"], b["flat"], b["dip"], b["ok"],
            b["floor"], b["ovf"], b["fl"]))
    verdict = "obstructive" if lim and sum(b["fl"] == "1" for b in lim) * 2 > len(lim) else "central"
    say(log, "   => hypopnea breaths judged %s by shape" % verdict)
    # stop
    ramp_down()
    send("sim set ff static")
    cp.cmd("therapy set pressure 4")
    time.sleep(8.0)
    cp.cmd("therapy stop")
    time.sleep(4.0)
    return br


def main():
    cp = Cpap()
    cp.cmd("telem off")
    with open(LOG, "w", encoding="utf-8") as log:
        try:
            for case in CASES.split(","):
                p, j = case.split(":")
                run_case(cp, log, float(p), int(j))
        finally:
            try:
                send("sim set flattening 0")
                ramp_down()
                send("sim set ff static")
                send("sim set plimit 0")
                send("sim set jitter 0")
            except Exception as e:
                print("sim stop:", e)
            cp.cmd("therapy set pressure 4")
            time.sleep(6.0)
            cp.cmd("therapy stop")
            cp.close()


if __name__ == "__main__":
    main()
