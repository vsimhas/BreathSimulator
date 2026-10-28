#!/usr/bin/env python3
"""AutoRamp (#38) and periodic-breathing / CSR (#32) bench checks on our CPAP.

Rig: CPAP (COM13) -> circuit -> breath simulator (COM15) -> 2 L lung.
CPAP mode (score-only APAP detectors), EPR off. 12 bpm, I:E 1:1, 500 mL.

  python cpap_autoramp_csr.py steady   AutoRamp, metronomic breathing:
        onset by 30 stable breaths (~15 s grace + ~150 s), then 4 -> 10
        cmH2O at 1 cmH2O/min.
  python cpap_autoramp_csr.py jitter   AutoRamp, 25 % breath-to-breath jitter
        (awake-like): no breathing onset; time limit set to 8 min, so the
        hold must end at 2 min (8 - 6 min of rise), cause 4.
  python cpap_autoramp_csr.py events   jitter 25 % + three flattened
        (obstructive) 55 % hypopneas 40 s apart: onset cause 2.
  python cpap_autoramp_csr.py snore    jitter 25 % + snore threshold lowered
        to 0.05 at 60 s (the rig cannot make a real snore): onset cause 3.
  python cpap_autoramp_csr.py csr      ramp off, 8 cmH2O: normal 300 s |
        simulator CSR (60 s cycle) 480 s | normal 360 s. pb must stay 0 in the
        normal blocks, come on inside the CSR block, and the span go to the
        CSL file (`therapy edf enable 1`).
  python cpap_autoramp_csr.py leak     #19, 8 cmH2O: the leak HIGH threshold
        is dropped to 5 slm (below the rig's ~12 slm) from 60 to 220 s, so
        the bench leak counts as large. Flattened 55 % hypopneas at 100, 150
        (inside: not scored, lx_drop counts them) and 270 s (after the
        10 s settle: scored). Also checks EDF logging is on without
        `therapy edf enable 1` (default since #19 build).
  python cpap_autoramp_csr.py rampstart  #5: ramp start 6 cmH2O, fixed 5 min
        ramp to 10 for 90 s (sp from 6, ~0.8 cmH2O/min), then AutoRamp
        holding at 6.

Stop order as the other harnesses: simulator ramp-down first, then the CPAP
via 4 cmH2O and `therapy stop`.
"""
import os
import sys
import threading
import time

import serial

# phase1 opens the simulator port from argv[1] on import: take the case first.
CASE = sys.argv[1] if len(sys.argv) > 1 else "steady"
del sys.argv[1:]
from phase1 import send, ramp_down, ser  # noqa: E402

CPAP_PORT = "COM13"
SIM_GUARD = 36000
TARGET = 10.0


class Cpap:
    def __init__(self):
        self.s = serial.Serial(CPAP_PORT, 115200, timeout=0.1)
        self.lock = threading.Lock()
        self.lines = {}
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
                for key in ("# APAP ", "# RAMP ", "# EDF "):
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
        return None

    def close(self):
        self.run = False
        time.sleep(0.3)
        self.s.close()


def drain_sim(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", "replace").strip()
        if t.startswith("B,"):
            p = t.split(",")
            try:
                if len(p) >= 8 and int(p[6]) > SIM_GUARD:
                    raise RuntimeError("simulator commanded %s rpm" % p[6])
            except ValueError:
                pass


def field(line, key):
    for tok in (line or "").split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


def sim_setup(jitter):
    for c in ("sim set waveform sine", "sim set timing rate_ie",
              "sim set rate 12", "sim set ie 1", "sim set insp_pause 0",
              "sim set exp_pause 0", "sim set jitter %d" % jitter,
              "sim set flattening 0", "sim set fl_shape plateau",
              "sim set exp_tau 0.4", "sim set rpm_base 8000",
              "sim set tidal 500", "sim set recovery 0", "sim set snore_hz 0",
              "sim set plimit 40", "sim set control flow",
              "sim set ff static", "sim script clear"):
        send(c)
    send("telem on")
    send("sim start")
    print("simulator breathing (static FF, jitter %d %%) - priming 20 s" % jitter)
    drain_sim(20.0)


def ramp_line(cp, t0, label=""):
    r = cp.query("therapy ramp status", "# RAMP ")
    a = cp.query("therapy apap events", "# APAP ")
    print("  %5.0f s %-10s st=%s phase=%s cause=%s onset_s=%s stable=%s sp=%s  "
          "OA=%s Hyp=%s sn=%s" % (
              time.monotonic() - t0, label, field(r, "st"), field(r, "phase"),
              field(r, "cause"), field(r, "onset_s"), field(r, "stable"),
              field(r, "sp"), field(a, "OA"), field(a, "Hyp"), field(a, "sn")))
    return r


def run_autoramp(cp, case):
    jitter = 0 if case == "steady" else 25
    sim_setup(jitter)
    for c in ("therapy edf enable 1", "therapy set epr 0",
              "therapy set pressure %.0f" % TARGET, "therapy set ramp auto",
              "apap set autoramp_max_min %d" % (8 if case == "jitter" else 30)):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP mode %.0f cmH2O, AutoRamp, started" % TARGET)
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic() - 6.0
    plan = []
    if case == "events":
        # flattened (obstructive) 55 % hypopneas, 25 s each, 40 s apart
        for at in (60, 100, 140):
            plan += [(at - 2, "sim set flattening 0.9"),
                     (at, "sim event hypopnea 25 55"),
                     (at + 27, "sim set flattening 0")]
    if case == "snore":
        plan += [(60, "cpap:apap set snore_rms_thresh 0.05")]
    plan.sort()
    onset_at = None
    limit = 360.0 if case != "jitter" else 220.0
    next_print = 10.0
    while time.monotonic() - t0 < limit:
        now = time.monotonic() - t0
        while plan and plan[0][0] <= now:
            _, c = plan.pop(0)
            if c.startswith("cpap:"):
                cp.cmd(c[5:])
            else:
                send(c)
            print("  %5.0f s  -> %s" % (now, c))
        drain_sim(1.0)
        if time.monotonic() - t0 < next_print:
            continue
        next_print += 10.0
        r = ramp_line(cp, t0, case)
        if onset_at is None and field(r, "phase") == "1":
            onset_at = time.monotonic()
        if onset_at is not None and time.monotonic() - onset_at > 100.0:
            break


def apap_line(cp, t0, label):
    a = cp.query("therapy apap events", "# APAP ")
    print("  %5.0f s %-8s lx=%s lx_s=%s lx_n=%s lx_drop=%s  Hyp=%s CA=%s OA=%s "
          "t_ms=%s AHI=%s" % (
              time.monotonic() - t0, label, field(a, "lx"), field(a, "lx_s"),
              field(a, "lx_n"), field(a, "lx_drop"), field(a, "Hyp"), field(a, "CA"),
              field(a, "OA"), field(a, "t_ms"), field(a, "AHI")))


def run_leak(cp):
    print("edf at boot: %s" % cp.query("therapy edf status", "# EDF "))
    sim_setup(0)
    for c in ("therapy set epr 0", "therapy set ramp 0", "therapy set pressure 8"):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP mode 8 cmH2O started")
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic() - 6.0
    plan = [(60, "cpap:leak set high_threshold 5"),
            (220, "cpap:leak set high_threshold 24")]
    for at in (100, 150, 270):
        plan += [(at - 2, "sim set flattening 0.9"),
                 (at, "sim event hypopnea 25 55"),
                 (at + 27, "sim set flattening 0")]
    plan.sort()
    next_print = 10.0
    while time.monotonic() - t0 < 330.0:
        now = time.monotonic() - t0
        while plan and plan[0][0] <= now:
            _, c = plan.pop(0)
            if c.startswith("cpap:"):
                cp.cmd(c[5:])
            else:
                send(c)
            print("  %5.0f s  -> %s" % (now, c))
        drain_sim(1.0)
        if time.monotonic() - t0 >= next_print:
            next_print += 10.0
            apap_line(cp, t0, "leak")


def run_rampstart(cp):
    sim_setup(0)
    for c in ("therapy set epr 0", "therapy set pressure 10", "therapy set ramp 5",
              "therapy set ramp_start 6"):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP 10, fixed 5 min ramp from 6 cmH2O")
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic() - 6.0
    while time.monotonic() - t0 < 90.0:
        drain_sim(10.0)
        ramp_line(cp, t0, "fixed")
    # The simulator's flow loop must not chase a stopped CPAP: static FF
    # first (36 krpm commanded on 2026-10-27 without this).
    send("sim set ff static")
    cp.cmd("therapy set pressure 4")
    drain_sim(8.0)
    cp.cmd("therapy stop")
    drain_sim(6.0)
    for c in ("therapy set pressure 10", "therapy set ramp auto"):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP 10, AutoRamp from 6 cmH2O")
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic() - 6.0
    while time.monotonic() - t0 < 40.0:
        drain_sim(10.0)
        ramp_line(cp, t0, "auto")


def run_csr(cp):
    sim_setup(0)
    for c in ("therapy edf enable 1", "therapy set epr 0", "therapy set ramp 0",
              "therapy set pressure 8"):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP mode 8 cmH2O started")
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic() - 6.0
    phases = [("normal", 300.0, None), ("csr", 480.0, "sim event csr 480"),
              ("normal2", 360.0, "sim event cancel")]
    for label, dur, c in phases:
        if c:
            send(c)
            print("  %5.0f s  -> %s" % (time.monotonic() - t0, c))
        end = time.monotonic() + dur
        while time.monotonic() < end:
            drain_sim(10.0)
            a = cp.query("therapy apap events", "# APAP ")
            print("  %5.0f s %-8s pb=%s pb_s=%s pb_n=%s r=%s depth=%s T=%s  "
                  "CA=%s Hyp=%s OA=%s" % (
                      time.monotonic() - t0, label, field(a, "pb"), field(a, "pb_s"),
                      field(a, "pb_n"), field(a, "pb_r"), field(a, "pb_d"),
                      field(a, "pb_T"), field(a, "CA"), field(a, "Hyp"), field(a, "OA")))


case = CASE
assert case in ("steady", "jitter", "events", "snore", "csr", "leak", "rampstart"), case
cp = None
try:
    cp = Cpap()
    cp.cmd("telem off")
    if case == "csr":
        run_csr(cp)
    elif case == "leak":
        run_leak(cp)
    elif case == "rampstart":
        run_rampstart(cp)
    else:
        run_autoramp(cp, case)
finally:
    try:
        send("sim event cancel")
        send("sim set flattening 0")
        ramp_down()
        send("sim set ff static")
        send("sim set plimit 0")
        send("sim set jitter 0")
    except Exception as e:
        print("sim stop: %s" % e)
    if cp is not None:
        cp.cmd("apap set snore_rms_thresh 0.35")
        cp.cmd("apap set autoramp_max_min 30")
        cp.cmd("leak set high_threshold 24")
        cp.cmd("therapy set ramp_start 4")
        cp.cmd("therapy set pressure 4")
        time.sleep(8.0)
        cp.cmd("therapy stop")
        time.sleep(3.0)
        cp.cmd("therapy set ramp 0")
        print("final: %s" % cp.query("therapy apap events", "# APAP "))
        print("ramp:  %s" % cp.query("therapy ramp status", "# RAMP "))
        print("edf:   %s" % cp.query("therapy edf status", "# EDF "))
        cp.close()
    print("done")
