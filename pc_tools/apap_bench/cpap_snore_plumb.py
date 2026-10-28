#!/usr/bin/env python3
"""Snore-index plumbing check (#60) on our CPAP - not a snore detection test.

Checks that normal breathing does NOT score snore at the default threshold,
then that a snore level reaches `therapy apap events` (snl= sn_s=), the PLD
Snore.2s channel and SUMMARY.CSV snore_s - either by lowering the CPAP's
threshold to its 0.05 floor, or (SIM_SNORE=1) by the simulator's inspiratory
blower modulation at 35 Hz, stepped 500 / 1000 / 1500 rpm.

Bench 2026-10-27: the 35 Hz modulation never reached the CPAP's snore RMS
(floor ~0.20 slm with or without it), and +/-1500 rpm scored as flow
limitation instead. Use the lowered threshold for the plumbing check.

Rig: CPAP (COM13) -> circuit -> breath simulator (COM15) -> 2 L lung.
CPAP mode 8 cmH2O, EPR off: score-only, so snore cannot move pressure.
snr= is one 200 ms RMS sample per poll (~1 s), taken in any breath phase;
each line gives the median / max of the samples in that 10 s.
Stop order as the harnesses: simulator ramp-down first, then the CPAP via
4 cmH2O and `therapy stop`.
"""
import os
import threading
import time

import serial

from phase1 import send, ramp_down, ser

CPAP_PORT = "COM13"
SIM_GUARD = 36000
THR_LOW = 0.05
THR_DEFAULT = 0.35
HP_LOW = float(os.environ.get("SNORE_HP_LOW", "20"))   # detector HP corner while lowered
HP_DEFAULT = 20.0
SIM_SNORE = os.environ.get("SIM_SNORE", "0") == "1"
SNORE_HZ = 35
SNORE_LADDER = (500, 1000, 1500)                     # sim rpm, peak


class Cpap:
    def __init__(self):
        self.s = serial.Serial(CPAP_PORT, 115200, timeout=0.1)
        self.lock = threading.Lock()
        self.ev = None
        self.rpm = None
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
                if t.startswith("# APAP "):
                    with self.lock:
                        self.ev = t
                elif t.startswith("B,"):
                    p = t.split(",")
                    try:
                        with self.lock:
                            self.rpm = int(p[4])
                    except (ValueError, IndexError):
                        pass

    def cmd(self, c):
        self.s.write((c + "\r\n").encode("ascii"))
        time.sleep(0.15)

    def events(self):
        with self.lock:
            self.ev = None
        self.cmd("therapy apap events")
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            with self.lock:
                if self.ev:
                    return self.ev
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


def field(ev, key):
    for tok in (ev or "").split():
        if tok.startswith(key + "="):
            return tok.split("=", 1)[1]
    return "?"


def stats(v):
    if not v:
        return "  -  /  -  "
    v = sorted(v)
    return "%.3f/%.3f" % (v[len(v) // 2], v[-1])


def watch(cp, t0, seconds, every, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        win = time.monotonic() + every
        snr = []
        ev = None
        while time.monotonic() < win:
            drain_sim(0.5)
            ev = cp.events() or ev
            try:
                snr.append(float(field(ev, "snr")))
            except ValueError:
                pass
        print("  %5.0f s  %-12s sn=%s snl=%s sn_s=%s Snore=%s fli=%s snr med/max=%s" % (
            time.monotonic() - t0, label, field(ev, "sn"), field(ev, "snl"),
            field(ev, "sn_s"), field(ev, "Snore"), field(ev, "fli"), stats(snr)))


cp = None
try:
    cp = Cpap()
    cp.cmd("telem off")
    for c in ("sim set waveform sine", "sim set timing rate_ie",
              "sim set rate 12", "sim set ie 1", "sim set insp_pause 0",
              "sim set exp_pause 0", "sim set jitter 0",
              "sim set flattening 0", "sim set fl_shape plateau",
              "sim set exp_tau 0.4", "sim set rpm_base 8000",
              "sim set tidal 500", "sim set recovery 0", "sim set snore_hz 0",
              "sim set plimit 40", "sim set control flow",
              "sim set ff static", "sim script clear"):
        send(c)
    send("telem on")
    send("sim start")
    print("simulator breathing (static FF) - priming 20 s")
    drain_sim(20.0)

    for c in ("therapy edf enable 1",           # BRP/PLD/EVE: off after each boot
              "therapy set ramp 0", "therapy set epr 0",
              "therapy set pressure 8",          # CPAP mode, 8 cmH2O
              "apap set snore_rms_thresh %.2f" % THR_DEFAULT):
        cp.cmd(c)
    cp.cmd("therapy start")
    print("CPAP mode 8 cmH2O started")
    drain_sim(6.0)
    send("sim set ff pressure")
    t0 = time.monotonic()
    watch(cp, t0, 60.0, 10.0, "normal")
    if SIM_SNORE:
        send("sim set snore_hz %d" % SNORE_HZ)
        for rpm in SNORE_LADDER:
            send("sim set snore_rpm %d" % rpm)
            print("sim snore %d Hz +/-%d rpm" % (SNORE_HZ, rpm))
            watch(cp, t0, 40.0, 10.0, "sim %d" % rpm)
        send("sim set snore_rpm 0")
        send("sim set snore_hz 0")
        print("sim snore off")
    else:
        cp.cmd("apap set snore_rms_thresh %.2f" % THR_LOW)
        cp.cmd("apap set snore_hp_hz %.1f" % HP_LOW)
        print("snore_rms_thresh -> %.2f, snore_hp_hz -> %.1f" % (THR_LOW, HP_LOW))
        watch(cp, t0, 90.0, 10.0, "thr %.2f" % THR_LOW)
        cp.cmd("apap set snore_rms_thresh %.2f" % THR_DEFAULT)
        cp.cmd("apap set snore_hp_hz %.1f" % HP_DEFAULT)
        print("snore_rms_thresh -> %.2f, snore_hp_hz -> %.1f" % (THR_DEFAULT, HP_DEFAULT))
    watch(cp, t0, 40.0, 10.0, "restored")
finally:
    try:
        send("sim set snore_rpm 0")
        send("sim set snore_hz 0")
        ramp_down()
        send("sim set ff static")
        send("sim set plimit 0")
    except Exception as e:
        print("sim stop: %s" % e)
    if cp is not None:
        cp.cmd("apap set snore_rms_thresh %.2f" % THR_DEFAULT)
        cp.cmd("apap set snore_hp_hz %.1f" % HP_DEFAULT)
        cp.cmd("therapy set pressure 4")
        time.sleep(8.0)
        cp.cmd("therapy stop")
        time.sleep(3.0)
        ev = cp.events()
        print("final: %s" % ev)
        cp.close()
    print("done")
