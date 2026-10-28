#!/usr/bin/env python3
"""#16 / #64 mask pulls with the full telemetry stream recorded.

    python cpap_mask_pull2.py [pressures] [open_s]     default 4,8,12  20

Spoken cues (Windows voice): "Test at P starting", "Get ready to open" ...
"Open the circuit now", "Get ready to reconnect" ... "Reconnect now",
"Done at P"; one pull per pressure. Everything from COM13 is written to
runs/cpap_mask_pull2_<stamp>_raw.log with the host arrival time, plus
"# CUE" lines. The report works in DEVICE time: when the circuit actually
opened / closed (T-line pressure and flow), when the firmware entered /
left mask-off (M lines), and so the real detection delays, independent of
reaction time and telemetry lag.
"""
import sys
import threading
import time

import serial

import subprocess


def speak(text):
    """Spoken cue (Windows built-in voice); returns once it has been said."""
    print("   [voice] " + text, flush=True)
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    "Add-Type -AssemblyName System.Speech; "
                    "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('%s')"
                    % text.replace("'", "")], check=False)

PRESSURES = [float(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "4,8,12").split(",")]
OPEN_S = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
STAMP = time.strftime("%Y%m%d_%H%M%S")
RAW = "runs/cpap_mask_pull2_%s_raw.log" % STAMP
LOG = "runs/cpap_mask_pull2_%s.log" % STAMP


class Rec(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.s = serial.Serial("COM13", 115200, timeout=0.05)
        self.raw = open(RAW, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.t = []      # (host, dev_ms, p_sensor, q_patient, p_mask)
        self.m = []      # (host, dev_ms, event, path, q_total, p_sensor, p_mask)
        self.stop_ev = threading.Event()

    def run(self):
        buf = b""
        while not self.stop_ev.is_set():
            buf += self.s.read(8192)
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                now = time.monotonic()
                ln = ln.decode("ascii", "replace").strip()
                self.raw.write("%.3f %s\n" % (now, ln))
                f = ln.split(",")
                try:
                    if f[0] == "T" and len(f) >= 7:
                        with self.lock:
                            self.t.append((now, int(f[1]), float(f[2]), float(f[3]), float(f[6])))
                    elif f[0] == "M" and len(f) >= 8:
                        with self.lock:
                            self.m.append((now, int(f[1]), int(f[2]), int(f[3]), float(f[4]),
                                           float(f[6]), float(f[7])))
                except ValueError:
                    pass

    def cmd(self, c):
        self.raw.write("%.3f # > %s\n" % (time.monotonic(), c))
        self.s.write((c + "\r\n").encode("ascii"))
        time.sleep(0.2)

    def cue(self, what):
        self.raw.write("%.3f # CUE %s\n" % (time.monotonic(), what))
        return time.monotonic()


def say(log, *a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    log.write(s + "\n")
    log.flush()


def analyse(log, r, p, segs):
    with r.lock:
        t = list(r.t)
        m = list(r.m)
    if len(t) < 100:
        say(log, "   no T lines")
        return
    offs = sorted(h - d / 1000.0 for h, d, *_ in t)
    off = offs[len(offs) // 50]                 # least-delayed arrivals
    for (c_open, c_close, t_end) in segs:
        d_open = (c_open - off) * 1000.0
        d_close = (c_close - off) * 1000.0
        d_end = (t_end - off) * 1000.0
        win = [x for x in t if d_open - 3000 <= x[1] <= d_end]
        if not win:
            continue
        pre = [x for x in win if x[1] < d_open]
        base_p = sorted(x[2] for x in pre)[len(pre) // 2] if pre else p
        # actual open: first sensor P below base - max(1, 0.25*base) after the cue
        thr = base_p - max(1.0, 0.25 * base_p)
        act_open = next((x[1] for x in win if x[1] >= d_open - 500 and x[2] < thr), None)
        # actual close: after the reconnect cue, sensor P back over 80 % of base
        act_close = next((x[1] for x in win if x[1] >= d_close - 500 and x[2] > 0.8 * base_p), None)
        ent = [x for x in m if x[2] == 1 and d_open - 3000 <= x[1] <= d_end]
        ext = [x for x in m if x[2] == 0 and d_open - 3000 <= x[1] <= d_end]
        say(log, "   base P %.2f; open cue -> P drop %s ms; reconnect cue -> P back %s ms" % (
            base_p, None if act_open is None else int(act_open - d_open),
            None if act_close is None else int(act_close - d_close)))
        for x in ent:
            say(log, "   M enter path %d at %s ms after the P drop (q %.0f, Ps %.2f, Pm %.2f)" % (
                x[3], None if act_open is None else int(x[1] - act_open), x[4], x[5], x[6]))
        for x in ext:
            say(log, "   M exit  path %d at %s ms after P came back (q %.0f, Ps %.2f, Pm %.2f)" % (
                x[3], None if act_close is None else int(x[1] - act_close), x[4], x[5], x[6]))
        if not ent:
            pmin = min((x[2] for x in win if x[1] >= d_open), default=None)
            say(log, "   NO mask-off entry; sensor P min after the cue %s" % pmin)
        # pressure / flow while open (median over the open window)
        op = [x for x in win if act_open and act_open + 1000 <= x[1] <= d_close]
        if op:
            say(log, "   while open: Ps med %.2f, Pmask med %.2f, Qpat med %.1f" % (
                sorted(x[2] for x in op)[len(op) // 2], sorted(x[4] for x in op)[len(op) // 2],
                sorted(x[3] for x in op)[len(op) // 2]))


def main():
    r = Rec()
    r.start()
    with open(LOG, "w", encoding="utf-8") as log:
        try:
            r.cmd("telem on")
            r.cmd("blower set telem_terms 0")
            for p in PRESSURES:
                for x in ("therapy set mode cpap", "therapy set ramp 0", "therapy set epr 0",
                          "therapy set pressure %.1f" % p):
                    r.cmd(x)
                speak("Test at %d starting. Keep the circuit connected." % round(p))
                r.cmd("therapy start")
                say(log, "\n== %g cmH2O: settling 25 s" % p)
                time.sleep(20.0)
                speak("Get ready to open.")
                time.sleep(3.0)
                speak("Open the circuit now.")
                c_open = r.cue("open %g" % p)
                say(log, "   OPEN cue")
                time.sleep(max(0.0, OPEN_S - 5.0))
                speak("Get ready to reconnect.")
                time.sleep(3.0)
                speak("Reconnect now.")
                c_close = r.cue("close %g" % p)
                say(log, "   RECONNECT cue")
                time.sleep(30.0)
                t_end = time.monotonic()
                r.cmd("therapy set pressure 4")
                time.sleep(6.0)
                r.cmd("therapy stop")
                time.sleep(4.0)
                speak("Done at %d. Keep the circuit connected." % round(p))
                analyse(log, r, p, [(c_open, c_close, t_end)])
            r.cmd("therapy mask")
            time.sleep(0.5)
        finally:
            r.cmd("therapy set pressure 4")
            time.sleep(3.0)
            r.cmd("therapy stop")
            time.sleep(1.0)
            r.cmd("telem off")
            r.stop_ev.set()
            time.sleep(0.3)
            r.raw.close()
            speak("All tests finished.")


if __name__ == "__main__":
    main()
