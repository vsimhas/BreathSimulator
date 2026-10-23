#!/usr/bin/env python3
"""First AutoSet trial: what does the CPAP's APAP actually detect?

Rig: CPAP (COM13) -> circuit -> collapsible chamber (squeeze blower OFF)
-> breath simulator (COM15) -> 2 L lung.

CPAP: AutoSet 8-12 cmH2O (range kept near the validated 10 because the
0.8 A supply-current guard stays), EPR off, ramp 0. Original APAP range and
mode restored afterwards, motor line switched back off.

CPAP firmware: CPAP_InitialCode working tree on top of 4ab7a0c (uncommitted),
"R4/R5": R4 one score per low-flow event (+ ca_reclass_window_s 30), R5a APAP
breath qualification (>= 400 ms, >= 80 mL), R5b leak adaptation frozen in
NO_BREATH/FOT. R6 (torn flow-sensor reads every ~17.75 s) is NOT fixed, so the
needles are still there - this run tests whether the APAP now ignores them.
Probe: `apap set breath_min_insp_ms 400` (its default) must be acked.

Simulator: pressure FF, 12 bpm I:E 1:1, 500 mL. Recovery OFF except before the
last apnea. Script = trial 6 with the R5 apnea block inserted:
  none 90 | apnea 5 | none 45 | 3 x (apnea 10 | none 45) |
  3 x (apnea 20 | none 60) | hyp 50% 40 | none 60 | hyp 55% 40 | none 60 |
  hyp 60% 40 | none 60 | [recovery 2 x 1.5] apnea 20 | none 90     (955 s)
Logged: CPAP T (50 Hz patient flow), PH, status every ~1 s.

START ORDER (2026-10-20): simulator breathing first (static FF), then
the CPAP, then pressure FF - no start-up CA expected.

FLATTENED-BREATH RUN ON OUR CPAP (2026-10-20): the AirSense 11 'flshape'
script - normal 90 | plateau 90% 120 | normal 120 | M 90% 120 | normal 120 |
plateau 60% 120 | normal 120 (810 s), full 500 mL breaths throughout.

AirSense on the same script: plateau 90% FL 0.28/0.42, P 8 -> 12 in ~75 s;
M 90% FL 0.45/0.60 (strongest), P held 12; plateau 60% FL <= 0.09, P down.

PREDICTIONS (from our rules: FL = flatness > 0.78 or peak < 0.70 x base):
  plateau 90 %  flat 0.82 -> FL, pressure raised
  M 90 %        flat 0.73, peak 80 % -> MISSED: no FL, no raise
  plateau 60 %  flat 0.75 -> missed
  no Hyp, no CA, no FOT

Guards: FOT watchdog (APAP oscillating continuously > 60 s - the current
guard cannot see FOT's spiky current), CPAP supply current > 0.8 A for
> 0.7 s, Vbus < 21 V, drive fault,
mask-off; simulator 36000 rpm and sensor liveness. Any trip -> verified
simulator ramp-down + CPAP setpoint 4 -> stop from < 14.5k rpm.
"""
import csv
import re
import statistics
import sys
import threading
import time

import serial

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
sys.path.insert(0, HERE)
from phase1 import send, collect, ramp_down, ser, decode
import liveness

CPAP_PORT = "COM13"
APAP_MIN, APAP_MAX = 8.0, 12.0
SIM_GUARD = 36000
I_LIMIT, I_HOLD_S, VBUS_MIN = 0.8, 0.7, 21.0
FOT_MAX_S = 60.0   # watchdog: continuous APAP oscillation longer than this aborts

SCRIPT = [("none", 90, None),
          ("flowlimit", 120, 90), ("none", 120, None),
          ("flowlimit", 120, 90), ("none", 120, None),
          ("flowlimit", 120, 60), ("none", 120, None)]
FL_SHAPES = ["plateau", "m", "plateau"]
# fl_shape is a global simulator setting: switch it 60 s into the normal gap
# before each flowlimit step (same timing as airsense_run.py flshape).
SHAPE_ACTIONS = []
_acc, _sh = 0.0, list(FL_SHAPES)
for _ev, _dur, _sev in SCRIPT:
    if _ev == "flowlimit" and _sh:
        SHAPE_ACTIONS.append((max(0.0, _acc - 60.0), _sh.pop(0)))
    _acc += _dur
# Recovery breaths go on just before the last apnea only, so they cannot
# contaminate the other tests. Applied at a breath boundary.
REC_ON_T = float("inf")          # no recovery breaths in this run
SCRIPT_S = sum(d for _, d, _ in SCRIPT)

APAP_RE = re.compile(
    r"# APAP OA=(\d+) CA=(\d+) Hyp=(\d+) FL=(\d+) Snore=(\d+) t_ms=(\d+) "
    r"AHI=([-\d.]+) RDI=([-\d.]+) SI=([-\d.]+) st=(\S+) hyp=(\d) fl=(\d) "
    r"sn=(\d) leak_blk=(\d) pol=(-?\d+) pol_ev=([-\d.]+) pol_bad=(\d+)"
    r"(?: flat=([-\d.]+) dip=([-\d.]+))?")      # R13 build: last breath's shape


class Abort(Exception):
    pass


class Cpap:
    """Background reader: B (guard), S (status) and # APAP (events) lines."""

    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2, write_timeout=2.0)
        self.lock = threading.Lock()
        self.b = self.st = self.ap = None
        self.st_log, self.ap_log, self.b_log = [], [], []
        self.t_log, self.ph_log = [], []
        self.diag_log = []   # (host, 'apap'|'epr', {key: value})
        self.fw_reply = 0
        self.abort = None
        self.i_over = None
        self.i_max = 0.0
        self.osc_since = None     # host time the current FOT run started
        self.osc_max_s = 0.0      # longest continuous FOT run seen
        self.t0 = time.monotonic()
        self.run = True
        threading.Thread(target=self._reader, daemon=True).start()

    def cmd(self, c):
        self.s.write((c + "\r\n").encode("ascii"))
        self.s.flush()
        time.sleep(0.15)

    def _reader(self):
        while self.run:
            try:
                raw = self.s.readline()
            except Exception:
                continue
            if not raw:
                continue
            t = raw.decode("ascii", "replace").strip()
            now = time.monotonic()
            p = t.split(",")
            try:
                if t.startswith("B,") and len(p) >= 22:
                    vbus = int(p[2]) / 1000.0
                    amps = ((1.5 * (int(p[19]) * int(p[3]) +
                                    int(p[20]) * int(p[5])) / 1e6) / vbus
                            if vbus > 1.0 else 0.0)
                    b = dict(vbus=vbus, rpm=int(p[6]), faults=int(p[9]),
                             amps=amps)
                    with self.lock:
                        self.b = b
                        self.b_log.append((now - self.t0, amps, b["rpm"]))
                        self.i_max = max(self.i_max, amps)
                        if amps > I_LIMIT:
                            if self.i_over is None:
                                self.i_over = now
                            elif now - self.i_over > I_HOLD_S and not self.abort:
                                self.abort = "CPAP current %.2f A > %.1f A" % (amps, I_LIMIT)
                        else:
                            self.i_over = None
                        if b["faults"] and not self.abort:
                            self.abort = "CPAP drive fault 0x%04X" % b["faults"]
                        if b["rpm"] > 1000 and vbus < VBUS_MIN and not self.abort:
                            self.abort = "CPAP Vbus %.1f V" % vbus
                elif t.startswith("# apap diag ") or t.startswith("# epr diag "):
                    kv = dict(x.split("=", 1) for x in t.split()[3:] if "=" in x)
                    with self.lock:
                        self.diag_log.append((now, t.split()[1], kv))
                elif t.startswith("T,") and len(p) >= 7:
                    row = (now - self.t0, int(p[1]), float(p[2]), float(p[3]),
                           int(p[4]), int(p[5]), float(p[6]))
                    with self.lock:
                        self.t_log.append(row)
                elif t.startswith("PH,") and len(p) >= 9:
                    row = (now - self.t0, int(p[1]), float(p[2]), int(p[6]),
                           float(p[7]), float(p[8]))
                    with self.lock:
                        self.ph_log.append(row)
                elif t.startswith("S,") and len(p) >= 21:
                    st = dict(t=now - self.t0, state=int(p[2]),
                              target=float(p[3]), meas=float(p[5]),
                              leak=float(p[6]), maskoff=int(p[8]),
                              mode=int(p[10]), amin=float(p[11]),
                              amax=float(p[12]), apst=int(p[13]),
                              osc=int(p[14]), fot_p=float(p[15]),
                              fot_q=float(p[16]), osa=int(p[18]))
                    with self.lock:
                        self.st = st
                        self.st_log.append(st)
                        # FOT watchdog: the current guard cannot see FOT's spiky
                        # current, so bound how long the APAP may oscillate.
                        if st["state"] == 2 and st["osc"]:
                            if self.osc_since is None:
                                self.osc_since = now
                            run_s = now - self.osc_since
                            self.osc_max_s = max(self.osc_max_s, run_s)
                            if run_s > FOT_MAX_S and not self.abort:
                                self.abort = ("APAP oscillating continuously for %.0f s "
                                              "(> %.0f s FOT watchdog)" % (run_s, FOT_MAX_S))
                        else:
                            self.osc_since = None
                        if st["state"] and st["maskoff"] and not self.abort:
                            self.abort = "CPAP reports mask-off"
                elif "apap set hyp_rearm_ratio" in t or "apap set fl_dip_ratio" in t or (
                        t.startswith("# err") and getattr(self, "fw_reply", 0) is None):
                    with self.lock:
                        self.fw_reply = t
                else:
                    m = APAP_RE.search(t)
                    if m:
                        g = m.groups()
                        ap = dict(t=now - self.t0, tms=int(g[5]), OA=int(g[0]), CA=int(g[1]),
                                  Hyp=int(g[2]), FL=int(g[3]), Snore=int(g[4]),
                                  AHI=float(g[6]), st=g[9], hyp=int(g[10]),
                                  fl=int(g[11]), sn=int(g[12]),
                                  leak_blk=int(g[13]), pol=int(g[14]),
                                  pol_bad=int(g[16]),
                                  flat=float(g[17]) if g[17] else None,
                                  dip=float(g[18]) if g[18] else None)
                        with self.lock:
                            self.ap = ap
                            self.ap_log.append(ap)
            except (ValueError, IndexError):
                pass

    def status(self, wait=3.0):
        with self.lock:
            self.st = None
        self.cmd("therapy status")
        end = time.monotonic() + wait
        while time.monotonic() < end:
            with self.lock:
                if self.st is not None:
                    return dict(self.st)
            time.sleep(0.05)
        return None

    def close(self):
        self.run = False
        time.sleep(0.3)
        self.s.close()


def cpap_safe_stop(cp):
    print("\n  [CPAP stop] setpoint -> 4 cmH2O")
    cp.cmd("therapy set pressure 4")          # also switches to CPAP mode
    end = time.monotonic() + 12.0
    while time.monotonic() < end:
        with cp.lock:
            b = cp.b
        if b and b["rpm"] < 14500:
            break
        time.sleep(0.25)
    time.sleep(1.0)
    with cp.lock:
        rpm = cp.b["rpm"] if cp.b else None
    print("  [CPAP stop] blower at %s rpm - therapy stop" % rpm)
    cp.cmd("therapy stop")
    end = time.monotonic() + 12.0
    st = None
    while time.monotonic() < end:
        st = cp.status()
        if st and st["state"] == 0:
            break
        time.sleep(0.5)
    print("  [CPAP stop] state %s" % (st["state"] if st else "?"))


cp = None
orig = None
sim_rows, sim_events = [], []
try:
    send("log 0")
    send("telem on")
    send("sim stop")
    send("flow on")
    send("press on")
    send("telem off")                        # quiet while the CPAP is set up

    cp = Cpap(CPAP_PORT)
    cp.cmd("telem on")
    cp.cmd("blower set telem_motor 1")
    time.sleep(3.0)

    print("=" * 78)
    print("PRE-CHECKS")
    print("=" * 78)
    st = cp.status()
    with cp.lock:
        b = cp.b
    if st is None:
        raise SystemExit("no CPAP status reply")
    orig = dict(mode=st["mode"], amin=st["amin"], amax=st["amax"],
                target=st["target"])
    print("  CPAP state %d mode %d measured %.2f  APAP range %.1f-%.1f   %s"
          % (st["state"], st["mode"], st["meas"], st["amin"], st["amax"],
             ("Vbus %.2f V" % b["vbus"]) if b else "NO B LINE"))
    if st["state"] != 0:
        raise SystemExit("CPAP not idle")
    if abs(st["meas"]) > 1.0:
        raise SystemExit("CPAP pressure %.2f at rest - stuck sensor?" % st["meas"])
    if b is None or not (22.0 <= b["vbus"] <= 26.0):
        raise SystemExit("CPAP B line missing or Vbus out of range")
    if not liveness.check(ser, "before"):
        raise SystemExit("simulator sensors not reading")

    # Is the CPAP running the R4/R5 build? Only it knows this name; sending
    # its own default value (400) changes nothing.
    with cp.lock:
        cp.fw_reply = None
    cp.cmd("apap set hyp_rearm_ratio 0.7")
    end = time.monotonic() + 4.0
    while time.monotonic() < end and cp.fw_reply is None:
        time.sleep(0.1)
    print("  CPAP firmware probe: %s" % (cp.fw_reply or "no reply"))
    if not (cp.fw_reply and cp.fw_reply.startswith("# ack")):
        raise SystemExit("CPAP is not running the R12 build")
    # R13 (2026-10-23): whole-inspiration flatness + dip; only that build
    # knows fl_dip_ratio, and 0.93 is its default (0.90 on the first R13 flash,
    # so this also sets the tuned value on that build).
    with cp.lock:
        cp.fw_reply = None
    cp.cmd("apap set fl_dip_ratio 0.93")
    end = time.monotonic() + 4.0
    while time.monotonic() < end and cp.fw_reply is None:
        time.sleep(0.1)
    print("  CPAP R13 probe: %s" % (cp.fw_reply or "no reply"))
    if not (cp.fw_reply and cp.fw_reply.startswith("# ack")):
        raise SystemExit("CPAP is not running the R13 build")

    # ---- start order: simulator FIRST, then the CPAP -----------------------
    # Every trial up to 7g started the CPAP first, so the APAP saw no breath
    # for its first ~10 s and scored a start-up central apnea (harness
    # artefact). Now the simulator is already breathing when therapy starts.
    # It breathes on the STATIC feed-forward through the CPAP start: in
    # pressure mode its inlet estimate would still be from 0 cmH2O when the
    # CPAP comes up to 8, and the model would over-command the first breaths.
    # The estimate keeps updating in static mode, so it is current at 8 cmH2O
    # by the time the pressure model is switched on.
    def drain_sim(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cp.abort:
                raise Abort(cp.abort)
            raw = ser.readline()
            if not raw:
                continue
            t = raw.decode("ascii", "replace").strip()
            if t.startswith("B,"):
                p = t.split(",")
                try:
                    if len(p) >= 8 and int(p[6]) > SIM_GUARD:
                        raise Abort("simulator commanded %s rpm" % p[6])
                except ValueError:
                    pass

    for c in ("sim set waveform sine", "sim set timing rate_ie",
              "sim set rate 12", "sim set ie 1", "sim set insp_pause 0",
              "sim set exp_pause 0", "sim set jitter 0",
              "sim set flattening 0", "sim set fl_shape plateau",
              "sim set exp_tau 0.4", "sim set rpm_base 8000",
              "sim set tidal 500", "sim set recovery 0", "sim set snore_hz 0",
              "sim set plimit 40", "sim set control flow",
              "sim set ff static", "sim script clear"):
        send(c)
    for ev, dur, sev in SCRIPT:
        send("sim script add %s %d%s" % (ev, dur, "" if sev is None else " %d" % sev))
    send("telem on")
    send("sim start")
    print("\n  simulator breathing (static FF, CPAP off) - priming 20 s...")
    drain_sim(20.0)

    # ---- CPAP: AutoSet, started on a breathing patient ---------------------
    for c in ("therapy set ramp 0", "therapy set epr 0",
              "therapy set pressure_min %.0f" % APAP_MIN,
              "therapy set pressure_max %.0f" % APAP_MAX,
              "therapy set mode autoset"):
        cp.cmd(c)
    cp.cmd("therapy start")
    end = time.monotonic() + 25.0
    st = None
    while time.monotonic() < end:
        st = cp.status()
        if st and st["state"] == 2 and st["mode"] == 1:
            break
        drain_sim(1.0)
    if not (st and st["state"] == 2 and st["mode"] == 1):
        raise Abort("CPAP did not reach running AutoSet (state %s mode %s)"
                    % (st and st["state"], st and st["mode"]))
    print("  CPAP running AutoSet %.0f-%.0f, target %.2f, measured %.2f"
          % (st["amin"], st["amax"], st["target"], st["meas"]))

    drain_sim(6.0)                    # a breath or two at 8 cmH2O for the estimate
    send("sim set ff pressure")
    print("  simulator -> pressure FF; settling 30 s (past the APAP's 15 s start-up grace)")
    drain_sim(30.0)

    cp.cmd("therapy apap events")
    cp.cmd("apap diag")
    drain_sim(2.5)
    st = cp.status()
    with cp.lock:
        ap0 = cp.ap
    if ap0:
        print("  APAP at script start: state %s | OA %d CA %d Hyp %d FL %d Snore %d   "
              "polarity %d (bad %d)%s"
              % (st and st["apst"], ap0["OA"], ap0["CA"], ap0["Hyp"], ap0["FL"],
                 ap0["Snore"], ap0["pol"], ap0["pol_bad"],
                 "" if (ap0["OA"] + ap0["CA"] + ap0["Hyp"]) == 0
                 else "   !! events scored during start-up"))

    print("  script start: %s  (%d s)\n" % (
        " | ".join("%s %d" % (e, d) for e, d, _ in SCRIPT), SCRIPT_S))
    send("sim script start")
    t_script = time.monotonic()
    t_stop = t_script + SCRIPT_S + 25.0
    next_poll, poll_i, next_note = t_script, 0, t_script + 30.0
    rec_sent = False
    while time.monotonic() < t_stop:
        if cp.abort:
            raise Abort(cp.abort)
        now = time.monotonic()
        while SHAPE_ACTIONS and now - t_script >= SHAPE_ACTIONS[0][0]:
            _, _shape = SHAPE_ACTIONS.pop(0)
            send("sim set fl_shape %s" % _shape)
            print("  t=%3.0fs  SIM  fl_shape -> %s" % (now - t_script, _shape))
        if (not rec_sent) and (now - t_script >= REC_ON_T):
            send("sim set recovery 2")
            send("sim set recovery_gain 1.5")
            rec_sent = True
            print("  t=%3.0fs  SIM  recovery breaths ON (2 x 1.5) for the last apnea"
                  % (now - t_script))
        if now >= next_poll:
            cp.cmd(("therapy status", "therapy status", "therapy status",
                    "therapy apap events", "apap diag")[poll_i % 5])
            poll_i += 1
            next_poll = now + 1.0
        if now >= next_note:
            with cp.lock:
                s_, a_ = cp.st, cp.ap
            print("  t=%3.0fs  CPAP target %s meas %s apap_st %s osc %s | "
                  "OA %s CA %s Hyp %s FL %s | fl %s hyp %s"
                  % (now - t_script,
                     s_ and "%.1f" % s_["target"], s_ and "%.1f" % s_["meas"],
                     s_ and s_["apst"], s_ and s_["osc"],
                     a_ and a_["OA"], a_ and a_["CA"], a_ and a_["Hyp"],
                     a_ and a_["FL"], a_ and a_["fl"], a_ and a_["hyp"]))
            next_note = now + 30.0
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", "replace").strip()
        p = t.split(",")
        if t.startswith("B,") and len(p) >= 8:
            try:
                if int(p[6]) > SIM_GUARD:
                    raise Abort("simulator commanded %s rpm" % p[6])
            except ValueError:
                pass
        elif t.startswith("E,") and len(p) >= 6:
            ev = dict(t=time.monotonic() - t_script, breath=int(p[2]),
                      kind=p[3], event=p[4], sev=p[5])
            sim_events.append(ev)
            print("  t=%3.0fs  SIM  E %-5s %-9s (breath %d)"
                  % (ev["t"], ev["kind"], ev["event"], ev["breath"]))
        elif t.startswith("V,") and len(p) >= 13:
            try:
                sim_rows.append(dict(t=time.monotonic() - t_script,
                                     breath=int(p[1]), vt=float(p[2]),
                                     qpk=float(p[9]), flat=float(p[11]),
                                     event=p[12]))
            except ValueError:
                pass

    ramp_down()
    alive = liveness.check(ser, "after")
    import pickle
    with cp.lock:
        raw_dump = dict(t_log=list(cp.t_log), ph_log=list(cp.ph_log),
                        st_log=list(cp.st_log), ap_log=list(cp.ap_log),
                        b_log=list(cp.b_log), t0=cp.t0)
    raw_dump.update(t_script=t_script, sim_events=sim_events, sim_rows=sim_rows,
                    script=SCRIPT, fl_shapes=FL_SHAPES)
    with open(SP + "/apap_flshape_raw.pkl", "wb") as fh:
        pickle.dump(raw_dump, fh)
    print("  raw capture saved: apap_flshape_raw.pkl (%d T lines, %d PH lines)"
          % (len(raw_dump["t_log"]), len(raw_dump["ph_log"])))

    # ---- analysis -------------------------------------------------------------
    with cp.lock:
        aps = list(cp.ap_log)
        sts = list(cp.st_log)
    t_off = t_script - cp.t0                 # CPAP log times are from cp.t0
    for a in aps:
        a["ts"] = a["t"] - t_off
    for s_ in sts:
        s_["ts"] = s_["t"] - t_off

    print("\n" + "=" * 78)
    print("SIMULATOR - what was delivered, by event label (V lines)")
    print("=" * 78)
    by = {}
    for r in sim_rows:
        by.setdefault(r["event"], []).append(r)
    for ev, rows in by.items():
        print("  %-10s %3d breaths  Vt %5.0f  peak %5.1f  flat %.3f"
              % (ev, len(rows), statistics.median(r["vt"] for r in rows),
                 statistics.median(r["qpk"] for r in rows),
                 statistics.median(r["flat"] for r in rows)))

    # Merged timeline: the moment each CPAP counter rose and each flag
    # changed, next to the simulator's markers. Fixed windows misattributed
    # an event across a segment boundary last run, so attribute nothing -
    # just show the times. CPAP lines arrive ~2 s late.
    plan, t = [], 0.0
    for ev, dur, sev in SCRIPT:
        plan.append((t, t + dur, ev if sev is None else "%s %d%%" % (ev, sev)))
        t += dur

    def seg(ts):
        for a0, z0, name in plan:
            if a0 <= ts < z0:
                return name
        return "after script"

    tl = []
    for e in sim_events:
        tl.append((e["t"], "SIM ", "E %s %s" % (e["kind"], e["event"])))
    prev = None
    for a in aps:
        if prev is not None:
            for k in ("OA", "CA", "Hyp", "FL", "Snore"):
                if a[k] > prev[k]:
                    tl.append((a["ts"], "CPAP", "%s count %d -> %d"
                               % (k, prev[k], a[k])))
            for k, nm in (("fl", "FL flag"), ("hyp", "Hyp flag"),
                          ("sn", "snore flag")):
                if a[k] != prev[k]:
                    tl.append((a["ts"], "CPAP", "%s %s" % (nm, "ON" if a[k] else "off")))
        prev = a
    prevs = None
    for s_ in sts:
        if prevs is not None and s_["osc"] != prevs["osc"]:
            tl.append((s_["ts"], "CPAP", "oscillation %s" % ("ON" if s_["osc"] else "off")))
        prevs = s_
    tl.sort(key=lambda x: x[0])

    print("\n" + "=" * 78)
    print("TIMELINE  (host seconds from script start; CPAP lines ~2 s late)")
    print("=" * 78)
    for a0, z0, name in plan:
        print("  -- %4.0f-%-4.0f %s" % (a0, z0, name))
        for ts, who, what in tl:
            if a0 <= ts < z0:
                print("     %6.1f  %s  %s" % (ts, who, what))
    for ts, who, what in tl:
        if ts >= plan[-1][1] or ts < 0:
            print("     %6.1f  %s  %s   (%s)" % (ts, who, what, seg(ts)))

    tgt = [(x["ts"], x["target"]) for x in sts if x["ts"] >= 0]
    if tgt:
        print("\n  CPAP target %.1f -> %.1f cmH2O (max %.1f)"
              % (tgt[0][1], tgt[-1][1], max(v for _, v in tgt)))
    # ---- per-depth delivery ------------------------------------------------
    norm_pk = [r["qpk"] for r in sim_rows if r["event"] == "None"]
    base_pk = statistics.median(norm_pk) if norm_pk else float("nan")
    print("\n  hypopnea delivery (normal peak %.1f slm):" % base_pk)
    for a0, z0, name in plan:
        if not name.startswith("hypopnea"):
            continue
        sel = [r for r in sim_rows
               if r["event"] == "Hypopnea" and a0 <= r["t"] <= z0 + 6.0]
        if sel:
            pk = [r["qpk"] for r in sel]
            print("    %-14s Vt %4.0f mL  peak %5.1f slm (%3.0f%%)  breaths %.1f-%.1f"
                  % (name, statistics.median(r["vt"] for r in sel),
                     statistics.median(pk),
                     100 * statistics.median(pk) / base_pk, min(pk), max(pk)))

    # ---- per-apnea: FOT onset, duration, scored? ----------------------------
    def count_at(key, ts):
        v = None
        for a in aps:
            if a["ts"] <= ts:
                v = a[key]
        return v

    osc_on, on0 = [], None
    for s_ in sts:
        if s_["osc"] and on0 is None:
            on0 = s_["ts"]
        elif not s_["osc"] and on0 is not None:
            osc_on.append((on0, s_["ts"]))
            on0 = None
    if on0 is not None:
        osc_on.append((on0, sts[-1]["ts"]))

    print("\n  apneas (oscillation times from S lines: ~4 s resolution, ~2 s late):")
    print("    %-12s %10s %10s %10s  %s" % ("segment", "FOT onset", "FOT for",
                                           "CA/OA +", "scored?"))
    for a0, z0, name in plan:
        if not name.startswith("apnea"):
            continue
        starts = [e["t"] for e in sim_events
                  if e["kind"] == "START" and e["event"] == "Apnea"
                  and a0 - 2.0 <= e["t"] <= z0]
        t_on = starts[0] if starts else a0
        w = [x for x in osc_on if t_on <= x[0] <= z0 + 10.0]
        ca0, ca1 = count_at("CA", t_on), count_at("CA", z0 + 20.0)
        oa0, oa1 = count_at("OA", t_on), count_at("OA", z0 + 20.0)
        d = ((ca1 or 0) - (ca0 or 0)) + ((oa1 or 0) - (oa0 or 0))
        kind = ("CA" if (ca1 or 0) > (ca0 or 0) else
                "OA" if (oa1 or 0) > (oa0 or 0) else "-")
        print("    %-12s %8s %10s %10s  %s"
              % (name,
                 ("%.1f s" % (w[0][0] - t_on)) if w else "none",
                 ("%.1f s" % (w[0][1] - w[0][0])) if w else "--",
                 "%d" % d, "YES (%s)" % kind if d else "NO"))

    # ---- CPAP current vs oscillation ------------------------------------------
    with cp.lock:
        bl = list(cp.b_log)
    bl = [(t - t_off, amps, rpm) for (t, amps, rpm) in bl]
    osc_win, on = [], None
    for s_ in sts:
        if s_["osc"] and on is None:
            on = s_["ts"]
        elif not s_["osc"] and on is not None:
            osc_win.append((on, s_["ts"]))
            on = None
    if on is not None:
        osc_win.append((on, sts[-1]["ts"]))

    def in_osc(t):   # S lines are ~4 s apart and ~2 s late: widen by 4 s
        return any(a0 - 4.0 <= t <= z0 + 4.0 for a0, z0 in osc_win)

    run_bl = [x for x in bl if 0.0 <= x[0] <= plan[-1][1] + 20.0]
    osc_amps = [x[1] for x in run_bl if in_osc(x[0])]
    br_amps = [x[1] for x in run_bl if not in_osc(x[0])]
    print("\n  CPAP supply current (%d samples over the script):" % len(run_bl))
    print("    oscillation windows: %s" % (", ".join("%.0f-%.0f s" % w for w in osc_win)
                                          or "none"))
    if osc_amps:
        print("    during oscillation : max %.2f A   (%d samples > 0.8 A)"
              % (max(osc_amps), sum(1 for v in osc_amps if v > 0.8)))
    if br_amps:
        print("    ordinary breathing : max %.2f A   (%d samples > 0.8 A)"
              % (max(br_amps), sum(1 for v in br_amps if v > 0.8)))
    top = sorted(run_bl, key=lambda x: -x[1])[:5]
    print("    top peaks:")
    for t, amps, rpm in sorted(top):
        print("      %6.1f s  %.2f A  CPAP %5d rpm  %-14s %s"
              % (t, amps, rpm, seg(t), "OSCILLATING" if in_osc(t) else "breathing"))
    if osc_amps and br_amps:
        verdict = ("SUPPORTED - >1 A only while oscillating"
                   if max(br_amps) <= 1.0 and max(osc_amps) > 1.0 else
                   "NOT SUPPORTED - >1 A also during ordinary breathing"
                   if max(br_amps) > 1.0 else
                   "no peak above 1 A this run")
        print("    FOT-current hypothesis: %s" % verdict)
    with open(SP + "/apap_flshape_current.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "amps", "cpap_rpm", "oscillating"])
        for t, amps, rpm in run_bl:
            w.writerow([round(t, 3), round(amps, 3), rpm, int(in_osc(t))])

    # ---- R5 capture: patient flow during each apnea -----------------------------
    with cp.lock:
        tl_ = [(x[0] - t_off,) + x[1:] for x in cp.t_log]
        ph_ = [(x[0] - t_off,) + x[1:] for x in cp.ph_log]
    print("\n" + "=" * 78)
    print("R5 CAPTURE - CPAP patient flow (T, 50 Hz) during each apnea")
    print("=" * 78)
    print("  T samples %d (%.1f Hz over the run), PH lines %d"
          % (len(tl_), len(tl_) / max(1.0, tl_[-1][0] - tl_[0][0]) if tl_ else 0.0,
             len(ph_)))
    ST = {0: "IDLE?", 1: "MON", 2: "NO_BR", 3: "FOT"}

    def sts_between(t0, t1):
        return [x for x in sts if t0 <= x["ts"] <= t1]

    def excursions(rows, thr):
        out, cur = [], None
        for r in rows:
            if r[3] > thr:
                if cur is None:
                    cur = [r[0], r[0], r[3], r[0], 0.0]
                cur[1] = r[0]
                if r[3] > cur[2]:
                    cur[2], cur[3] = r[3], r[0]
                cur[4] += r[3] / 60.0 * 1000.0 * 0.02   # mL, 50 Hz
            elif cur is not None:
                out.append(cur)
                cur = None
        if cur is not None:
            out.append(cur)
        return out

    starts_ = [e for e in sim_events if e["kind"] == "START" and e["event"] == "Apnea"]
    ends_ = [e for e in sim_events if e["kind"] == "END" and e["event"] == "Apnea"]
    cap_rows = []
    for k, (a0, z0, name) in enumerate([x for x in plan if x[2].startswith("apnea")]):
        st_ = [e["t"] for e in starts_ if a0 - 2.0 <= e["t"] <= z0 + 2.0]
        en_ = [e["t"] for e in ends_ if a0 <= e["t"] <= z0 + 8.0]
        t_on = st_[0] if st_ else a0
        t_end = en_[0] if en_ else z0
        win = [r for r in tl_ if t_on - 3.0 <= r[0] <= t_end + 3.0]
        ins = [r for r in tl_ if t_on + 0.3 <= r[0] <= t_end]
        print("\n  -- apnea %d (%s): sim onset %.1f s, end %.1f s (%.1f s long)"
              % (k + 1, name, t_on, t_end, t_end - t_on))
        if not ins:
            print("     no T samples!")
            continue
        q = [r[3] for r in ins]
        pk = max(ins, key=lambda r: r[3])
        print("     patient flow in the apnea: mean %+.2f  min %+.2f  max %+.2f slm "
              "(max at +%.1f s)" % (statistics.mean(q), min(q), pk[3], pk[0] - t_on))
        pm = [r[6] for r in ins]
        print("     P_mask %.2f..%.2f cmH2O   PID rpm %d..%d"
              % (min(pm), max(pm), min(r[4] for r in ins), max(r[4] for r in ins)))
        # 1 s means: shows a slow drift (H1) vs a breath-shaped bump
        sec = []
        t = t_on
        while t < t_end:
            w = [r[3] for r in ins if t <= r[0] < t + 1.0]
            if w:
                sec.append("%+.1f" % statistics.mean(w))
            t += 1.0
        print("     1 s mean flow: %s" % " ".join(sec))
        for thr in (4.0, 2.0):
            ex = excursions(ins, thr)
            print("     excursions > %.0f slm: %s" % (thr, "none" if not ex else ""))
            for e in ex[:8]:
                print("       +%5.1f..+%5.1f s  (%.2f s)  peak %.1f slm at +%.1f s  %.0f mL"
                      % (e[0] - t_on, e[1] - t_on, e[1] - e[0] + 0.02, e[2],
                         e[3] - t_on, e[4]))
            if ex:
                break
        ss = sts_between(t_on - 2.0, t_end + 15.0)
        trans, prev = [], None
        for x in ss:
            key = (x["apst"], x["osc"])
            if key != prev:
                trans.append("+%.1f %s%s leak %.1f" % (x["ts"] - t_on, ST.get(x["apst"], x["apst"]),
                                                      "/osc" if x["osc"] else "", x["leak"]))
                prev = key
        print("     APAP state (S, ~1 s, arrives late): %s" % " | ".join(trans))
        lk = [x["leak"] for x in ss if t_on <= x["ts"] <= t_end]
        if lk:
            print("     leak estimate in the apnea: %.2f -> %.2f (min %.2f)"
                  % (lk[0], lk[-1], min(lk)))
        phs = [x for x in ph_ if t_on - 1.0 <= x[0] <= t_end + 1.0]
        print("     PH lines: %s" % (", ".join("+%.1f src%d Vt %.0f" % (x[0] - t_on, x[3], x[2])
                                               for x in phs) or "none"))
        exits = [x["ts"] for i, x in enumerate(ss[1:], 1)
                 if ss[i - 1]["apst"] == 2 and x["apst"] == 1]
        nb = [x["ts"] for x in ss if x["apst"] == 2]
        fot = [x["ts"] for x in ss if x["osc"]]
        cap_rows.append(dict(k=k + 1, name=name, t_on=t_on, dur=t_end - t_on,
                             qmax=pk[3], t_qmax=pk[0] - t_on,
                             nb=(nb[0] - t_on) if nb else None,
                             early_exit=(exits[0] - t_on) if exits and (not fot or exits[0] < fot[0]) else None,
                             fot=(fot[0] - t_on) if fot else None,
                             physio=any(x[3] == 1 for x in phs)))
        with open(SP + "/apap_flshape_apnea%d.csv" % (k + 1), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_rel_s", "tick", "p_sensor", "q_patient", "pid_rpm",
                        "mech_rpm", "p_mask"])
            for r in win:
                w.writerow([round(r[0] - t_on, 3)] + list(r[1:]))

    print("\n  SUMMARY (times from sim apnea onset; S-line times arrive ~1-2 s late)")
    print("    %-3s %-10s %6s %8s %9s %9s %10s %8s %7s"
          % ("#", "segment", "dur", "q max", "q max at", "NO_BR", "early exit",
             "FOT", "physio"))
    for c in cap_rows:
        f = lambda v: ("+%.1f" % v) if v is not None else "--"
        print("    %-3d %-10s %5.1fs %6.1f   %8s %9s %10s %8s %7s"
              % (c["k"], c["name"], c["dur"], c["qmax"], f(c["t_qmax"]), f(c["nb"]),
                 f(c["early_exit"]), f(c["fot"]), "YES" if c["physio"] else "no"))
    with open(SP + "/apap_flshape_T.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "tick", "p_sensor", "q_patient", "pid_rpm", "mech_rpm", "p_mask"])
        for r in tl_:
            w.writerow([round(r[0], 3)] + list(r[1:]))
    with open(SP + "/apap_flshape_PH.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "tick", "vt_ml", "fot_src", "p_pp", "q_pp"])
        for r in ph_:
            w.writerow([round(r[0], 3)] + list(r[1:]))

    # ---- apap diag summary (R11) -------------------------------------------------
    with cp.lock:
        dl = [(x[0] - t_off,) + x[1:] for x in cp.diag_log]
    ep = [x for x in dl if x[1] == "epr"]
    ap = [x for x in dl if x[1] == "apap"]
    print("\n" + "=" * 78)
    print("APAP DIAG (R11) - %d apap / %d epr lines" % (len(ap), len(ep)))
    print("=" * 78)
    if ep:
        def f(x, k):
            try:
                return float(x[2].get(k, "nan"))
            except ValueError:
                return float("nan")
        bases = [f(x, "base") for x in ep]
        print("  EPR flow baseline over the run: min %+.2f  max %+.2f slm (R11: should stay near 0)"
              % (min(bases), max(bases)))
        print("  breath candidates: rejects %s at start -> %s at end; volume median %s -> %s mL"
              % (ep[0][2].get("rej"), ep[-1][2].get("rej"), ep[0][2].get("med_ml"), ep[-1][2].get("med_ml")))
        first = [x for x in dl if x[0] >= -5.0][:2]
        for x in first:
            print("  at script start  %s: %s" % (x[1], " ".join("%s=%s" % kv for kv in x[2].items())))
        last = dl[-2:]
        for x in last:
            print("  last (t=%.0f s)  %s: %s" % (x[0], x[1], " ".join("%s=%s" % kv for kv in x[2].items())))
    with open(SP + "/apap_flshape_diag.csv", "w") as fh:
        keys = sorted({k for x in dl for k in x[2]})
        fh.write("t_s,kind," + ",".join(keys) + "\n")
        for x in dl:
            fh.write("%.2f,%s," % (x[0], x[1]) + ",".join(x[2].get(k, "") for k in keys) + "\n")

    # ---- scorecard per fix ---------------------------------------------------
    def first_rise(key, t0, t1):
        prev = count_at(key, t0)
        for a in aps:
            if t0 < a["ts"] <= t1 and prev is not None and a[key] > prev:
                return a["ts"]
        return None

    print("\n" + "=" * 78)
    print("SCORECARD vs the R4/R5 build")
    print("=" * 78)
    segs = list(plan)
    starts = {}
    for e in sim_events:
        if e["kind"] == "START":
            starts.setdefault(round(e["t"]), e["t"])
    def onset(a0, z0):
        c = [e["t"] for e in sim_events if e["kind"] == "START"
             and e["event"] in ("Apnea", "Hypopnea") and a0 - 2.0 <= e["t"] <= z0]
        return c[0] if c else a0
    ap_i = 0
    last_apnea = None
    for i, (a0, z0, name) in enumerate(segs):
        if name.startswith("apnea"):
            ap_i += 1
            t_on = onset(a0, z0)
            ca = first_rise("CA", t_on, z0 + 20.0)
            oa = first_rise("OA", t_on, z0 + 20.0)
            dur = int(round(z0 - a0))
            tag = "R5" if i < len(segs) - 2 else "R1"
            print("  %-5s apnea %2d s%s: %s"
                  % (tag, dur, " + arousal" if tag == "R1" else "",
                     ("scored %s %.1f s after onset" % ("CA" if ca else "OA",
                                                       (ca or oa) - t_on))
                     if (ca or oa) else "NOT scored"))
            if tag == "R1":
                last_apnea = (z0, segs[i + 1][1] if i + 1 < len(segs) else z0 + 90)
        elif name.startswith("hypopnea"):
            t_on = onset(a0, z0)
            hyp = first_rise("Hyp", t_on, z0 + 20.0)
            ca = first_rise("CA", t_on, z0 + 20.0)
            fl = first_rise("FL", t_on, z0 + 20.0)
            tag = "R4" if name.endswith("60%") else "hyp"
            c0, c1 = count_at("Hyp", t_on), count_at("Hyp", z0 + 25.0)
            n_hyp = (c1 or 0) - (c0 or 0)
            n_osa = sum(1 for x in sts if t_on <= x["ts"] <= z0 + 10.0 and x["osa"])
            print("  %-5s %-13s: %s  [Hyp events %d%s]%s%s%s"
                  % (tag, name,
                     ("Hyp %.1f s in" % (hyp - t_on)) if hyp else "NO Hyp",
                     n_hyp, "  !! DOUBLE-COUNTED" if n_hyp > 1 else "",
                     ("; CA %.1f s in" % (ca - t_on)) if ca else "",
                     ("; FL %.1f s in" % (fl - t_on)) if fl else "",
                     ("; OSA-likely x%d" % n_osa) if n_osa else ""))
    if last_apnea:
        z0, z1 = last_apnea
        fl = first_rise("FL", z0, z1)
        print("  R1    after the arousal breaths: %s"
              % (("FALSE FL call %.1f s after the apnea" % (fl - z0)) if fl
                 else "no FL call - fixed"))
    if aps:
        L = aps[-1]
        hrs = L["tms"] / 3600000.0 if L["tms"] else 0.0
        exp_ahi = (L["OA"] + L["CA"] + L["Hyp"]) / hrs if hrs > 0 else float("nan")
        print("  AHI   %.2f   (OA %d + CA %d + Hyp %d) / %.3f h = %.2f  -> %s"
              % (L["AHI"], L["OA"], L["CA"], L["Hyp"], hrs, exp_ahi,
                 "includes CA" if abs(L["AHI"] - exp_ahi) < 0.2 else "MISMATCH"))

    if aps:
        last = aps[-1]
        print("\n  totals: OA %d CA %d Hyp %d FL %d Snore %d  AHI %.2f   "
              "polarity %d (bad %d)   leak-blocked raises seen: %s"
              % (last["OA"], last["CA"], last["Hyp"], last["FL"],
                 last["Snore"], last["AHI"], last["pol"], last["pol_bad"],
                 "yes" if any(x["leak_blk"] for x in aps) else "no"))
    osa = [x for x in sts if x["osa"]]
    print("  OSA-likely calls during the run: %d samples" % len(osa))
    print("  longest continuous FOT %.0f s (watchdog %.0f s)" % (cp.osc_max_s, FOT_MAX_S))
    print("  peak CPAP supply current %.2f A   simulator sensors %s"
          % (cp.i_max, "OK" if alive else "!! NOT READING AT END"))

    with open(SP + "/apap_flshape_cpap.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "OA", "CA", "Hyp", "FL", "Snore", "AHI", "st",
                    "hyp", "fl", "sn", "leak_blk", "pol"])
        for a in aps:
            w.writerow([round(a["ts"], 2), a["OA"], a["CA"], a["Hyp"], a["FL"],
                        a["Snore"], a["AHI"], a["st"], a["hyp"], a["fl"],
                        a["sn"], a["leak_blk"], a["pol"]])
    with open(SP + "/apap_flshape_status.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "target", "meas", "leak", "apap_state", "osc",
                    "fot_p", "fot_q", "osa"])
        for s_ in sts:
            w.writerow([round(s_["ts"], 2), s_["target"], s_["meas"],
                        s_["leak"], s_["apst"], s_["osc"], s_["fot_p"],
                        s_["fot_q"], s_["osa"]])
    with open(SP + "/apap_flshape_sim.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "breath", "vt", "qpk", "flat", "event"])
        for r in sim_rows:
            w.writerow([round(r["t"], 2), r["breath"], r["vt"], r["qpk"],
                        r["flat"], r["event"]])
    print("  logs: apap_flshape_cpap.csv / _status.csv / _sim.csv (runs/)")

except Abort as a:
    print("\n  !! ABORT: %s" % a)
    if cp is not None:
        with cp.lock:
            last = list(cp.diag_log[-2:])
        for x in last:
            print("  last diag  %s: %s" % (x[1], " ".join("%s=%s" % kv for kv in x[2].items())))
except SystemExit as e:
    print("\n  !! NOT STARTED: %s" % e)
finally:
    try:
        send("sim script stop")
        ramp_down()
        send("sim set ff static")
        send("sim set recovery 0")
        send("sim set plimit 0")
        send("log 3")
    except Exception:
        pass
    if cp is not None:
        try:
            # A dropped status reply must not skip the stop (trial 7 left the
            # CPAP running that way). Unknown = stop; it is harmless when idle.
            st = None
            for _ in range(3):
                st = cp.status()
                if st is not None:
                    break
            if st is None or st["state"] != 0:
                cpap_safe_stop(cp)
            if orig is not None:
                cp.cmd("therapy set pressure_min %.1f" % orig["amin"])
                cp.cmd("therapy set pressure_max %.1f" % orig["amax"])
                if orig["mode"] == 1:
                    cp.cmd("therapy set mode autoset")
                elif 4.0 <= orig["target"] <= 25.0:
                    # the stop left the CPAP-mode setpoint at 4
                    cp.cmd("therapy set pressure %.1f" % orig["target"])
                cp.cmd("blower set telem_motor 0")
                time.sleep(2.5)
                st = cp.status()
                print("  [CPAP] restored: mode %s, target %s, APAP range %s-%s, "
                      "state %s" % (st and st["mode"], st and st["target"],
                                    st and st["amin"], st and st["amax"],
                                    st and st["state"]))
        except Exception as e:
            print("  !! CPAP teardown error: %s - CHECK THE CPAP BY HAND" % e)
        cp.close()
    ser.close()
    print("\nsimulator stopped. CPAP handled above.")
