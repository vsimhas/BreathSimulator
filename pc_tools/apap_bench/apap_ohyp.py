#!/usr/bin/env python3
"""Central vs obstructive hypopneas on our CPAP (2026-10-23).

Built from apap_flshape.py (same rig, guards, start order and teardown).

Rig: CPAP (COM13) -> circuit -> collapsible chamber (squeeze blower OFF)
-> breath simulator (COM15) -> 2 L lung. AutoSet 8-12, EPR off, ramp 0.

Firmware under test (CPAP_InitialCode, after bdb69af): FL is called on
shape only (fl_amp_arm 0), and a hypopnea raises pressure only when at least
more than hyp_raise_shape_frac (0.5) of its limited breaths are shape-limited; a round
one is scored and holds the pressure. Probe: `apap set hyp_raise_shape_frac
0.5` must be acked (only that build knows it).

Script (480 s), 12 bpm I:E 1:1, 500 mL normal breaths:
  normal 90 | hypopnea 55 % 40 FLATTENED | normal 90 |
  hypopnea 55 % 40 ROUND | normal 90 | hypopnea 55 % 40 FLATTENED | normal 90
"Flattened" = `sim set flattening 0.9` (plateau) for the hypopnea, switched
on 2 s before it and off 2 s after it by the script clock.

PREDICTIONS (written before the run):
  flattened hypopnea  Hyp 1 + FL (shape), pressure RAISED (obstructive)
  round hypopnea      Hyp 1, no FL, pressure HELD (central; AirSense held 8.0)
  normal blocks       no events; lowering resumes ~20 s after each event
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
          ("hypopnea", 40, 55), ("none", 90, None),
          ("hypopnea", 40, 55), ("none", 90, None),
          ("hypopnea", 40, 55), ("none", 90, None)]
FL_SHAPES = []
HYP_KIND = ["flattened", "round", "flattened"]
# Global `flattening` switched by the script clock (E lines can drop).
FLAT_ACTIONS = []
_acc, _k = 0.0, list(HYP_KIND)
for _ev, _dur, _sev in SCRIPT:
    if _ev == "hypopnea" and _k.pop(0) == "flattened":
        FLAT_ACTIONS += [(_acc - 2.0, 0.9), (_acc + _dur + 2.0, 0.0)]
    _acc += _dur
SHAPE_ACTIONS = []
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
                elif ("apap set hyp_rearm_ratio" in t or "apap set fl_dip_ratio" in t
                      or "apap set hyp_raise_shape_frac" in t) or (
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
    with cp.lock:
        cp.fw_reply = None
    cp.cmd("apap set hyp_raise_shape_frac 0.5")
    end = time.monotonic() + 4.0
    while time.monotonic() < end and cp.fw_reply is None:
        time.sleep(0.1)
    print("  CPAP central/obstructive Hyp probe: %s" % (cp.fw_reply or "no reply"))
    if not (cp.fw_reply and cp.fw_reply.startswith("# ack")):
        raise SystemExit("CPAP is not running the central-hypopnea build")

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
        while FLAT_ACTIONS and now - t_script >= FLAT_ACTIONS[0][0]:
            _, _fl = FLAT_ACTIONS.pop(0)
            send("sim set flattening %.2f" % _fl)
            print("  t=%3.0fs  SIM  flattening -> %.2f" % (now - t_script, _fl))
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
    with open(SP + "/apap_ohyp_raw.pkl", "wb") as fh:
        pickle.dump(raw_dump, fh)
    print("  raw capture saved: apap_ohyp_raw.pkl (%d T lines, %d PH lines)"
          % (len(raw_dump["t_log"]), len(raw_dump["ph_log"])))


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
        send("sim set flattening 0")
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
