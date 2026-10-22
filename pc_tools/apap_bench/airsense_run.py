#!/usr/bin/env python3
"""AirSense 11 reference run: same rig, same simulator scripts as our APAP trials.

Rig: AirSense 11 (own hose) -> vent -> external sensor (COM4) -> collapsible
chamber (squeeze blower OFF unless the script says otherwise) -> simulator
box -> 2 L lung. Our CPAP is not connected; this harness never talks to the
AirSense - the user starts and stops its therapy by hand, and the harness
detects both from the external sensor's pressure.

AirSense settings (clinical menu): AutoSet, min 8, max 12 cmH2O, EPR off,
Ramp off / AutoRamp off, a fixed mask type, Response standard.

Sequence (simulator first, as in trial 8c):
  1. checks (simulator sensors, external sensor live)
  2. simulator starts breathing on static FF (AirSense off), primes 20 s
  3. waits up to 5 min for therapy: external pressure > 5 cmH2O for 2 s
  4. simulator -> pressure FF; settle 60 s
  5. script; ground truth from the simulator's V/E lines, external sensor
     logged throughout; PC wall clock recorded for the EDF alignment
  6. verified simulator ramp-down, then "STOP THE AIRSENSE"; logs 30 s more

Scripts: 8c (apneas + hypopneas, as trial 8c), threshold (45 % / 55 %
hypopneas), flshape (flattened breaths at full amplitude: plateau 90 %,
M 90 %, plateau 60 %; 120 s each with 120 s normal between), obstruct
(five 30 s apneas; 2, 4 and 5 with the chamber squeezed by hand on PC beeps).

Saves airsense_<script>_<HHMM>_raw.pkl; compare with airsense_compare.py once
the SD card's DATALOG files are copied over.

usage: python airsense_run.py 8c|threshold|flshape|obstruct
"""
import pickle
import statistics
import sys
import time

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
sys.path.insert(0, HERE)
NAME = sys.argv.pop(1) if len(sys.argv) > 1 else "8c"   # before phase1 reads argv

from phase1 import send, ramp_down, ser
import liveness
from ext_sensor import ExtSensorLogger, looks_frozen

SCRIPTS = {
    "8c": [("none", 90, None),
           ("apnea", 5, 100), ("none", 45, None),
           ("apnea", 10, 100), ("none", 45, None),
           ("apnea", 10, 100), ("none", 45, None),
           ("apnea", 10, 100), ("none", 45, None),
           ("apnea", 20, 100), ("none", 60, None),
           ("apnea", 20, 100), ("none", 60, None),
           ("apnea", 20, 100), ("none", 60, None),
           ("hypopnea", 40, 50), ("none", 60, None),
           ("hypopnea", 40, 55), ("none", 60, None),
           ("hypopnea", 40, 60), ("none", 60, None),
           ("apnea", 20, 100), ("none", 90, None)],
    "threshold": [("none", 90, None),
                  ("hypopnea", 40, 45), ("none", 60, None),
                  ("hypopnea", 40, 45), ("none", 60, None),
                  ("hypopnea", 40, 45), ("none", 60, None),
                  ("hypopnea", 40, 55), ("none", 60, None),
                  ("hypopnea", 40, 55), ("none", 60, None),
                  ("hypopnea", 40, 55), ("none", 60, None)],
    # Flattened breaths at full amplitude. fl_shape is a global simulator
    # setting, so the harness switches it in the normal gaps (ACTIONS below).
    "flshape": [("none", 90, None),
                ("flowlimit", 120, 90), ("none", 120, None),
                ("flowlimit", 120, 90), ("none", 120, None),
                ("flowlimit", 120, 60), ("none", 120, None)],
    # Obstructive apneas: simulator apnea (blower at rpm_base - no effort
    # against the closed tube) with the collapsible chamber squeezed by hand
    # on apneas 2, 4 and 5; 1 and 3 stay open as controls. PC beeps cue the
    # squeeze (see CUES).
    "obstruct": [("none", 90, None),
                 ("apnea", 30, 100), ("none", 60, None),
                 ("apnea", 30, 100), ("none", 60, None),
                 ("apnea", 30, 100), ("none", 60, None),
                 ("apnea", 30, 100), ("none", 60, None),
                 ("apnea", 30, 100), ("none", 90, None)],
}
SQUEEZE_APNEAS = {"obstruct": {2, 4, 5}}     # 1-based apnea numbers to squeeze
SQUEEZE_ON_S, SQUEEZE_OFF_S = 5.0, 24.0      # after the apnea step starts
# Shape of each flowlimit step, in order (recorded in the raw pickle).
FL_SHAPES = {"flshape": ["plateau", "m", "plateau"]}
if NAME not in SCRIPTS:
    raise SystemExit("script must be one of %s" % ", ".join(SCRIPTS))
SCRIPT = SCRIPTS[NAME]
SCRIPT_S = sum(d for _, d, _ in SCRIPT)
# Host actions at script times: recovery breaths before the last apnea of the
# 8c script (as in our trials); fl_shape switches 60 s into the normal gap
# before each flowlimit step of the flshape script.
ACTIONS = []
if NAME == "8c":
    ACTIONS.append((sum(d for _, d, _ in SCRIPT[:-2]) - 8.0,
                    ["sim set recovery 2", "sim set recovery_gain 1.5"],
                    "recovery breaths ON for the last apnea"))
shapes = list(FL_SHAPES.get(NAME, []))
acc = 0.0
for i, (ev, dur, sev) in enumerate(SCRIPT):
    if ev == "flowlimit" and shapes:
        shape = shapes.pop(0)
        at = max(0.0, acc - 60.0) if i else 0.0
        ACTIONS.append((at, ["sim set fl_shape %s" % shape], "fl_shape -> %s" % shape))
    acc += dur
ACTIONS.sort(key=lambda a: a[0])

# Squeeze cues on the script clock. The simulator starts an apnea at the next
# breath boundary (<= 5 s after its step), and it lasts the step's 30 s from
# there, so ON at +5 s and OFF at +24 s always sits inside the apnea and
# leaves >= 6 s for the tube to reopen before breathing resumes.
CUES = []
acc, k = 0.0, 0
for ev, dur, sev in SCRIPT:
    if ev == "apnea":
        k += 1
        if k in SQUEEZE_APNEAS.get(NAME, set()):
            CUES += [(acc + SQUEEZE_ON_S - 5.0, "ready", k), (acc + SQUEEZE_ON_S, "squeeze", k),
                     (acc + SQUEEZE_OFF_S, "release", k)]
    acc += dur
CUES.sort(key=lambda c: c[0])


def beep(kind):
    """Audible cue on the PC, in a thread so the logging never stalls.
    ready   - one short low tone   (squeeze coming in 5 s)
    squeeze - one long high tone   (squeeze blower to MAX now)
    release - two short tones      (squeeze blower OFF now)"""
    import threading
    import winsound

    def play():
        if kind == "ready":
            winsound.Beep(440, 300)
        elif kind == "squeeze":
            winsound.Beep(880, 1200)
        else:
            winsound.Beep(1320, 250)
            time.sleep(0.15)
            winsound.Beep(1320, 250)
    threading.Thread(target=play, daemon=True).start()
SIM_GUARD = 36000
P_ON, P_OFF = 5.0, 2.0                     # cmH2O at the external sensor


class Abort(Exception):
    pass


ext = None
sim_f, sim_v, sim_e = [], [], []
t_script = t_therapy = None
wall_off = time.time() - time.monotonic()   # host monotonic -> PC wall clock


def ext_pressure(window_s=2.0):
    now = time.monotonic()
    p = [s[1] for s in ext.samples[-400:] if s[0] >= now - window_s]
    return statistics.median(p) if p else float("nan")


def pump(seconds, log=False):
    """Read the simulator for `seconds`: rpm guard, and (log=True) ground truth."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if ext.error:
            raise Abort("external sensor stopped: %s" % ext.error)
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", "replace").strip()
        p = t.split(",")
        now = time.monotonic()
        try:
            if t.startswith("B,") and len(p) >= 8:
                if int(p[6]) > SIM_GUARD:
                    raise Abort("simulator commanded %s rpm" % p[6])
            elif not log:
                continue
            elif t.startswith("F,") and len(p) >= 6:
                sim_f.append((now, int(p[1]), float(p[3]), float(p[4]), int(p[5])))
            elif t.startswith("V,") and len(p) >= 13:
                sim_v.append((now, int(p[1]), float(p[2]), float(p[9]), float(p[11]), p[12]))
            elif t.startswith("E,") and len(p) >= 6:
                sim_e.append((now, int(p[2]), p[3], p[4]))
                print("  t=%3.0fs  SIM  E %-5s %s" % (now - t_script, p[3], p[4]), flush=True)
        except ValueError:
            pass


def save_raw(alive, aborted=False):
    """Pickle everything captured so far (also called from the abort path)."""
    stamp = time.strftime("%H%M", time.localtime(wall_off + t_script))
    out = SP + "/airsense_%s_%s%s_raw.pkl" % (NAME, stamp, "_ABORTED" if aborted else "")
    raw = dict(kind="airsense", script_name=NAME, script=SCRIPT, t_script=t_script,
               fl_shapes=FL_SHAPES.get(NAME, []),
               squeeze_apneas=sorted(SQUEEZE_APNEAS.get(NAME, set())),
               cue_log=list(globals().get("cue_log", [])), aborted=aborted,
               t_therapy=t_therapy, wall_off=wall_off, ext=list(ext.samples),
               ext_crc=ext.parser.crc_errors, sim_f=sim_f, sim_v=sim_v, sim_e=sim_e,
               alive=alive)
    with open(out, "wb") as fh:
        pickle.dump(raw, fh)
    return out


try:
    send("log 0")
    send("telem on")
    send("sim stop")
    send("flow on")
    send("press on")

    print("=" * 78)
    print("AIRSENSE REFERENCE RUN - script '%s' (%d s)" % (NAME, SCRIPT_S))
    print("=" * 78)
    if not liveness.check(ser, "before"):
        raise SystemExit("simulator sensors not reading")
    try:
        ext = ExtSensorLogger(None)
    except Exception as e:
        raise SystemExit("cannot open external sensor COM4: %s" % e)
    ext.start()
    time.sleep(3.0)
    es = [x for x in ext.samples if x[0] > time.monotonic() - 2.0]
    if looks_frozen(es):
        raise SystemExit("external sensor frozen or silent (%d samples in 2 s) - "
                         "power-cycle its board" % len(es))
    p0 = ext_pressure()
    print("  external sensor live (%d samples in 2 s), pressure %.2f cmH2O%s"
          % (len(es), p0, "  - AirSense already running" if p0 > P_ON else ""))

    if CUES:
        print("\n  squeeze cues - listen (apneas %s will be squeezed):"
              % sorted(SQUEEZE_APNEAS.get(NAME, set())), flush=True)
        for kind, what in (("ready", "short low tone  = squeeze in 5 s"),
                           ("squeeze", "long high tone  = squeeze blower to MAX now"),
                           ("release", "two short tones = squeeze blower OFF now")):
            print("    %s" % what, flush=True)
            beep(kind)
            time.sleep(2.5)

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
    print("  simulator breathing (static FF) - priming 20 s", flush=True)
    pump(20.0)

    print("\n  >>> START THE AIRSENSE THERAPY NOW (waiting up to 5 min) <<<", flush=True)
    end = time.monotonic() + 300.0
    since = None
    while time.monotonic() < end:
        pump(0.5)
        if ext_pressure() > P_ON:
            since = since or time.monotonic()
            if time.monotonic() - since >= 2.0:
                break
        else:
            since = None
    else:
        raise Abort("no therapy pressure seen within 5 min")
    t_therapy = time.monotonic()
    print("  therapy detected: %.2f cmH2O at the external sensor" % ext_pressure(), flush=True)

    pump(6.0)
    send("sim set ff pressure")
    print("  simulator -> pressure FF; settling 60 s", flush=True)
    pump(60.0)

    print("  script start: %s\n" % " | ".join(
        "%s %d%s" % (e, d, "" if s is None else " %d%%" % s) for e, d, s in SCRIPT), flush=True)
    send("sim script start")
    t_script = time.monotonic()
    pending = list(ACTIONS)
    cues = list(CUES)
    cue_log = []
    next_note = t_script + 30.0
    while time.monotonic() < t_script + SCRIPT_S + 10.0:
        pump(0.5, log=True)
        now = time.monotonic()
        while cues and now - t_script >= cues[0][0]:
            _, kind, k = cues.pop(0)
            beep(kind)
            cue_log.append((now - t_script, kind, k))
            print("  t=%3.0fs  CUE  %s (apnea %d)%s"
                  % (now - t_script, kind.upper(), k,
                     {"ready": " - squeeze in 5 s", "squeeze": " - squeeze blower MAX",
                      "release": " - squeeze blower OFF"}[kind]), flush=True)
        while pending and now - t_script >= pending[0][0]:
            _, cmds, what = pending.pop(0)
            for c in cmds:
                send(c)
            print("  t=%3.0fs  SIM  %s" % (now - t_script, what), flush=True)
        if now >= next_note:
            p = ext_pressure()
            print("  t=%3.0fs  external pressure %.2f cmH2O%s"
                  % (now - t_script, p, "   !! therapy pressure lost" if p < P_OFF else ""),
                  flush=True)
            next_note = now + 30.0

    ramp_down()
    alive = liveness.check(ser, "after")
    print("\n  >>> STOP THE AIRSENSE THERAPY NOW (logging 30 s more) <<<", flush=True)
    time.sleep(30.0)
    ext.stop()

    out = save_raw(alive=alive)

    print("\n" + "=" * 78)
    print("GROUND TRUTH - what the simulator delivered (V lines)")
    print("=" * 78)
    by = {}
    for r in sim_v:
        by.setdefault(r[5], []).append(r)
    for ev, rows in by.items():
        print("  %-10s %3d breaths  Vt %5.0f  peak %5.1f  flat %.3f"
              % (ev, len(rows), statistics.median(r[2] for r in rows),
                 statistics.median(r[3] for r in rows), statistics.median(r[4] for r in rows)))
    print("\n  saved %s" % out)
    print("  PC clock at script start: %s" % time.strftime(
        "%Y-%m-%d %H:%M:%S", time.localtime(wall_off + t_script)))
    print("  next: copy the SD card's DATALOG/<date> folder, then airsense_compare.py")

except Abort as a:
    print("\n  !! ABORT: %s" % a)
    # Keep what was captured: an aborted run is still evidence (2026-10-20:
    # the obstruct run aborted after apnea 4 and its capture was lost).
    if t_script is not None and ext is not None:
        try:
            print("  partial capture saved: %s" % save_raw(alive=None, aborted=True))
        except Exception as ex:
            print("  !! could not save the partial capture: %s" % ex)
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
    if ext is not None and ext.is_alive():
        ext.stop()
    ser.close()
    if CUES:
        print("  !! make sure the squeeze blower is OFF")
    print("\nsimulator stopped. Stop the AirSense by hand if it is still running.")
