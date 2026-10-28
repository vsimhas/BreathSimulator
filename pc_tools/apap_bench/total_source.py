#!/usr/bin/env python3
"""Total-flow capture (R10): where does the CPAP lose ~4 slm of inspiratory
flow - in its flow measurement, or in the leak/vent model it subtracts?

Same rig, script and three sensors as leak_source.py, plus the CPAP's TOTAL
flow from its B line (field 10, Flow_GetSlmCtrl(), ~25 Hz, same device clock
as the T line). Per simulator breath:
  CPAP total - external   = the vent flow as the CPAP's sensor sees it
                            (vent is between the two) - flat if the sensor
                            reads right
  CPAP total - CPAP patient = the leak/vent model the CPAP subtracts

CPAP fixed 8 cmH2O (no APAP), EPR 0, ramp 0; restored afterwards. Simulator:
pressure FF, 12 bpm, 500 mL. Script:
  none 60 | hypopnea 60% 60 | none 60 | hypopnea 50% 60 | none 60   (300 s)

PREDICTIONS, written before the run (2026-10-19): the CPAP total-minus-
external stays flat across normal and hypopnea breaths (the sensor is fine),
and the CPAP's subtracted model (total - patient) rises by ~1.5-2 slm in the
hypopneas - i.e. the error is in the leak/vent/patient model. Weak lean: the
inspiratory-peak deficit is the same ~4 slm for big and small breaths, which
fits a model term more than a sensor gain error.

Guards: CPAP current > 0.8 A for > 0.7 s, Vbus < 21 V, drive fault, mask-off;
simulator 36000 rpm; sensor liveness. Teardown: verified simulator ramp-down,
CPAP setpoint 4 -> stop from < 14.5k rpm (also when status is unknown).
"""
import pickle
import re
import sys
import threading
import time

import serial

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
sys.path.insert(0, HERE)

if len(sys.argv) > 1 and sys.argv[1] == "--analyze":
    # --analyze [file]   default: the 19 October R10 capture
    import total_analysis
    f = sys.argv[2] if len(sys.argv) > 2 else SP + "/total_raw.pkl"
    total_analysis.main(pickle.load(open(f, "rb")))
    sys.exit(0)

# #25 (28 Oct): optional CPAP pressure; phase1 takes argv[1] as its port.
CPAP_P = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
del sys.argv[1:]
OUT = SP + "/total_raw_P%g_%s.pkl" % (CPAP_P, time.strftime("%Y%m%d_%H%M%S"))

from phase1 import send, ramp_down, ser
import liveness
from ext_sensor import ExtSensorLogger, looks_frozen

CPAP_PORT = "COM13"
SIM_GUARD = 36000
I_LIMIT, I_HOLD_S, VBUS_MIN = 0.8, 0.7, 21.0
SCRIPT = [("none", 60, None), ("hypopnea", 60, 60), ("none", 60, None),
          ("hypopnea", 60, 50), ("none", 60, None)]
SCRIPT_S = sum(d for _, d, _ in SCRIPT)


class Abort(Exception):
    pass


class Cpap:
    """Background reader: B (guard), S (status), T (50 Hz flow) lines."""

    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2, write_timeout=2.0)
        self.lock = threading.Lock()
        self.b = self.st = None
        self.t_log, self.st_log, self.b_log = [], [], []
        self.bt_log = []   # (host, tick, total flow, dp_raw)
        self.fw_reply = 0
        self.abort = None
        self.i_over = None
        self.i_max = 0.0
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
                if t.startswith("T,") and len(p) >= 7:
                    with self.lock:
                        self.t_log.append((now, int(p[1]), float(p[2]), float(p[3]),
                                           int(p[4]), int(p[5]), float(p[6])))
                elif t.startswith("B,") and len(p) >= 22:
                    vbus = int(p[2]) / 1000.0
                    amps = ((1.5 * (int(p[19]) * int(p[3]) +
                                    int(p[20]) * int(p[5])) / 1e6) / vbus
                            if vbus > 1.0 else 0.0)
                    b = dict(vbus=vbus, rpm=int(p[6]), faults=int(p[9]), amps=amps)
                    with self.lock:
                        self.b = b
                        self.b_log.append((now, amps, b["rpm"]))
                        self.bt_log.append((now, int(p[1]), float(p[10]), float(p[11])))
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
                elif t.startswith("S,") and len(p) >= 21:
                    st = dict(t=now, state=int(p[2]), target=float(p[3]),
                              meas=float(p[5]), leak=float(p[6]), maskoff=int(p[8]),
                              mode=int(p[10]), amin=float(p[11]), amax=float(p[12]),
                              apst=int(p[13]), osc=int(p[14]))
                    if len(p) >= 36:
                        # #25: VIVO state - MeanRatio, its 10 s instant mean,
                        # vent ref, uninten, flow leak, rest volume, breath
                        # FSM insp flag, rate, last insp / exp volumes.
                        st.update(mr=int(p[25]), mr10=int(p[26]), vent=float(p[27]),
                                  unint=float(p[28]), fleak=float(p[29]),
                                  rest=float(p[31]), binsp=int(p[32]), bpm=float(p[33]),
                                  vi=float(p[34]), ve=float(p[35]))
                    with self.lock:
                        self.st = st
                        self.st_log.append(st)
                        if st["state"] and st["maskoff"] and not self.abort:
                            self.abort = "CPAP reports mask-off"
                elif "apap set breath_min_insp_ms" in t or (
                        t.startswith("# err") and self.fw_reply is None):
                    with self.lock:
                        self.fw_reply = t
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
    cp.cmd("therapy set pressure 4")
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


cp = ext = None
orig = None
sim_f, sim_p, sim_e, sim_v = [], [], [], []
t_script = None
try:
    send("log 0")
    send("telem on")
    send("sim stop")
    send("flow on")
    send("press on")
    send("telem off")

    try:
        ext = ExtSensorLogger(None)
    except Exception as e:
        raise SystemExit("cannot open external sensor COM4: %s" % e)
    ext.start()

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
    orig = dict(mode=st["mode"], amin=st["amin"], amax=st["amax"], target=st["target"])
    print("  CPAP state %d mode %d measured %.2f target %.1f  APAP range %.1f-%.1f  %s"
          % (st["state"], st["mode"], st["meas"], st["target"], st["amin"], st["amax"],
             ("Vbus %.2f V" % b["vbus"]) if b else "NO B LINE"))
    if st["state"] != 0:
        raise SystemExit("CPAP not idle")
    if abs(st["meas"]) > 1.0:
        raise SystemExit("CPAP pressure %.2f at rest - stuck sensor?" % st["meas"])
    if b is None or not (22.0 <= b["vbus"] <= 26.0):
        raise SystemExit("CPAP B line missing or Vbus out of range")
    if not liveness.check(ser, "before"):
        raise SystemExit("simulator sensors not reading")
    es = [x for x in ext.samples if x[0] > time.monotonic() - 2.0]
    if looks_frozen(es):
        raise SystemExit("external sensor frozen or silent (%d samples in 2 s) - "
                         "power-cycle its board" % len(es))
    print("  external sensor live: %d samples in the last 2 s" % len(es))

    with cp.lock:
        cp.fw_reply = None
    cp.cmd("apap set breath_min_insp_ms 400")
    end = time.monotonic() + 4.0
    while time.monotonic() < end and cp.fw_reply is None:
        time.sleep(0.1)
    print("  CPAP firmware probe: %s" % (cp.fw_reply or "no reply"))
    if not (cp.fw_reply and cp.fw_reply.startswith("# ack")):
        raise SystemExit("CPAP is not running the R4/R5 build")

    # ---- CPAP: fixed pressure -----------------------------------------------
    for c in ("therapy set ramp 0", "therapy set epr 0", "therapy set mode cpap",
              "therapy set pressure %.0f" % CPAP_P):
        cp.cmd(c)
    cp.cmd("therapy start")
    end = time.monotonic() + 25.0
    while time.monotonic() < end:
        if cp.abort:
            raise Abort(cp.abort)
        st = cp.status()
        if st and st["state"] == 2 and st["mode"] == 0:
            break
        time.sleep(1.0)
    if not (st and st["state"] == 2 and st["mode"] == 0):
        raise Abort("CPAP did not reach running CPAP mode (state %s mode %s)"
                    % (st and st["state"], st and st["mode"]))
    print("  CPAP running fixed %.1f cmH2O, measured %.2f" % (st["target"], st["meas"]))

    # ---- simulator ------------------------------------------------------------
    send("sim event cancel")      # ramp_down() leaves a 120 s apnea latched
    for c in ("sim set waveform sine", "sim set timing rate_ie",
              "sim set rate 12", "sim set ie 1", "sim set insp_pause 0",
              "sim set exp_pause 0", "sim set jitter 0",
              "sim set flattening 0", "sim set fl_shape plateau",
              "sim set exp_tau 0.4", "sim set rpm_base 8000",
              "sim set tidal 500", "sim set recovery 0", "sim set snore_hz 0",
              "sim set plimit 40", "sim set control flow",
              "sim set ff pressure", "sim script clear"):
        send(c)
    for ev, dur, sev in SCRIPT:
        send("sim script add %s %d%s" % (ev, dur, "" if sev is None else " %d" % sev))
    send("telem on")
    send("sim start")
    print("\n  simulator priming (30 s)...")
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end:
        if cp.abort:
            raise Abort(cp.abort)
        ser.readline()

    print("  script start: %s  (%d s)\n" % (" | ".join("%s %d" % x[:2] for x in SCRIPT),
                                           SCRIPT_S))
    send("sim script start")
    t_script = time.monotonic()
    t_stop = t_script + SCRIPT_S + 5.0
    next_poll, next_note = t_script, t_script + 30.0
    while time.monotonic() < t_stop:
        if cp.abort:
            raise Abort(cp.abort)
        if ext.error:
            raise Abort("external sensor stopped: %s" % ext.error)
        now = time.monotonic()
        if now >= next_poll:
            cp.cmd("therapy status")
            next_poll = now + 1.0
        if now >= next_note:
            with cp.lock:
                s_, nt = cp.st, len(cp.t_log)
            print("  t=%3.0fs  CPAP meas %s leak %s | T %d  F %d  ext %d"
                  % (now - t_script, s_ and "%.2f" % s_["meas"],
                     s_ and "%.1f" % s_["leak"], nt, len(sim_f), len(ext.samples)))
            next_note = now + 30.0
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", "replace").strip()
        p = t.split(",")
        now = time.monotonic()
        try:
            if t.startswith("F,") and len(p) >= 6:
                sim_f.append((now, int(p[1]), float(p[3]), float(p[4]), int(p[5])))
            elif t.startswith("P,") and len(p) >= 7:
                sim_p.append((now, int(p[1]), float(p[4]), int(p[6])))
            elif t.startswith("B,") and len(p) >= 8:
                if int(p[6]) > SIM_GUARD:
                    raise Abort("simulator commanded %s rpm" % p[6])
            elif t.startswith("V,") and len(p) >= 13:
                sim_v.append((now, int(p[1]), float(p[2]), float(p[9]), p[12]))
            elif t.startswith("E,") and len(p) >= 6:
                sim_e.append((now, int(p[2]), p[3], p[4]))
                print("  t=%3.0fs  SIM  E %-5s %s" % (now - t_script, p[3], p[4]))
        except ValueError:
            pass

    ramp_down()
    alive = liveness.check(ser, "after")
    ext.stop()
    with cp.lock:
        raw_dump = dict(t_log=list(cp.t_log), st_log=list(cp.st_log),
                        b_log=list(cp.b_log), bt_log=list(cp.bt_log))
    raw_dump.update(ext=list(ext.samples), ext_crc=ext.parser.crc_errors,
                    sim_f=sim_f, sim_p=sim_p, sim_e=sim_e, sim_v=sim_v, t_script=t_script,
                    script=SCRIPT, i_max=cp.i_max, alive=alive)
    with open(OUT, "wb") as fh:
        pickle.dump(raw_dump, fh)
    print("\n  raw saved: %s  (T %d, ext %d, sim F %d, sim P %d)"
          % (OUT, len(raw_dump["t_log"]), len(raw_dump["ext"]), len(sim_f), len(sim_p)))
    print("  peak CPAP supply current %.2f A   simulator sensors %s"
          % (cp.i_max, "OK" if alive else "!! NOT READING AT END"))
    try:
        import total_analysis
        total_analysis.main(raw_dump)
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("  !! analysis failed (%s) - raw data is saved; rerun with --analyze" % e)

except Abort as a:
    print("\n  !! ABORT: %s" % a)
except SystemExit as e:
    print("\n  !! NOT STARTED: %s" % e)
finally:
    try:
        send("sim script stop")
        ramp_down()
        send("sim set ff static")
        send("sim set plimit 0")
        send("log 3")
    except Exception:
        pass
    if ext is not None and ext.is_alive():
        ext.stop()
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
                    cp.cmd("therapy set pressure %.1f" % orig["target"])
            cp.cmd("blower set telem_motor 0")
            time.sleep(2.5)
            st = cp.status()
            print("  [CPAP] restored: mode %s, target %s, APAP range %s-%s, state %s"
                  % (st and st["mode"], st and st["target"], st and st["amin"],
                     st and st["amax"], st and st["state"]))
        except Exception as e:
            print("  !! CPAP teardown error: %s - CHECK THE CPAP BY HAND" % e)
        cp.close()
    ser.close()
    print("\nsimulator stopped. CPAP handled above.")
