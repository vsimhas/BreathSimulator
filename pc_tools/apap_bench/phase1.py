#!/usr/bin/env python3
"""Phase 1: regression + the flattening-index prediction.

CPAP off, 2 L lung connected, so the simulator blower alone moves the gas.

The check that matters: an UNFLATTENED half-sine has a commanded
inspiratory mean/peak of 2/pi = 0.637. If the measured flat_index comes
back near that, the rig reproduces the commanded shape and the Phase 2
flow-limitation severities will mean what they say. If it comes back much
higher, the blower is flat-topping the breath and every FL number after
this would be measuring the actuator, not the morphology.

Also verifies the new V line parses with 13 fields and that the line
corruption is gone now that `log 0` silences the second CDC producer.
"""
import sys, time, statistics
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"
ser = serial.Serial(PORT, 115200, timeout=0.2)

FLAGS = {0x0001: "SLEW_LIMITED", 0x0002: "TRACK_ERROR", 0x0004: "BELOW_OBS_MIN",
         0x0008: "MOTOR_FAULT", 0x0010: "TICK_OVERRUN", 0x0020: "VBUS_DERATE",
         0x0040: "FLOW_INVALID", 0x0080: "FLOW_SAT", 0x0100: "PRESSURE_LIMIT",
         0x0200: "VOLUME_SHORT", 0x0400: "FLOW_NO_RESPONSE",
         0x0800: "FF_NO_PRESSURE"}


def decode(f):
    hit = [n for b, n in sorted(FLAGS.items()) if f & b]
    return "|".join(hit) if hit else "none"


def send(c):
    ser.write((c + "\r\n").encode("ascii"))
    ser.flush()
    time.sleep(0.12)


def collect(seconds, want_v=False):
    """Return (V rows, malformed count, total lines, last flags)."""
    end = time.monotonic() + seconds
    vs, bad, total, flags = [], 0, 0, 0
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", errors="replace").strip()
        if not t:
            continue
        total += 1
        # A spliced line has a second record start inside it.
        body = t[2:] if len(t) > 2 else ""
        if any(m in body for m in ("B,", "V,", "E,", "S,", "Q,", "M,")) and \
           t[:2] in ("B,", "V,", "E,", "S,", "Q,", "M,"):
            bad += 1
            continue
        if t.startswith("B,"):
            p = t.split(",")
            if len(p) >= 6:
                try:
                    flags = int(p[5], 16)
                except ValueError:
                    bad += 1
        elif t.startswith("V,"):
            p = t.split(",")
            if len(p) < 13:
                bad += 1
                continue
            try:
                vs.append(dict(breath=int(p[1]), tidal=float(p[2]),
                               qpk_cmd=float(p[6]), flags=int(p[7], 16),
                               qpk_meas=float(p[9]), ti=float(p[10]),
                               flat=float(p[11]), event=p[12]))
            except ValueError:
                bad += 1
    return vs, bad, total, flags


def _rpm_act_now(window=0.5):
    """Latest measured rpm from the B line, or None."""
    end = time.monotonic() + window
    act = None
    while time.monotonic() < end:
        raw = ser.readline()
        if not raw:
            continue
        t = raw.decode("ascii", errors="replace").strip()
        if t.startswith("B,"):
            p = t.split(",")
            if len(p) >= 8:
                try:
                    act = int(p[7])
                except ValueError:
                    pass
    return act


def ramp_down(timeout=15.0, slow_rpm=10000):
    """Bring the blower down under control, VERIFIED, and only then stop.

    The previous version set amplitude 0 and stepped rpm_base. In FLOW mode
    amplitude is ignored, so it did nothing: every flow-mode run before this
    fix was stopped at whatever point the breath had reached - up to ~31000
    rpm mid-inspiration - which is exactly what the bench rule forbids.

    A commanded apnea drives the command to rpm_base in BOTH modes (verified
    2026-10-09 after the integrator fix), so use that, and do not send stop
    until the motor is MEASURED below slow_rpm for a full second.
    """
    send("sim set amplitude 0")
    send("sim event apnea 120 100")      # latches at the next boundary
    end = time.monotonic() + timeout
    slow_since = None
    act = None
    while time.monotonic() < end:
        act = _rpm_act_now(0.25)
        if act is None:
            continue
        if act < slow_rpm:
            if slow_since is None:
                slow_since = time.monotonic()
            elif time.monotonic() - slow_since >= 1.0:
                break
        else:
            slow_since = None
    for r in (8000, 6000):
        send("sim set rpm_base %d" % r)
        time.sleep(2.0)
    act = _rpm_act_now(0.6)
    send("sim stop")
    time.sleep(1.0)
    print("  [ramp-down] stopped at measured %s rpm"
          % (act if act is not None else "?"))
    return act


def run(label, setup, settle=26.0, capture=40.0):
    print("\n" + "=" * 70)
    print(label)
    print("=" * 70)
    for c in setup:
        send(c)
    send("sim start")
    print("  priming (rev-up + any coast hold-off)...")
    collect(settle)
    vs, bad, total, flags = collect(capture, want_v=True)
    print("  lines %d, malformed %d, V rows %d, flags %s"
          % (total, bad, len(vs), decode(flags)))
    if not vs:
        print("  !! no V rows - nothing to report")
        return None
    # drop the first, it starts from a standing blower
    use = vs[1:] if len(vs) > 1 else vs
    print("\n  %-7s %8s %9s %7s %7s  %s"
          % ("breath", "Vt mL", "Qpk slm", "Ti s", "flat", "flags"))
    for v in use:
        print("  %-7d %8.1f %9.1f %7.3f %7.3f  %s"
              % (v["breath"], v["tidal"], v["qpk_meas"], v["ti"], v["flat"],
                 decode(v["flags"])))
    med = statistics.median([v["flat"] for v in use])
    print("\n  median flat_index = %.3f   (unflattened half-sine = 0.637)" % med)
    print("  median Vt         = %.1f mL" % statistics.median([v["tidal"] for v in use]))
    print("  median peak flow  = %.1f slm" % statistics.median([v["qpk_meas"] for v in use]))
    return med


if __name__ == "__main__":
    try:
        send("log 0")                 # silences the second CDC producer
        send("telem on")
        send("sim stop")
        time.sleep(1.0)
        ser.reset_input_buffer()

        send("flow on")
        send("press on")
        send("flow clearzero")
        time.sleep(2.0)
        ser.reset_input_buffer()
        time.sleep(1.5)
        # tare only if the air really is still
        f = None
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            t = ser.readline().decode("ascii", errors="replace").strip()
            if t.startswith("F,"):
                p = t.split(",")
                if len(p) >= 5:
                    try:
                        f = float(p[4])
                    except ValueError:
                        pass
        print("at rest, blower stopped: %s slm" % ("%.2f" % f if f is not None else "??"))
        if f is not None and abs(f) < 2.0:
            send("flow zero")
            print("  tared")
        else:
            print("  NOT tared - air is moving or no samples")

        base = ["sim set control openrpm", "sim set waveform sine",
                "sim set timing rate_ie", "sim set rate 12", "sim set ie 2",
                "sim set insp_pause 0", "sim set exp_pause 0",
                "sim set jitter 0", "sim set flattening 0",
                "sim set fl_shape plateau", "sim set recovery 0",
                "sim set snore_hz 0", "sim set exp_tau 0.4",
                "sim set rpm_base 8000", "sim set amplitude 10000"]

        run("OPEN LOOP  rpm_base 8000 + amplitude 10000, 12 bpm, I:E 1:2", base)
        ramp_down()

    finally:
        try:
            ramp_down()
            send("log 3")
        except Exception:
            pass
        ser.close()
        print("\nblower ramped down and stopped.")
