"""Sensor liveness: are the flow and pressure read counters actually moving?

`flow status` reports meas=1 even after SensorTask has stopped reading - it
stayed set through a ~35 min hang on 2026-10-09 while the read counter sat
frozen. The only trustworthy test is that `reads=` advances between two
looks. Used before a run (refuse to start on a dead feed) and after it (mark
the run void rather than report numbers built on stale samples).

Telemetry is switched off around the check. If the host has not read the
port for a while the device's TX ring fills, and the status reply is then
dropped as a whole line - which the first version reported as a dead sensor
loop when it was only an unread port (2026-10-09). "No reply" and "frozen"
are reported as the two different things they are.
"""
import re
import time


def _counter(ser, cmd, pattern, wait=2.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    end = time.monotonic() + wait
    while time.monotonic() < end:
        t = ser.readline().decode("ascii", "replace").strip()
        m = re.search(pattern, t)
        if m:
            return int(m.group(1))
    return None


def counters(ser):
    f = _counter(ser, "flow status", r"# flow reads=(\d+)")
    p = _counter(ser, "press status", r"# press PS reads=(\d+)")
    return f, p


def check(ser, label, gap=1.0):
    """True if both counters advanced over `gap` seconds. Prints why not."""
    ser.write(b"telem off\r\n")
    ser.flush()
    time.sleep(1.0)                  # let the device's backlog drain
    ser.reset_input_buffer()
    f0, p0 = counters(ser)
    time.sleep(gap)
    f1, p1 = counters(ser)
    ser.write(b"telem on\r\n")
    ser.flush()

    if None in (f0, p0, f1, p1):
        verdict = "!! NO REPLY to status (port/link problem, not proven a hang)"
        ok = False
    elif (f1 > f0) and (p1 > p0):
        verdict = "OK"
        ok = True
    elif (f1 <= f0) and (p1 > p0):
        # The loop is running - pressure advances - but no flow sample is
        # being taken: the SFM3300 itself is down (e.g. not answering on
        # I2C after a boot), not SensorTask. 'flow status' will say why.
        verdict = "!! FLOW SENSOR NOT READING (loop running) - see 'flow status'"
        ok = False
    else:
        verdict = "!! COUNTERS FROZEN - sensor loop not running"
        ok = False
    print("  liveness %-6s flow %s -> %s   press %s -> %s   %s"
          % (label, f0, f1, p0, p1, verdict))
    return ok
