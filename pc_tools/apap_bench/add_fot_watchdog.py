"""Add the FOT watchdog to a trial harness (default apap_trial8.py).

The supply-current guard (> 0.8 A held 0.7 s) cannot see FOT: its current is
spiky, so trial 8 oscillated ~14 min with > 0.8 A in 1010 samples and never
tripped. The watchdog times each continuous oscillation run from the CPAP's
S lines and aborts the run (verified simulator ramp-down, CPAP setpoint 4 ->
stop) once one exceeds FOT_MAX_S. Longest legitimate FOT so far: ~17 s
(20 s apnea).
"""
import ast
import io
import sys

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
path = HERE + "/" + (sys.argv[1] if len(sys.argv) > 1 else "apap_trial8.py")
s = io.open(path, encoding="utf-8").read()
if "FOT_MAX_S" in s:
    raise SystemExit("already has the watchdog")

old = "I_LIMIT, I_HOLD_S, VBUS_MIN = 0.8, 0.7, 21.0\n"
assert s.count(old) == 1
s = s.replace(old, old + "FOT_MAX_S = 60.0   # watchdog: continuous APAP oscillation longer than this aborts\n")

old = "        self.i_max = 0.0\n"
assert s.count(old) == 1
s = s.replace(old, old + "        self.osc_since = None     # host time the current FOT run started\n"
                         "        self.osc_max_s = 0.0      # longest continuous FOT run seen\n")

old = '''                        if st["state"] and st["maskoff"] and not self.abort:'''
assert s.count(old) == 1
s = s.replace(old, '''                        # FOT watchdog: the current guard cannot see FOT's spiky
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
''' + old)

old = '''    print("  peak CPAP supply current %.2f A   simulator sensors %s"'''
assert s.count(old) == 1
s = s.replace(old, '''    print("  longest continuous FOT %.0f s (watchdog %.0f s)" % (cp.osc_max_s, FOT_MAX_S))
''' + old)

old = "Guards: CPAP supply current > 0.8 A for > 0.7 s, Vbus < 21 V, drive fault,"
assert s.count(old) == 1
s = s.replace(old, "Guards: FOT watchdog (APAP oscillating continuously > 60 s - the current\n"
                   "guard cannot see FOT's spiky current), CPAP supply current > 0.8 A for\n"
                   "> 0.7 s, Vbus < 21 V, drive fault,")

ast.parse(s)
io.open(path, "w", encoding="utf-8").write(s)
print("watchdog added to", path)
