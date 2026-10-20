"""Count CPAP-side flow needles in a trial's T log: rise >= 4 slm in 40 ms that
falls back by half within 0.4 s, outside FOT; test them against a 17.75 s grid."""
import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
import pickle, sys, bisect
import numpy as np
sys.path.insert(0, HERE)
from spike_analysis import detect

def main(path):
    d = pickle.load(open(path, "rb"))
    off = d["t_script"] - d["t0"]
    T = np.array([(r[1] / 1000.0, r[3], r[0] - off) for r in d["t_log"]])
    tick, q = T[:, 0], T[:, 1]
    to_script = np.median(T[:, 2] - tick)
    sts = sorted((x["t"] - off, x["osc"]) for x in d["st_log"])
    st_t = [x[0] for x in sts]
    def osc(t):   # S lines ~1-2 s late; treat +-3 s around FOT as FOT
        return any(sts[max(0, bisect.bisect_right(st_t, u) - 1)][1] for u in (t - 3, t, t + 3))
    ev = [e for e in detect(tick, q) if not osc(e["t"] + to_script)]
    span = (tick[-1] - tick[0]) / 60.0
    print("%s: %d needles outside FOT in %.1f min (%.2f/min)" % (path, len(ev), span, len(ev) / span))
    if len(ev) >= 3:
        t = np.array([e["t"] for e in ev])
        k = np.round((t - t[0]) / 17.75)
        r = t - t[0] - k * 17.75
        print("  on a 17.75 s grid: rms %.2f s (random would be ~5 s)" % np.sqrt(np.mean(r ** 2)))
        print("  amplitudes: %s" % ", ".join("%.1f" % e["amp"] for e in ev[:12]))

for p in sys.argv[1:]:
    main(p)
