"""Per hypopnea event: how deep does the CPAP see it, and what did it score?

For each event (simulator V-line labels), the CPAP-seen peak of every weak
breath (max of the T-line patient flow over that breath) is divided by the
median CPAP-seen peak of the normal breaths in the 30 s before the event.
That ratio is what the APAP's Hyp rule (< 0.50) and FL rule (< 0.70) compare,
up to its own baseline filter. Next to it: the simulator's delivered ratio,
and the Hyp / FL counts that rose during the event (+ 20 s).

usage: python hyp_ratio.py <raw.pkl>
"""
import pickle
import statistics as S
import sys

import numpy as np


def main(path):
    d = pickle.load(open(path, "rb"))
    off = d["t_script"] - d["t0"]
    T = np.array([(r[0] - off, r[3]) for r in d["t_log"]])
    V = [(r["t"], r["event"], r["qpk"], r["vt"]) for r in d["sim_rows"]]
    aps = sorted(({**a, "ts": a["t"] - off} for a in d["ap_log"]), key=lambda a: a["ts"])

    def count_at(key, ts):
        v = 0
        for a in aps:
            if a["ts"] <= ts:
                v = a[key]
        return v

    breaths = []
    for (t0, _, _, _), (t1, ev, qpk, vt) in zip(V[:-1], V[1:]):
        w = T[(T[:, 0] >= t0) & (T[:, 0] < t1)][:, 1]
        if len(w):
            breaths.append(dict(t0=t0, t1=t1, ev=ev, cp=float(w.max()), sim=qpk, vt=vt))

    events, cur = [], None
    for b in breaths:
        if b["ev"] == "Hypopnea":
            if cur is None or b["t0"] - cur[-1]["t1"] > 1.0:
                cur = []
                events.append(cur)
            cur.append(b)
        else:
            cur = None

    seg_sev = [sev for ev, dur, sev in d["script"] if ev == "hypopnea"]
    print("  %-4s %-7s %5s | %-24s | %-24s | %s"
          % ("#", "level", "Vt", "CPAP-seen peak / normal", "sim peak / normal", "scored during event"))
    for i, ev in enumerate(events):
        a, z = ev[0]["t0"], ev[-1]["t1"]
        norm = [b for b in breaths if b["ev"] == "None" and a - 30.0 <= b["t0"] < a]
        if not norm:
            continue
        ncp = S.median(b["cp"] for b in norm)
        nsim = S.median(b["sim"] for b in norm)
        r_cp = [b["cp"] / ncp for b in ev]
        r_sim = [b["sim"] / nsim for b in ev]
        dh = count_at("Hyp", z + 20.0) - count_at("Hyp", a)
        dfl = count_at("FL", z + 20.0) - count_at("FL", a)
        dca = count_at("CA", z + 20.0) - count_at("CA", a)
        below = sum(1 for r in r_cp if r < 0.50)
        lvl = "%d %%" % seg_sev[i] if i < len(seg_sev) else "?"
        print("  %-4d %-7s %5.0f | median %3.0f %%  (%3.0f-%3.0f) | median %3.0f %%  (%3.0f-%3.0f) | Hyp %d  FL %d  CA %d   [%d of %d breaths < 50 %%]"
              % (i + 1, lvl, S.median(b["vt"] for b in ev),
                 100 * S.median(r_cp), 100 * min(r_cp), 100 * max(r_cp),
                 100 * S.median(r_sim), 100 * min(r_sim), 100 * max(r_sim),
                 dh, dfl, dca, below, len(ev)))


if __name__ == "__main__":
    main(sys.argv[1])
