"""Per flattened-breath block on OUR CPAP, in the same shape as the AirSense
comparison: delivered flatness and peak, what the APAP scored (FL / Hyp / CA
counter rises, FL flag on-time), and its pressure target at onset -> end ->
end + 60 s, plus the fastest rise and fall rates of the target.

usage: python cpap_fl_report.py <apap_flshape raw.pkl>
"""
import pickle
import statistics as S
import sys

import numpy as np


def main(path):
    d = pickle.load(open(path, "rb"))
    off = d["t_script"] - d["t0"]
    sts = sorted(({**x, "ts": x["t"] - off} for x in d["st_log"]), key=lambda x: x["ts"])
    aps = sorted(({**a, "ts": a["t"] - off} for a in d["ap_log"]), key=lambda a: a["ts"])
    rows = d["sim_rows"]
    shapes = list(d.get("fl_shapes", []))
    sevs = [sev for ev, dur, sev in d["script"] if ev == "flowlimit"]

    blocks, cur, prev_t = [], None, None
    for r in rows:
        if r["event"] == "FlowLimit":
            if cur is None:
                cur = dict(onset=prev_t if prev_t is not None else r["t"] - 5.0, rows=[])
                blocks.append(cur)
            cur["end"] = r["t"]
            cur["rows"].append(r)
        else:
            cur = None
        prev_t = r["t"]
    norm = [r for r in rows if r["event"] == "None"]
    npk = S.median(r["qpk"] for r in norm)
    nflat = S.median(r["flat"] for r in norm)

    st_t = np.array([x["ts"] for x in sts])
    tgt = np.array([x["target"] for x in sts])

    def at(key, ts):
        v = 0
        for a in aps:
            if a["ts"] <= ts:
                v = a[key]
        return v

    print("  normal breaths: flatness %.2f, peak %.1f slm; APAP flat rule > 0.78, FL peak rule < 0.70"
          % (nflat, npk))
    print("  %-14s %5s %5s | %-26s | %s" % ("block", "flat", "peak", "scored (block + 20 s)",
                                           "our pressure target: onset -> end -> end+60 s (max)"))
    for i, b in enumerate(blocks):
        a, z = b["onset"], b["end"]
        lab = "%s %d %%" % (shapes[i] if i < len(shapes) else "FL", sevs[i] if i < len(sevs) else 0)
        flat = S.median(r["flat"] for r in b["rows"])
        pk = S.median(r["qpk"] for r in b["rows"]) / npk
        got = []
        for k in ("FL", "Hyp", "CA", "OA"):
            dlt = at(k, z + 20.0) - at(k, a - 1.0)
            if dlt:
                got.append("%s %d" % (k, dlt))
        fl_on = [x for x in aps if a <= x["ts"] <= z]
        frac = (sum(1 for x in fl_on if x["fl"]) / len(fl_on)) if fl_on else float("nan")
        sel = (st_t >= a) & (st_t <= z + 60.0)
        print("  %-14s %5.2f %4.0f%% | %-26s | %.1f -> %.1f -> %.1f  (%.1f)"
              % (lab, flat, 100 * pk,
                 (", ".join(got) or "nothing") + ("; FL flag %.0f%%" % (100 * frac) if fl_on else ""),
                 np.interp(a, st_t, tgt), np.interp(z, st_t, tgt), np.interp(z + 60.0, st_t, tgt),
                 tgt[sel].max() if sel.any() else float("nan")))
    if len(st_t) > 10:
        g = np.arange(max(st_t[0], 0.0), st_t[-1], 5.0)
        tg = np.interp(g, st_t, tgt)
        slope = np.diff(tg) / 5.0 * 60.0          # cmH2O per minute over 5 s steps
        print("  pressure target over the run %.1f .. %.1f cmH2O; fastest rise %.1f, fastest fall %.1f cmH2O/min"
              % (tgt.min(), tgt.max(), slope.max(), slope.min()))
    tot = aps[-1] if aps else None
    if tot:
        print("  totals: OA %d CA %d Hyp %d FL %d" % (tot["OA"], tot["CA"], tot["Hyp"], tot["FL"]))


if __name__ == "__main__":
    main(sys.argv[1])
