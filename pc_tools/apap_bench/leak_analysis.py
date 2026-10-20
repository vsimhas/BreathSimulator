"""Analysis for leak_source.py (R9): per simulator breath, compare the real
box leak (external - simulator lung flow) with the CPAP's patient-flow mean.

Clock alignment as in spike_analysis: device ticks -> host by the early-edge
offset, external by a chunked index fit, then CPAP and simulator refined
against the external by cross-correlating breathing flow.
Breaths are split at the simulator's own inspiration starts (lung flow rising
through +2 slm after >= 0.5 s below +0.5 slm); each is labelled by the V line
that closes it (falling back to the script segment).
"""
import statistics
import numpy as np
from spike_analysis import dev_to_host, ext_times, xcorr_lag

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
def insp_starts(t, q):
    out, quiet_since = [], None
    for i in range(1, len(t)):
        if q[i] < 0.5:
            if quiet_since is None:
                quiet_since = t[i]
        elif q[i] >= 2.0 and q[i - 1] < 2.0:
            if quiet_since is not None and t[i] - quiet_since >= 0.5:
                out.append(t[i])
            quiet_since = None
    return out


def main(raw):
    ts0 = raw["t_script"]
    segs, acc = [], 0.0
    for ev, dur, sev in raw["script"]:
        segs.append((acc, acc + dur, ev if sev is None else "%s %d%%" % (ev, sev)))
        acc += dur

    def seg(x):
        for a, b, n in segs:
            if a <= x < b:
                return n
        return "after"

    T = np.array([(r[0], r[1], r[3]) for r in raw["t_log"]], dtype=float)
    F = np.array([(r[0], r[1], r[2], r[4]) for r in raw["sim_f"]], dtype=float)
    E = np.array(raw["ext"], dtype=float)
    t_c = dev_to_host(T[:, 0], T[:, 1] / 1000.0) - ts0
    q_c = T[:, 2]
    ok = F[:, 3] > 0
    t_s = dev_to_host(F[ok, 0], F[ok, 1] / 1000.0) - ts0
    q_s = F[ok, 2]
    t_e = ext_times(E[:, 0]) - ts0
    q_e, p_e = E[:, 2], E[:, 1]

    breathing = [(a + 3, b - 3) for a, b, n in segs if n == "none"]
    lag_c, sgn_c, r_c = xcorr_lag(t_c, q_c, t_e, q_e, breathing)
    lag_s, sgn_s, r_s = xcorr_lag(t_s, q_s, t_e, q_e, breathing)
    t_c, t_s = t_c + lag_c, t_s + lag_s
    q_e = q_e * sgn_c
    if sgn_s != sgn_c:
        q_s = -q_s
    print("\n" + "=" * 78)
    print("LEAK SOURCE - per-breath means on one clock")
    print("=" * 78)
    print("  alignment to external: CPAP %+.2f s (r %.3f%s), simulator %+.2f s (r %.3f)"
          % (lag_c, r_c, ", ext sign flipped" if sgn_c < 0 else "", lag_s, r_s))

    starts = insp_starts(t_s, q_s)
    v = [(x[0] - ts0 + lag_s, x[1], x[2], x[3], x[4]) for x in raw.get("sim_v", [])]
    rows = []
    for a, b in zip(starts[:-1], starts[1:]):
        if not (2.0 < b - a < 8.0):
            continue
        def win(t, q):
            sel = (t >= a) & (t < b)
            return q[sel]
        s_, e_, c_ = win(t_s, q_s), win(t_e, q_e), win(t_c, q_c)
        if min(len(s_), len(e_), len(c_)) < 20:
            continue
        lab = [x for x in v if b - 0.5 <= x[0] <= b + 2.0]
        sname = seg(a)
        label = lab[0][4] if lab else ("None" if sname == "none" else
                                      sname.split()[0].capitalize())
        vt = lab[0][2] if lab else float("nan")
        e_mean = float(e_.mean())
        rows.append(dict(t=a, label=label, vt=vt,
                         sim_mean=float(s_.mean()), sim_pk=float(s_.max()),
                         ext_mean=e_mean, ext_pk=float(e_.max()) - e_mean,
                         leak=e_mean - float(s_.mean()),
                         cp_mean=float(c_.mean()), cp_pk=float(c_.max())))
    print("  %d breaths (simulator inspiration starts)" % len(rows))

    order = ["None", "Hypopnea"]
    print("\n  per segment (medians over its breaths):")
    print("    %-22s %3s %6s | %7s %7s | %8s | %7s %7s %7s"
          % ("segment", "n", "sim Vt", "sim pk", "ext pk", "BOX LEAK", "CPAP mn", "CPAP pk", "pk ratio"))
    print("    %-22s %3s %6s | %7s %7s | %8s | %7s %7s %7s"
          % ("", "", "mL", "slm", "-mean", "ext-sim", "slm", "slm", "CPAP/sim"))
    summary = []
    for a, b, name in segs:
        sel = [r for r in rows if a <= r["t"] < b + 6.0 and
               ((name == "none" and r["label"] == "None") or
                (name != "none" and r["label"] == "Hypopnea"))]
        if not sel:
            continue
        med = lambda k: statistics.median(r[k] for r in sel)
        line = dict(name="%s %.0f-%.0f s" % (name, a, b), n=len(sel), vt=med("vt"),
                    sim_pk=med("sim_pk"), ext_pk=med("ext_pk"), leak=med("leak"),
                    cp_mean=med("cp_mean"), cp_pk=med("cp_pk"))
        line["ratio"] = line["cp_pk"] / line["sim_pk"] if line["sim_pk"] else float("nan")
        summary.append(line)
        print("    %-22s %3d %6.0f | %7.1f %7.1f | %8.2f | %+7.2f %7.1f %7.2f"
              % (line["name"], line["n"], line["vt"], line["sim_pk"], line["ext_pk"],
                 line["leak"], line["cp_mean"], line["cp_pk"], line["ratio"]))

    norm = [x for x in summary if x["name"].startswith("none")]
    hyp = [x for x in summary if x["name"].startswith("hypopnea")]
    if norm and hyp:
        dleak = statistics.mean(h["leak"] for h in hyp) - statistics.mean(n["leak"] for n in norm)
        dcp = statistics.mean(h["cp_mean"] for h in hyp) - statistics.mean(n["cp_mean"] for n in norm)
        print("\n  hypopnea minus normal:  box leak %+.2f slm   CPAP patient-flow mean %+.2f slm"
              % (dleak, dcp))
        if abs(dleak) < 1.0 and dcp < -1.5:
            v_ = ("CPAP LEAK ESTIMATE - the real box leak is flat but the CPAP's patient "
                  "flow drops: the CPAP over-reads leak when breaths shrink (R9 cause 1)")
        elif dleak < -1.5 and abs(dcp - dleak) < 1.0:
            v_ = ("RIG - the box leak itself drops in the hypopnea and the CPAP reads it "
                  "correctly (R9 cause 2)")
        elif dcp > -1.0:
            v_ = "NO OFFSET this run (CPAP mean did not drop) - inconclusive"
        else:
            v_ = "MIXED - box leak changes %+.1f, CPAP %+.1f; see the table" % (dleak, dcp)
        print("  VERDICT: %s" % v_)

    with open(SP + "/leak_breaths.csv", "w") as fh:
        fh.write("t_s,label,sim_vt,sim_mean,sim_pk,ext_mean,ext_pk,box_leak,cpap_mean,cpap_pk\n")
        for r in rows:
            fh.write("%.2f,%s,%.0f,%.3f,%.2f,%.3f,%.2f,%.3f,%.3f,%.2f\n"
                     % (r["t"], r["label"], r["vt"], r["sim_mean"], r["sim_pk"], r["ext_mean"],
                        r["ext_pk"], r["leak"], r["cp_mean"], r["cp_pk"]))
    plot(rows)
    print("  per breath: leak_breaths.csv   plot: leak_source.png")


def plot(rows):
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(1, 1, figsize=(10, 3.6))
    ax.plot(t, [r["leak"] for r in rows], color="#eb6834", lw=2, marker="o", ms=4,
            label="box leak (external - simulator)")
    ax.plot(t, [r["cp_mean"] for r in rows], color="#2a78d6", lw=2, marker="o", ms=4,
            label="CPAP patient-flow mean")
    for r in rows:
        if r["label"] == "Hypopnea":
            ax.axvspan(r["t"], r["t"] + 5.0, color="#eeeeee", zorder=0, lw=0)
    ax.axhline(0, color="#bbbbbb", lw=1, zorder=0)
    ax.grid(True, color="#e6e6e6", lw=0.6)
    ax.set_xlabel("s from script start (grey = hypopnea breaths)", fontsize=9, color="#555555")
    ax.set_ylabel("slm, per-breath mean", fontsize=9, color="#555555")
    ax.tick_params(labelsize=8, colors="#555555")
    for s in ax.spines.values():
        s.set_color("#cccccc")
    ax.legend(fontsize=8, frameon=False, loc="best")
    ax.set_title("Real box leak vs the CPAP's patient-flow mean, per breath",
                 fontsize=11, color="#222222")
    fig.tight_layout()
    fig.savefig(SP + "/leak_source.png", dpi=110)
    plt.close(fig)
