"""Analysis for total_source.py (R10): per simulator breath, split the CPAP's
patient-flow error into its flow MEASUREMENT (CPAP total vs external) and the
MODEL it subtracts (CPAP total - CPAP patient).

Air path: CPAP sensor -> hose -> VENT -> external sensor -> chamber -> box
(leak) -> simulator sensor -> lung. So:
  external - simulator      = real box leak
  CPAP total - external     = vent flow as the CPAP's sensor sees it
  CPAP total - CPAP patient = vent + leak, as the CPAP's model has it
The B line (total) and T line (patient) share the CPAP's tick, so they are put
on the host clock with the same offset and the same cross-correlation lag.
"""
import statistics
import numpy as np
from spike_analysis import ext_times, xcorr_lag
from leak_analysis import insp_starts

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
def main(raw):
    ts0 = raw["t_script"]
    segs, acc = [], 0.0
    for ev, dur, sev in raw["script"]:
        segs.append((acc, acc + dur, ev if sev is None else "%s %d%%" % (ev, sev)))
        acc += dur

    T = np.array([(r[0], r[1], r[3]) for r in raw["t_log"]], dtype=float)
    B = np.array([(r[0], r[1], r[2]) for r in raw["bt_log"]], dtype=float)
    F = np.array([(r[0], r[1], r[2], r[4]) for r in raw["sim_f"]], dtype=float)
    E = np.array(raw["ext"], dtype=float)
    off_c = np.percentile(T[:, 0] - T[:, 1] / 1000.0, 2)       # CPAP tick -> host
    t_c = T[:, 1] / 1000.0 + off_c - ts0
    q_c = T[:, 2]
    t_b = B[:, 1] / 1000.0 + off_c - ts0
    q_b = B[:, 2]
    ok = F[:, 3] > 0
    t_s = F[ok, 1] / 1000.0 + np.percentile(F[ok, 0] - F[ok, 1] / 1000.0, 2) - ts0
    q_s = F[ok, 2]
    t_e = ext_times(E[:, 0]) - ts0
    q_e = E[:, 2]

    breathing = [(a + 3, b - 3) for a, b, n in segs if n == "none"]
    lag_c, sgn_c, r_c = xcorr_lag(t_c, q_c, t_e, q_e, breathing)
    lag_s, sgn_s, r_s = xcorr_lag(t_s, q_s, t_e, q_e, breathing)
    t_c, t_b, t_s = t_c + lag_c, t_b + lag_c, t_s + lag_s
    q_e = q_e * sgn_c
    if sgn_s != sgn_c:
        q_s = -q_s
    print("\n" + "=" * 78)
    print("TOTAL-FLOW CAPTURE - measurement vs model, per breath")
    print("=" * 78)
    print("  CPAP T %d, CPAP B (total) %d (%.1f Hz), sim F %d, ext %d"
          % (len(T), len(B), (len(B) - 1) / (t_b[-1] - t_b[0]), len(F), len(E)))
    print("  alignment to external: CPAP %+.2f s (r %.3f%s), simulator %+.2f s (r %.3f)"
          % (lag_c, r_c, ", ext sign flipped" if sgn_c < 0 else "", lag_s, r_s))

    v = [(x[0] - ts0 + lag_s, x[4]) for x in raw.get("sim_v", [])]
    starts = insp_starts(t_s, q_s)
    rows = []
    for a, b in zip(starts[:-1], starts[1:]):
        if not (2.0 < b - a < 8.0):
            continue
        w = lambda t, q: q[(t >= a) & (t < b)]
        s_, e_, c_, b_ = w(t_s, q_s), w(t_e, q_e), w(t_c, q_c), w(t_b, q_b)
        if min(len(s_), len(e_), len(c_), len(b_)) < 20:
            continue
        lab = [x for x in v if b - 0.5 <= x[0] <= b + 2.0]
        # inspiratory peaks above each stream's own breath mean
        rows.append(dict(
            t=a, label=lab[0][1] if lab else "?",
            sim=float(s_.mean()), ext=float(e_.mean()), tot=float(b_.mean()),
            pat=float(c_.mean()),
            sim_pk=float(s_.max()),
            ext_pk=float(e_.max() - e_.mean()),
            tot_pk=float(b_.max() - b_.mean()),
            pat_pk=float(c_.max())))
    for r in rows:
        r["box_leak"] = r["ext"] - r["sim"]
        r["vent_seen"] = r["tot"] - r["ext"]          # measurement check
        r["model"] = r["tot"] - r["pat"]              # what the CPAP subtracts
    print("  %d breaths" % len(rows))

    print("\n  per segment (medians):")
    hdr = ("segment", "n", "box leak", "tot-ext", "tot-pat", "pat mean", "sim pk", "ext pk", "tot pk", "pat pk")
    print("    %-22s %3s | %8s | %8s %8s | %8s | %6s %6s %6s %6s" % hdr)
    print("    %-22s %3s | %8s | %8s %8s | %8s | %6s %6s %6s %6s"
          % ("", "", "ext-sim", "VENT as", "MODEL", "", "", "-mean", "-mean", ""))
    print("    %-22s %3s | %8s | %8s %8s | %8s |" % ("", "", "(real)", "measured", "subtr.", ""))
    summ = []
    for a, b, name in segs:
        want = "None" if name == "none" else "Hypopnea"
        sel = [r for r in rows if a <= r["t"] < b + 6.0 and r["label"] == want]
        if not sel:
            continue
        m = {k: statistics.median(r[k] for r in sel) for k in
             ("box_leak", "vent_seen", "model", "pat", "sim_pk", "ext_pk", "tot_pk", "pat_pk")}
        m["name"], m["n"] = "%s %.0f-%.0f s" % (name, a, b), len(sel)
        summ.append(m)
        print("    %-22s %3d | %8.2f | %8.2f %8.2f | %+8.2f | %6.1f %6.1f %6.1f %6.1f"
              % (m["name"], m["n"], m["box_leak"], m["vent_seen"], m["model"], m["pat"],
                 m["sim_pk"], m["ext_pk"], m["tot_pk"], m["pat_pk"]))

    norm = [m for m in summ if m["name"].startswith("none")]
    hyp = [m for m in summ if m["name"].startswith("hypopnea")]
    if norm and hyp:
        d = lambda k: (statistics.mean(h[k] for h in hyp) - statistics.mean(n[k] for n in norm))
        print("\n  hypopnea minus normal:")
        print("    real box leak            %+.2f slm" % d("box_leak"))
        print("    vent as measured         %+.2f slm   (CPAP total - external)" % d("vent_seen"))
        print("    model subtracted         %+.2f slm   (CPAP total - CPAP patient)" % d("model"))
        print("    CPAP patient-flow mean   %+.2f slm" % d("pat"))
        meas, model, real = d("vent_seen"), d("model"), d("box_leak")
        # If the sensor reads right, vent_seen stays flat; the model should then
        # move by the real leak change. The excess is the model's error.
        print("    -> measurement error  ~%+.2f slm;  model error ~%+.2f slm (model change "
              "minus real leak change)" % (meas, model - real))
        if abs(meas) < 0.5 and (model - real) > 1.0:
            v_ = "MODEL - the CPAP measures total flow correctly; its subtracted leak/vent model rises in weak breathing"
        elif abs(meas) >= 1.0 and abs(model - real) < 0.5:
            v_ = "MEASUREMENT - the CPAP's total flow itself is off in weak breathing; the model is not"
        else:
            v_ = "MIXED - measurement %+.1f, model %+.1f; see the table" % (meas, model - real)
        print("  VERDICT: %s" % v_)

    with open(SP + "/total_breaths.csv", "w") as fh:
        keys = ["t", "label", "sim", "ext", "tot", "pat", "box_leak", "vent_seen", "model",
                "sim_pk", "ext_pk", "tot_pk", "pat_pk"]
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(str(round(r[k], 3)) if isinstance(r[k], float) else str(r[k])
                              for k in keys) + "\n")
    plot(rows)
    print("  per breath: total_breaths.csv   plot: total_source.png")


def plot(rows):
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rows]
    fig, ax = plt.subplots(1, 1, figsize=(10, 3.8))
    ax.plot(t, [r["vent_seen"] for r in rows], color="#2a78d6", lw=2, marker="o", ms=4,
            label="vent as the CPAP measures it (CPAP total - external)")
    ax.plot(t, [r["model"] - r["box_leak"] for r in rows], color="#eb6834", lw=2, marker="o", ms=4,
            label="vent as the CPAP's model has it (total - patient - real box leak)")
    for r in rows:
        if r["label"] == "Hypopnea":
            ax.axvspan(r["t"], r["t"] + 5.0, color="#eeeeee", zorder=0, lw=0)
    ax.grid(True, color="#e6e6e6", lw=0.6)
    ax.set_xlabel("s from script start (grey = hypopnea breaths)", fontsize=9, color="#555555")
    ax.set_ylabel("slm, per-breath mean", fontsize=9, color="#555555")
    ax.tick_params(labelsize=8, colors="#555555")
    for s in ax.spines.values():
        s.set_color("#cccccc")
    ax.legend(fontsize=8, frameon=False, loc="best")
    ax.set_title("Vent flow: measured by the CPAP's sensor vs implied by its model",
                 fontsize=11, color="#222222")
    fig.tight_layout()
    fig.savefig(SP + "/total_source.png", dpi=110)
    plt.close(fig)
