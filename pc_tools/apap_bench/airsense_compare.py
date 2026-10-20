#!/usr/bin/env python3
"""Compare an AirSense 11 reference run with the simulator's ground truth and
(optionally) our CPAP's scoring of the same script.

usage:
  python airsense_compare.py <airsense_raw.pkl> <DATALOG date folder or files...>
                             [--ours <apap_trial raw.pkl>]

Clock alignment, AirSense device clock -> PC clock:
  coarse  1 Hz breathing-amplitude envelopes (AirSense BRP flow vs external
          sensor flow), cross-correlated over +/-60 min
  fine    raw flow at 25 Hz, +/-2 s around the coarse lag
Ground truth: the simulator's V lines (one per breath, labelled with the
event it belonged to). Each event's onset is the start of its first
affected breath. AirSense annotations whose onset falls between onset - 5 s
and end + 30 s are attributed to it; anything else in the script window is
listed as unattributed.
"""
import datetime as dt
import glob
import os
import pickle
import statistics as S
import sys

import numpy as np

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
sys.path.insert(0, HERE)
from airsense_edf import read_edf
from spike_analysis import ext_times

EV_SHORT = {"central apnea": "CA", "obstructive apnea": "OA", "hypopnea": "Hyp",
            "apnea": "A", "arousal": "Arousal", "rera": "RERA"}


def short(text):
    t = text.strip().lower()
    return EV_SHORT.get(t, text.strip())


def load_airsense(paths):
    """Concatenate EVE / BRP / PLD across files, on absolute device time (s)."""
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += glob.glob(os.path.join(p, "*.edf")) + glob.glob(os.path.join(p, "*.EDF"))
        else:
            files.append(p)
    files = list({os.path.normcase(os.path.abspath(f)): f for f in files}.values())
    ev, brp_t, brp_q, brp_p, pld = [], [], [], [], {}
    for f in sorted(files):
        kind = os.path.basename(f).upper()
        try:
            e = read_edf(f)
        except Exception as ex:
            print("  skip %s (%s)" % (f, ex))
            continue
        t0 = e.start.timestamp()
        if "_EVE" in kind:
            for on, du, text in e.annots:
                if "recording" in text.lower():
                    continue
                ev.append((t0 + on, du, short(text)))
        elif "_BRP" in kind:
            q = next((s for l, s in e.signals.items() if l.lower().startswith("flow")), None)
            pr = next((s for l, s in e.signals.items() if l.lower().startswith("press")), None)
            if q is not None:
                scale = 60.0 if q.unit.lower().startswith("l/s") else 1.0   # -> L/min
                brp_t.append(t0 + q.t)
                brp_q.append(q.x * scale)
                brp_p.append(np.interp(q.t, pr.t, pr.x) if pr is not None else np.full(len(q.t), np.nan))
        elif "_PLD" in kind:
            for l, s in e.signals.items():
                if l.lower().startswith("crc"):
                    continue
                scale = 60.0 if (s.unit.lower().startswith("l/s") and "leak" in l.lower()) else 1.0
                pld.setdefault(l, []).append((t0 + s.t, s.x * scale))
    brp = None
    if brp_t:
        t = np.concatenate(brp_t)
        o = np.argsort(t)
        brp = (t[o], np.concatenate(brp_q)[o], np.concatenate(brp_p)[o])
    pld = {l: (np.concatenate([a for a, _ in v]), np.concatenate([b for _, b in v])) for l, v in pld.items()}
    for l in pld:
        o = np.argsort(pld[l][0])
        pld[l] = (pld[l][0][o], pld[l][1][o])
    return sorted(ev), brp, pld, files


def envelope(t, q, t0, t1, step=1.0, win=2.0):
    grid = np.arange(t0, t1, step)
    out = np.full(len(grid), np.nan)
    for i, g in enumerate(grid):
        sel = (t >= g - win / 2) & (t < g + win / 2)
        if sel.sum() > 5:
            out[i] = q[sel].std()
    return grid, out


def best_lag(ga, a, gb, b, lags):
    best = (0.0, 0.0)
    for lag in lags:
        bb = np.interp(ga + lag, gb, b, left=np.nan, right=np.nan)
        m = ~np.isnan(a) & ~np.isnan(bb)
        if m.sum() < 30:
            continue
        x, y = a[m] - a[m].mean(), bb[m] - bb[m].mean()
        den = np.sqrt((x * x).sum() * (y * y).sum())
        if den == 0:
            continue
        r = (x * y).sum() / den
        if abs(r) > abs(best[1]):
            best = (lag, r)
    return best


def ground_truth(raw):
    t0 = raw["t_script"]
    # V tuples: (host, breath, vt, qpk[, flat], event) - flat only in newer runs
    v = [(r[0] - t0, r[2], r[3], r[4] if len(r) >= 6 else float("nan"), r[-1])
         for r in raw["sim_v"]]
    events, cur = [], None
    for (tp, _, _, _, _), (t, vt, qpk, flat, lab) in zip(v[:-1], v[1:]):
        if lab in ("Apnea", "Hypopnea", "FlowLimit"):
            if cur is None or cur["type"] != lab or tp - cur["end"] > 1.0:
                cur = dict(type=lab, onset=tp, end=t, vt=[], pk=[], flat=[])
                events.append(cur)
            cur["end"] = t
            cur["vt"].append(vt)
            cur["pk"].append(qpk)
            cur["flat"].append(flat)
        else:
            cur = None
    sevs = [(ev, sev, dur) for ev, dur, sev in raw["script"]
            if ev in ("apnea", "hypopnea", "flowlimit")]
    shapes = list(raw.get("fl_shapes", []))
    norm = [r for r in v if r[4] == "None"]
    npk = S.median(r[2] for r in norm) if norm else float("nan")
    raw["_normal_flat"] = S.median(r[3] for r in norm) if norm else float("nan")
    for i, e in enumerate(events):
        if i < len(sevs):
            kind, sev, dur = sevs[i]
            if kind == "apnea":
                e["label"] = "apnea %d s" % dur
            elif kind == "hypopnea":
                e["label"] = "hyp %d %%" % sev
            else:
                e["label"] = "%s %d %%" % (shapes.pop(0) if shapes else "FL", sev)
        else:
            e["label"] = e["type"].lower()
        e["pk_ratio"] = S.median(e["pk"]) / npk if npk else float("nan")
        e["flat_med"] = S.median(e["flat"]) if e["flat"] else float("nan")
    return events


def ours_by_segment(ours, events):
    """Counter rises (CA/OA/Hyp/FL) of our CPAP between onset and end + 20 s;
    both runs used the same script, so script-relative times line up."""
    if ours is None:
        return {}
    off = ours["t_script"] - ours["t0"]
    aps = sorted(({**a, "ts": a["t"] - off} for a in ours["ap_log"]), key=lambda a: a["ts"])
    def at(k, ts):
        v = 0
        for a in aps:
            if a["ts"] <= ts:
                v = a[k]
        return v
    out = {}
    for i, e in enumerate(events):
        got = []
        for k in ("CA", "OA", "Hyp", "FL"):
            d = at(k, e["end"] + 20.0) - at(k, e["onset"] - 2.0)
            if d:
                got.append("%s%s" % (k, "" if d == 1 else " x%d" % d))
        out[i] = ", ".join(got) or "-"
    return out


def main(argv):
    ours_path = None
    if "--ours" in argv:
        k = argv.index("--ours")
        ours_path = argv[k + 1]
        argv = argv[:k] + argv[k + 2:]
    raw = pickle.load(open(argv[0], "rb"))
    ours = pickle.load(open(ours_path, "rb")) if ours_path else None
    ev_as, brp, pld, files = load_airsense(argv[1:])
    print("=" * 78)
    print("AIRSENSE vs GROUND TRUTH%s - script '%s'" % (" vs OUR CPAP" if ours else "", raw["script_name"]))
    print("=" * 78)
    print("  AirSense files: %d   events %d   BRP %s   PLD signals: %s"
          % (len(files), len(ev_as), "%d samples" % len(brp[0]) if brp else "none",
             ", ".join(sorted(pld)) or "none"))

    # ---- external sensor on the PC wall clock -----------------------------------
    E = np.array(raw["ext"], dtype=float)
    t_e = ext_times(E[:, 0]) + raw["wall_off"]                # PC wall seconds
    q_e = E[:, 2]
    w0 = raw["t_script"] + raw["wall_off"]                      # script start, PC wall
    w1 = w0 + sum(d for _, d, _ in raw["script"])

    # ---- align the AirSense clock ------------------------------------------------
    lag, r = 0.0, float("nan")
    if brp is not None:
        ga, a = envelope(t_e, q_e, w0, w1)
        gb, b = envelope(brp[0], brp[1], brp[0][0], brp[0][-1])
        # Seed from the session start: the AirSense's flow record begins when
        # therapy starts, which the run harness timed from the external
        # pressure. Its clock can be hours off (2026-10-20: +2 h 31 min), so
        # search +/-15 min around that seed rather than around zero.
        seed = 0.0
        if raw.get("t_therapy"):
            seed = round(brp[0][0] - (raw["t_therapy"] + raw["wall_off"]))
        coarse, r1 = best_lag(ga, a, gb, b, np.arange(seed - 900.0, seed + 900.5, 1.0))
        grid = np.arange(w0 + 5, w1 - 5, 0.04)
        fa = np.interp(grid, t_e, q_e)
        fine, r = best_lag(grid, fa, brp[0], brp[1], np.arange(coarse - 2.0, coarse + 2.0, 0.04))
        lag = fine
        print("  clock: AirSense = PC %+.2f s   (envelope r %.2f, flow r %.3f%s)"
              % (lag, r1, r, ", flow sign inverted" if r < 0 else ""))
        if abs(r) < 0.8:
            print("  !! weak alignment - treat event timing with care")
    else:
        print("  !! no BRP flow - assuming the AirSense clock equals the PC clock")

    def to_script(t_dev):                     # AirSense device time -> script seconds
        return t_dev - lag - w0

    events = ground_truth(raw)
    ours_seg = ours_by_segment(ours, events)
    as_rel = [(to_script(t), du, k) for t, du, k in ev_as]
    used = set()
    press = next((pld[l] for l in pld if l.lower().startswith("press")), None)
    flim = next((pld[l] for l in pld if "flowlim" in l.lower()), None)

    print("\n  %-14s %6s %5s | %-34s | %-16s | %s"
          % ("event", "onset", "peak", "AirSense (s after onset, duration)", "AirSense P / FL", "our CPAP"))
    for i, e in enumerate(events):
        hits = [(k, t - e["onset"], du, j) for j, (t, du, k) in enumerate(as_rel)
                if e["onset"] - 5.0 <= t <= e["end"] + 30.0]
        for h in hits:
            used.add(h[3])
        txt = ", ".join("%s +%.0f (%.0f s)" % (k, d, du) for k, d, du, _ in hits) or "nothing"
        pf = ""
        if press is not None:
            pt = press[0] - lag - w0
            p_end = np.interp(e["end"] + 30.0, pt, press[1])
            pf = "%.1f" % p_end
        if flim is not None:
            ft = flim[0] - lag - w0
            sel = (ft >= e["onset"]) & (ft <= e["end"])
            pf += " / %.2f" % (flim[1][sel].max() if sel.any() else float("nan"))
        print("  %-14s %6.0f %4.0f%% | %-34s | %-16s | %s"
              % (e["label"], e["onset"], 100 * e["pk_ratio"], txt, pf, ours_seg.get(i, "")))
    fl_events = [e for e in events if e["type"] == "FlowLimit"]
    if fl_events and (press is not None or flim is not None):
        print("\n  flattened breaths (normal-breath flatness %.2f; the APAP's flat rule is > 0.78):"
              % raw.get("_normal_flat", float("nan")))
        print("    %-14s %6s %6s | %-19s | %s"
              % ("event", "flat", "Vt", "AirSense FL med/max", "AirSense pressure: onset -> end -> end+60 s (max)"))
        for e in fl_events:
            fl_txt = p_txt = ""
            if flim is not None:
                ft = flim[0] - lag - w0
                sel = (ft >= e["onset"]) & (ft <= e["end"])
                if sel.any():
                    fl_txt = "%.2f / %.2f" % (np.median(flim[1][sel]), flim[1][sel].max())
            if press is not None:
                pt = press[0] - lag - w0
                sel = (pt >= e["onset"]) & (pt <= e["end"] + 60.0)
                p_txt = "%.1f -> %.1f -> %.1f  (%.1f)" % (
                    np.interp(e["onset"], pt, press[1]), np.interp(e["end"], pt, press[1]),
                    np.interp(e["end"] + 60.0, pt, press[1]),
                    press[1][sel].max() if sel.any() else float("nan"))
            print("    %-14s %6.2f %5.0f | %-19s | %s"
                  % (e["label"], e["flat_med"], S.median(e["vt"]), fl_txt, p_txt))
    if press is not None:
        pt = press[0] - lag - w0
        sel = (pt >= 0) & (pt <= (w1 - w0) + 30)
        if sel.any():
            print("\n  AirSense therapy pressure over the script: %.1f .. %.1f cmH2O"
                  % (press[1][sel].min(), press[1][sel].max()))
    extra = [(t, du, k) for j, (t, du, k) in enumerate(as_rel)
             if j not in used and -30 <= t <= (w1 - w0) + 30]
    if extra:
        print("\n  AirSense events outside any delivered event:")
        for t, du, k in extra:
            print("    %s at script t=%.0f s (%.0f s)" % (k, t, du))
    tot = {}
    for t, du, k in as_rel:
        if 0 <= t <= (w1 - w0) + 30:
            tot[k] = tot.get(k, 0) + 1
    print("\n  AirSense totals in the script window: %s"
          % (", ".join("%s %d" % kv for kv in sorted(tot.items())) or "none"))
    plot(raw, events, as_rel, press, flim, lag, w0, w1, t_e, q_e, argv[0])


def plot(raw, events, as_rel, press, flim, lag, w0, w1, t_e, q_e, raw_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = 1 + (press is not None) + (flim is not None)
    fig, axes = plt.subplots(n, 1, figsize=(11, 2.3 * n + 0.6), sharex=True, squeeze=False)
    axes = axes[:, 0]
    sel = (t_e >= w0) & (t_e <= w1 + 30)
    ax = axes[0]
    ax.plot(t_e[sel] - w0, q_e[sel], color="#2a78d6", lw=0.6)
    ax.set_ylabel("ext flow, slm", fontsize=8, color="#555555")
    for e in events:
        for a in axes:
            a.axvspan(e["onset"], e["end"], color="#eeeeee", zorder=0, lw=0)
    ymax = np.nanmax(q_e[sel]) if sel.any() else 1
    for t, du, k in as_rel:
        if -30 <= t <= (w1 - w0) + 30:
            ax.annotate(k, (t, ymax), fontsize=7, color="#333333", ha="left", va="bottom")
            ax.axvline(t, color="#eb6834", lw=1)
    row = 1
    if press is not None:
        axes[row].plot(press[0] - lag - w0, press[1], color="#1baf7a", lw=1.5)
        axes[row].set_ylabel("AirSense P, cmH2O", fontsize=8, color="#555555")
        row += 1
    if flim is not None:
        axes[row].plot(flim[0] - lag - w0, flim[1], color="#6250d6", lw=1.5)
        axes[row].set_ylabel("AirSense FL index", fontsize=8, color="#555555")
    for a in axes:
        a.grid(True, color="#e6e6e6", lw=0.6)
        a.tick_params(labelsize=7, colors="#555555")
        for s in a.spines.values():
            s.set_color("#cccccc")
    axes[-1].set_xlim(-10, (w1 - w0) + 30)
    axes[-1].set_xlabel("s from script start (grey = delivered events, orange = AirSense events)",
                        fontsize=8, color="#555555")
    fig.suptitle("AirSense reference run '%s'" % raw["script_name"], fontsize=11, color="#222222")
    fig.tight_layout()
    out = raw_path.replace("_raw.pkl", "_compare.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("  plot: %s" % out)


if __name__ == "__main__":
    main(sys.argv[1:])
