"""Analysis for spike_source.py: put three flow sensors on one clock, find the
fast transients in each, and say which sensors saw each one.

Clocks:
  CPAP T and simulator F carry device ticks (ms) - uniform but on their own
  clocks; mapped to host time by the earliest-arrival offset, then refined by
  cross-correlating breathing flow against the external sensor.
  External sensor has no timestamp and arrives in USB batches that share one
  host time - never use the arrival times directly. Sample index is uniform
  (~100 Hz), so fit host time vs index in 30 s chunks and take the early edge.

Transient = rise >= RISE slm within 40 ms AND back below half the rise within
0.4 s. Breath onsets and the end of expiration rise too, but they don't come
back down, so they are not counted.
"""
import numpy as np

import os as _os
HERE = _os.path.dirname(_os.path.abspath(__file__))   # code
SP = _os.path.join(HERE, "runs")                       # run data in / out
RISE = 4.0
RISE_WIN = 0.04
BACK_WIN = 0.40
MATCH = 0.25
COLORS = {"CPAP": "#2a78d6", "external": "#eb6834", "simulator": "#1baf7a"}


def dev_to_host(host, tick_s):
    """Device tick -> host time. Arrival = true time + delay >= 0, so the
    early edge (2nd percentile) of host - tick is the offset."""
    return tick_s + np.percentile(host - tick_s, 2)


def ext_times(host, chunk_s=30.0, rate_guess=100.0):
    host = np.asarray(host)
    n = len(host)
    idx = np.arange(n, dtype=float)
    t = np.empty(n)
    step = max(int(chunk_s * rate_guess), 200)
    for a in range(0, n, step):
        b = min(n, a + step)
        if b - a < 20:
            t[a:b] = host[a:b]
            continue
        k, c = np.polyfit(idx[a:b], host[a:b], 1)
        fit = k * idx[a:b] + c
        t[a:b] = fit + np.percentile(host[a:b] - fit, 2)
    return t


def xcorr_lag(ta, qa, tb, qb, windows, max_lag=3.0, step=0.01):
    """Lag (s) to ADD to stream a's time to match b, the sign of b relative to
    a, and |r|. Uses only samples inside `windows` (breathing)."""
    grid = np.concatenate([np.arange(w0, w1, 0.01) for w0, w1 in windows])
    if len(grid) < 500:
        return 0.0, 1.0, 0.0
    b = np.interp(grid, tb, qb)
    b = b - b.mean()
    best = (0.0, 1.0, -1.0)
    for lag in np.arange(-max_lag, max_lag + step / 2, step):
        a = np.interp(grid - lag, ta, qa)
        a = a - a.mean()
        den = np.sqrt((a * a).sum() * (b * b).sum())
        if den == 0:
            continue
        r = (a * b).sum() / den
        if abs(r) > best[2]:
            best = (lag, 1.0 if r >= 0 else -1.0, abs(r))
    return best


def max_rise(t, q, t0, t1):
    """Largest 40 ms rise anywhere in [t0, t1]."""
    sel = (t >= t0) & (t <= t1)
    if sel.sum() < 2:
        return float("nan")
    ts = t[sel]
    prev = np.interp(ts - RISE_WIN, t, q)
    return float(np.max(q[sel] - prev))


def detect(t, q, rise=RISE):
    prev = np.interp(t - RISE_WIN, t, q)
    cand = np.where(q - prev >= rise)[0]
    out, last = [], -1e9
    for i in cand:
        if t[i] - last < 0.5:
            continue
        base = float(np.interp(t[i] - 0.06, t, q))
        w = (t >= t[i]) & (t <= t[i] + 0.3)
        k = np.argmax(np.where(w, q, -1e9))
        peak, tpk = float(q[k]), float(t[k])
        after = float(np.interp(tpk + BACK_WIN, t, q))
        last = t[i]
        if after - base < 0.5 * (peak - base):
            sel = (t >= t[i] - 0.06) & (t <= tpk + 0.6)
            dt = np.diff(t[sel], prepend=t[sel][0])
            vol = float(np.sum(np.clip(q[sel] - base, 0, None) * dt) / 60.0 * 1000.0)
            out.append(dict(t=float(t[i]), base=base, peak=peak, amp=peak - base,
                            vol_ml=vol))
    return out


def main(raw):
    import statistics
    ts0 = raw["t_script"]
    seg_bounds, acc = [], 0.0
    for ev, dur in raw["script"]:
        seg_bounds.append((acc, acc + dur, ev))
        acc += dur

    def seg(x):
        for a, b, ev in seg_bounds:
            if a <= x < b:
                return ev
        return "after"

    T = np.array([(r[0], r[1], r[3], r[6]) for r in raw["t_log"]], dtype=float)
    F = np.array([(r[0], r[1], r[2], r[4]) for r in raw["sim_f"]], dtype=float)
    E = np.array(raw["ext"], dtype=float)
    print("\n" + "=" * 78)
    print("SPIKE SOURCE - three flow sensors on one clock")
    print("=" * 78)
    for name, arr in (("CPAP T", T), ("sim F", F), ("ext", E)):
        if len(arr) < 100:
            print("  !! %s: only %d samples - cannot analyse" % (name, len(arr)))
            return
    t_c = dev_to_host(T[:, 0], T[:, 1] / 1000.0) - ts0
    q_c = T[:, 2]
    okf = F[:, 3] > 0
    t_s = dev_to_host(F[okf, 0], F[okf, 1] / 1000.0) - ts0
    q_s = F[okf, 2]
    t_e = ext_times(E[:, 0]) - ts0
    q_e, p_e = E[:, 2], E[:, 1]
    for name, tt in (("CPAP T", t_c), ("sim F", t_s), ("ext", t_e)):
        print("  %-7s %6d samples  %.1f Hz" % (name, len(tt), (len(tt) - 1) / (tt[-1] - tt[0])))
    print("  sim F samples flagged not-ok: %d   ext CRC errors: %d"
          % (int((~okf).sum()), raw.get("ext_crc", 0)))

    breathing = [(a + 3, b - 3) for a, b, ev in seg_bounds if ev == "none"]
    lag_c, sgn_c, r_c = xcorr_lag(t_c, q_c, t_e, q_e, breathing)
    lag_s, sgn_s, r_s = xcorr_lag(t_s, q_s, t_e, q_e, breathing)
    t_c = t_c + lag_c
    t_s = t_s + lag_s
    q_e = q_e * sgn_c                      # put ext flow in the CPAP's sign
    print("  alignment to external: CPAP %+.2f s (r %.3f%s), simulator %+.2f s (r %.3f%s)"
          % (lag_c, r_c, ", ext sign flipped" if sgn_c < 0 else "",
             lag_s, r_s, "" if sgn_s == sgn_c else ", SIM SIGN DIFFERS"))
    if sgn_s != sgn_c:
        q_s = -q_s
    if min(r_c, r_s) < 0.8:
        print("  !! weak alignment (r < 0.8) - treat coincidences with care")

    streams = {"CPAP": (t_c, q_c), "external": (t_e, q_e), "simulator": (t_s, q_s)}
    ev = []
    for name, (tt, qq) in streams.items():
        for d in detect(tt, qq):
            d["by"] = name
            ev.append(d)
    ev.sort(key=lambda d: d["t"])
    clusters = []
    for d in ev:
        if clusters and d["t"] - clusters[-1][-1]["t"] <= MATCH:
            clusters[-1].append(d)
        else:
            clusters.append([d])

    print("\n  transients (rise >= %.0f slm in 40 ms, back down within %.1f s):"
          % (RISE, BACK_WIN))
    print("    %7s %-6s  %-28s %8s %8s %8s  %s"
          % ("t", "seg", "seen by", "CPAP", "ext", "sim", "ext P dip"))
    rows = []
    for cl in clusters:
        t0 = min(d["t"] for d in cl)
        by = sorted({d["by"] for d in cl}, key=list(streams).index)
        rise = {n: max_rise(tt, qq, t0 - MATCH, t0 + MATCH) for n, (tt, qq) in streams.items()}
        sel = (t_e >= t0 - 0.3) & (t_e <= t0 + 0.3)
        pdip = float(np.min(p_e[sel]) - np.interp(t0 - 0.3, t_e, p_e)) if sel.any() else float("nan")
        amp = {d["by"]: d for d in cl}
        rows.append(dict(t=t0, seg=seg(t0), by=by, rise=rise, pdip=pdip, amp=amp))
        print("    %7.2f %-6s  %-28s %8s %8s %8s  %+.2f"
              % (t0, seg(t0), "+".join(by),
                 *["%.1f%s" % (rise[n], "*" if n in by else "") for n in streams],
                 pdip))
    print("    (columns = largest 40 ms rise within +/-%.2f s in each stream; * = "
          "detected as a transient there)" % MATCH)

    def key(r):
        return "+".join(r["by"])
    combos = {}
    for r in rows:
        combos.setdefault(key(r), []).append(r)
    print("\n  SUMMARY by which sensors saw it:")
    for k, v in sorted(combos.items(), key=lambda kv: -len(kv[1])):
        segs = statistics.mode([r["seg"] for r in v])
        print("    %-30s %3d   (mostly in %s)" % (k, len(v), segs))
    cp_rows = [r for r in rows if "CPAP" in r["by"]]
    if cp_rows:
        n_e = sum(1 for r in cp_rows if "external" in r["by"])
        n_s = sum(1 for r in cp_rows if "simulator" in r["by"])
        print("\n  Of %d CPAP-side transients: external also saw %d, simulator also saw %d."
              % (len(cp_rows), n_e, n_s))
        if n_e == 0 and n_s == 0:
            verdict = ("CPAP ONLY -> between the CPAP sensor and the external sensor "
                       "(hose/vent), or a CPAP flow-sensor artefact")
        elif n_s >= max(1, len(cp_rows) // 2):
            verdict = "ALL THREE -> air really moved at the simulator/lung end"
        elif n_e >= max(1, len(cp_rows) // 2):
            verdict = ("CPAP + EXTERNAL, not the simulator -> between the external "
                       "sensor and the simulator sensor: collapsible chamber or box")
        else:
            verdict = "MIXED - see the table"
        print("  VERDICT: %s" % verdict)
    else:
        print("\n  No CPAP-side transient this run - inconclusive; rerun longer.")

    plot(rows[:12], streams, p_e, t_e)
    with open(SP + "/spike_events.csv", "w") as fh:
        fh.write("t_s,segment,seen_by,rise_cpap,rise_ext,rise_sim,ext_p_dip\n")
        for r in rows:
            fh.write("%.3f,%s,%s,%.2f,%.2f,%.2f,%.3f\n"
                     % (r["t"], r["seg"], key(r), r["rise"]["CPAP"],
                        r["rise"]["external"], r["rise"]["simulator"], r["pdip"]))
    print("  events: spike_events.csv   plots: spike_source.png")


def plot(rows, streams, p_e, t_e):
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(rows)
    cols = 3 if n > 4 else n
    nr = (n + cols - 1) // cols
    fig, axes = plt.subplots(nr, cols, figsize=(4.6 * cols, 3.0 * nr), squeeze=False)
    for ax in axes.flat[n:]:
        ax.axis("off")
    for ax, r in zip(axes.flat, rows):
        t0 = r["t"]
        for name, (tt, qq) in streams.items():
            sel = (tt >= t0 - 1.0) & (tt <= t0 + 1.5)
            ref = float(np.interp(t0 - 0.1, tt, qq))
            ax.plot(tt[sel] - t0, qq[sel] - ref, color=COLORS[name], lw=2, label=name)
        ax.set_title("%.1f s (%s): %s" % (t0, r["seg"], "+".join(r["by"])),
                     fontsize=9, color="#333333")
        ax.axvline(0, color="#bbbbbb", lw=1, zorder=0)
        ax.grid(True, color="#e6e6e6", lw=0.6)
        ax.set_xlabel("s from transient", fontsize=8, color="#555555")
        ax.set_ylabel("flow change, slm", fontsize=8, color="#555555")
        ax.tick_params(labelsize=7, colors="#555555")
        for s in ax.spines.values():
            s.set_color("#cccccc")
    axes.flat[0].legend(fontsize=7, frameon=False, loc="upper right")
    fig.suptitle("Fast flow transients: CPAP vs external vs simulator sensor "
                 "(air-path order; change from 0.1 s before)", fontsize=11, color="#222222")
    fig.tight_layout()
    fig.savefig(SP + "/spike_source.png", dpi=110)
    plt.close(fig)
