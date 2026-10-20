"""Offline replay of the CPAP's EPR breath FSM (epr_ctrl.c, R4/R5 working tree)
on logged T-line patient flow, to see what each breath candidate looked like
when noteQualifiedBreath() judged it.

Approximations: the firmware runs at 200 Hz, T is 50 Hz (20 ms steps); the
base setpoint is the S-line APAP target (interpolated); polarity +1; EPR
defaults: bump_mit 0, peak_fall_detect 1, insp_pressure_band 0.5, confirm
50 ms, min_hold 120, max_hold 1200, min_peak 4, min fall 6, post_peak 80,
exp_end 4 slm for 120 ms. Startup grace and pressurize-hold are ignored.

usage: python epr_replay.py <raw.pkl>
"""
import bisect
import pickle
import sys

IDLE, PEND, INSP, EXPP, EXP = range(5)
MIN_MS, MIN_ML = 400, 80.0


def replay(rows, sp_at):
    base, seeded = 0.0, False
    fsm, phase, drop = IDLE, 0, 0
    peak, peak_q, peak_ms = 0.0, 0.0, 0
    start, vol, enter = 0, 0.0, 0
    awaiting = False
    out = []
    prev_tick = None
    for t, tick, q, p in rows:
        dt = 20 if prev_tick is None else max(1, min(100, tick - prev_tick))
        prev_tick = tick
        now = tick
        if not seeded:
            base, seeded = q, True
        elif abs(q - base) < 5.0:
            a = (dt / 1000.0) / (2.5 + dt / 1000.0)
            base = a * q + (1 - a) * base
        delta = q - base
        ith = 4.0 + max(abs(base), 2.0) * 0.02

        def stamp(kind):
            if peak_q < 4.0:
                return
            ms = now - start
            out.append(dict(t=t, ms=ms, ml=vol, peak=peak_q, kind=kind,
                            old=True, new=(ms >= MIN_MS and vol >= MIN_ML)))

        if fsm == IDLE:
            peak_q, peak = 0.0, 0.0
            drop = 0
            if not awaiting and q > ith:
                fsm, phase = PEND, 0
                peak, peak_ms = abs(delta), now
                start, vol = now, 0.0
        elif fsm == PEND:
            if q > ith:
                drop = 0
                phase += dt
                if delta > peak:
                    peak, peak_ms = delta, now
                if q > peak_q:
                    peak_q, peak_ms = q, now
                if q > 0:
                    vol += q * dt / 60.0
                if phase >= 50:
                    fsm, phase = INSP, 0
                    enter = now
                    peak_ms = enter
            else:
                drop += dt
                phase = 0
                if drop >= 80:
                    fsm, drop, peak, peak_q = IDLE, 0, 0.0, 0.0
        elif fsm == INSP:
            if delta > peak:
                peak, peak_ms = delta, now
            if q > peak_q:
                peak_q, peak_ms = q, now
            if q > 0:
                vol += q * dt / 60.0
            el = now - enter
            if el >= 1200:
                stamp("max_hold")
                awaiting, fsm, phase = True, EXP, 0
            else:
                ending = (peak_q >= 4.0 and peak_q - q >= 6.0
                          and p >= sp_at(t) - 0.5 and now - peak_ms >= 80)
                if ending and el >= 120:
                    phase += dt
                    if phase >= 50:
                        stamp("peak_fall")
                        awaiting, fsm, phase = True, EXPP, 0
                else:
                    phase = 0
        elif fsm == EXPP:
            phase += dt
            if phase >= 50:
                fsm, phase = EXP, 0
        elif fsm == EXP:
            if abs(q) < 4.0:
                phase += dt
                if phase >= 120:
                    fsm, phase, awaiting = IDLE, 0, False
                    peak, peak_q = 0.0, 0.0
            else:
                phase = 0
    return out


def main(path):
    d = pickle.load(open(path, "rb"))
    off = d["t_script"] - d["t0"]
    rows = [(r[0] - off, r[1], r[3], r[6]) for r in d["t_log"]]
    sts = sorted((x["t"] - off, x["target"], x["osc"], x["apst"]) for x in d["st_log"])
    st_t = [x[0] for x in sts]

    def st_at(t):
        i = max(0, bisect.bisect_right(st_t, t) - 1)
        return sts[i]

    cands = replay(rows, lambda t: st_at(t)[1])
    # S lines arrive ~1-2 s late: judge FOT by the status 1.5 s later
    for c in cands:
        s = st_at(c["t"] + 1.5)
        c["fot"], c["apst"] = s[2], s[3]
    for label, sel in (("outside FOT", [c for c in cands if not c["fot"]]),
                       ("during FOT", [c for c in cands if c["fot"]])):
        n = len(sel)
        if not n:
            print("  %-12s no candidates" % label)
            continue
        ms = sorted(c["ms"] for c in sel)
        ml = sorted(c["ml"] for c in sel)
        print("  %-12s %4d candidates | insp ms median %4d (p10 %4d) | mL median %5.0f "
              "(p10 %4.0f) | old rule accepts %3d%% | R5a accepts %3d%% | max_hold %d"
              % (label, n, ms[n // 2], ms[n // 10], ml[n // 2], ml[n // 10],
                 100 * sum(c["old"] for c in sel) // n,
                 100 * sum(c["new"] for c in sel) // n,
                 sum(c["kind"] == "max_hold" for c in sel)))
    return cands


if __name__ == "__main__":
    main(sys.argv[1])
