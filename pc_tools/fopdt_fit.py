#!/usr/bin/env python3
"""
Fit FOPDT plant model (K, tau, theta) from CPAP telemetry logs and compute SIMC PID gains.

Expects the same VCP line formats as cpap_telemetry_plot.py:
  T,<ms>,<pressure_cmh2o>,<patient_flow_slm>[,<pid_rpm_cmd>[,<mech_rpm>]]
  R,<ms>,<rpm_cmd>,<rpm_act>,<pressure_cmh2o>

Typical workflow:
  1. Capture raw telemetry (recommended, 50 Hz):
       python cpap_telemetry_plot.py --port COM13 --monitor --record bench_raw.log --monitor-time 60
  2. Or use monitor output redirected to a file (lower sample rate, still works):
       python cpap_telemetry_plot.py --port COM13 --monitor --monitor-time 60 > bench.log
       python fopdt_fit.py bench.log
       python fopdt_fit.py bench.log --step 1 --plot
       python fopdt_fit.py bench.log --tau-c 0.6 --ksqrt 6141

The fit uses pressure samples (T lines) around an R-line RPM command step.
Open-loop bench data (PID off, fixed leak) gives the cleanest FOPDT estimate.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    from scipy.optimize import curve_fit
except ImportError as exc:
    raise SystemExit(
        "scipy is required for fopdt_fit.py\n"
        "  pip install scipy\n"
        "  or: pip install -r pc_tools/requirements.txt"
    ) from exc

TELEM_RE = re.compile(
    r"^T,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)(?:,-?\d+){0,2}\s*$"
)
BENCH_RE = re.compile(
    r"^R,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$"
)
# cpap_telemetry_plot.py --monitor embeds the latest line as "last=T,..." / "last=R,..."
MONITOR_TELEM_RE = re.compile(
    r"last=T,(\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)(?:,-?\d+){0,2}"
)
MONITOR_BENCH_RE = re.compile(
    r"last=R,(\d+),(-?\d+),(-?\d+),([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
)

KSQRT_DEFAULT = 6141.0
ZONE_P_CMH2O = {
    "LOW@6": 6.0,
    "LOW@8": 8.0,
    "MID@12": 12.0,
    "HIGH@20": 20.0,
}


@dataclass(frozen=True)
class TelemSample:
    ms: int
    pressure: float
    flow: float


@dataclass(frozen=True)
class BenchEvent:
    ms: int
    rpm_cmd: int
    rpm_act: int
    pressure: float


@dataclass(frozen=True)
class FopdtFit:
    k_cmh2o_per_rpm: float
    tau_s: float
    theta_s: float
    delta_rpm: float
    p0_cmh2o: float
    p_ss_cmh2o: float
    step_ms: int
    rmse_cmh2o: float
    n_samples: int


@dataclass(frozen=True)
class SimcGains:
    kp: float
    ki: float
    kd: float
    tau_i_s: float
    tau_d_s: float


def parse_log(text: str) -> tuple[list[TelemSample], list[BenchEvent], str]:
    telem_by_ms: dict[int, TelemSample] = {}
    bench_by_ms: dict[int, BenchEvent] = {}
    raw_t = 0
    raw_r = 0
    mon_t = 0
    mon_r = 0

    for raw in re.split(r"[\r\n]+", text):
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith("-"):
            continue
        if line.startswith("Monitoring ") or line.startswith("Expect lines"):
            continue
        if line.startswith("Done in ") or line.startswith("Telemetry OK"):
            continue

        matched = False
        m = TELEM_RE.match(line)
        if m:
            ms = int(m.group(1))
            telem_by_ms[ms] = TelemSample(ms=ms, pressure=float(m.group(2)), flow=float(m.group(3)))
            raw_t += 1
            matched = True

        m = BENCH_RE.match(line)
        if m:
            ms = int(m.group(1))
            bench_by_ms[ms] = BenchEvent(
                ms=ms,
                rpm_cmd=int(m.group(2)),
                rpm_act=int(m.group(3)),
                pressure=float(m.group(4)),
            )
            raw_r += 1
            matched = True

        for m in MONITOR_TELEM_RE.finditer(line):
            ms = int(m.group(1))
            telem_by_ms[ms] = TelemSample(ms=ms, pressure=float(m.group(2)), flow=float(m.group(3)))
            mon_t += 1
            matched = True

        for m in MONITOR_BENCH_RE.finditer(line):
            ms = int(m.group(1))
            bench_by_ms[ms] = BenchEvent(
                ms=ms,
                rpm_cmd=int(m.group(2)),
                rpm_act=int(m.group(3)),
                pressure=float(m.group(4)),
            )
            mon_r += 1
            matched = True

        _ = matched

    telem = sorted(telem_by_ms.values(), key=lambda s: s.ms)
    bench = sorted(bench_by_ms.values(), key=lambda e: e.ms)

    if raw_t:
        fmt = "raw T/R lines"
    elif mon_t:
        fmt = f"monitor last=T/R snippets (~{mon_t} pressure samples, subsampled)"
    else:
        fmt = "unknown"

    return telem, bench, fmt


def k_plant_sqrt(p_cmh2o: float, ksqrt: float) -> float:
    """Local dP/dRPM from P = (RPM/K_sqrt)^2."""
    return 2.0 * math.sqrt(max(p_cmh2o, 1e-6)) / ksqrt


def fopdt_step(t: np.ndarray, k: float, tau: float, theta: float, delta_rpm: float, p0: float) -> np.ndarray:
    """Pressure response to RPM step: p0 + K*du*(1 - exp(-(t-theta)/tau)), t>=theta."""
    dt = np.maximum(0.0, t - theta)
    return p0 + k * delta_rpm * (1.0 - np.exp(-dt / np.maximum(tau, 1e-6)))


def estimate_initial(
    t: np.ndarray, p: np.ndarray, delta_rpm: float
) -> tuple[float, float, float, float]:
    """63% / onset heuristics for curve_fit seeds."""
    pre_mask = t < -0.02
    if np.any(pre_mask):
        p0 = float(np.mean(p[pre_mask]))
    else:
        p0 = float(p[0])

    post_mask = t > 0.5 * float(t[-1])
    p_ss = float(np.mean(p[post_mask])) if np.any(post_mask) else float(p[-1])
    dp = p_ss - p0
    if abs(delta_rpm) < 1.0:
        raise ValueError("delta RPM too small for plant gain estimate")
    k0 = dp / delta_rpm

    if abs(dp) < 0.02:
        return k0, 0.35, 0.08, p0

    target_02 = p0 + 0.02 * dp
    target_63 = p0 + 0.632 * dp
    post = t >= 0.0
    p_post = p[post]
    t_post = t[post]
    idx_onset = int(np.argmax(p_post > target_02)) if np.any(p_post > target_02) else 0
    theta0 = float(t_post[max(0, idx_onset)]) if len(t_post) else 0.05
    theta0 = min(max(theta0, 0.0), 0.5 * float(t[-1]))

    above_63 = np.where(p_post >= target_63)[0]
    if len(above_63) == 0:
        tau0 = 0.35
    else:
        t_63 = float(t_post[above_63[0]])
        tau0 = max(0.05, t_63 - theta0)

    return k0, tau0, theta0, p0


def fit_fopdt(
    t: np.ndarray,
    p: np.ndarray,
    delta_rpm: float,
    *,
    p0: float | None = None,
) -> tuple[float, float, float, float, float]:
    """Return k, tau, theta, p0, rmse."""
    if len(t) < 8:
        raise ValueError(f"need at least 8 pressure samples in window, got {len(t)}")

    k0, tau0, theta0, p0_est = estimate_initial(t, p, delta_rpm)
    if p0 is None:
        p0 = p0_est

    t_max = float(t[-1])

    def model(tt: np.ndarray, k: float, tau: float, theta: float) -> np.ndarray:
        return fopdt_step(tt, k, tau, theta, delta_rpm, p0)

    p_span = float(np.max(p) - np.min(p))
    k_lo = max(1e-6, abs(k0) * 0.2)
    k_hi = max(k_lo * 2.0, abs(k0) * 5.0 + 0.01)

    p0_opt, _ = curve_fit(
        model,
        t,
        p,
        p0=(k0, tau0, theta0),
        bounds=(
            [k_lo, 0.03, 0.0],
            [k_hi, min(8.0, t_max * 2.0), min(2.0, t_max * 0.8)],
        ),
        maxfev=20000,
    )
    k, tau, theta = (float(p0_opt[0]), float(p0_opt[1]), float(p0_opt[2]))
    pred = model(t, k, tau, theta)
    rmse = float(np.sqrt(np.mean((p - pred) ** 2)))
    return k, tau, theta, p0, rmse


def simc_pid(k: float, tau: float, theta: float, tau_c: float) -> SimcGains:
    """Skogestad SIMC PID gains matching blower_ctrl.c parallel form."""
    kc = (1.0 / k) * (tau + 0.5 * theta) / (tau_c + 0.5 * theta)
    tau_i = tau + 0.5 * theta
    tau_d = (tau * theta) / (2.0 * tau + theta)
    ki = kc / tau_i
    kd = kc * tau_d
    return SimcGains(kp=kc, ki=ki, kd=kd, tau_i_s=tau_i, tau_d_s=tau_d)


def bench_delta_rpm(bench: list[BenchEvent], step_idx: int, delta_rpm: float | None) -> float:
    if delta_rpm is not None:
        return delta_rpm
    if not bench:
        raise ValueError("no R,... bench lines in log; pass --delta-rpm and --step-ms")
    if step_idx < 0 or step_idx >= len(bench):
        raise ValueError(f"--step {step_idx} out of range (0..{len(bench) - 1})")
    if step_idx == 0:
        return float(bench[0].rpm_cmd - bench[0].rpm_act)
    return float(bench[step_idx].rpm_cmd - bench[step_idx - 1].rpm_cmd)


def extract_window(
    telem: list[TelemSample],
    step_ms: int,
    pre_s: float,
    post_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    t0 = step_ms - int(pre_s * 1000.0)
    t1 = step_ms + int(post_s * 1000.0)
    pts = [s for s in telem if t0 <= s.ms <= t1]
    if not pts:
        raise ValueError(
            f"no T samples in [{t0}, {t1}] ms around step at {step_ms} ms"
        )
    t = np.array([(s.ms - step_ms) / 1000.0 for s in pts], dtype=float)
    p = np.array([s.pressure for s in pts], dtype=float)
    return t, p


def resolve_step_ms(
    bench: list[BenchEvent],
    step_idx: int,
    step_ms: int | None,
) -> int:
    if step_ms is not None:
        return step_ms
    if not bench:
        raise ValueError("no R lines; pass --step-ms <ms>")
    if step_idx < 0 or step_idx >= len(bench):
        raise ValueError(f"--step {step_idx} out of range (0..{len(bench) - 1})")
    return bench[step_idx].ms


def scaled_zone_gains(
    base: SimcGains,
    p_ref: float,
    ksqrt: float,
) -> dict[str, SimcGains]:
    k_ref = k_plant_sqrt(p_ref, ksqrt)
    out: dict[str, SimcGains] = {}
    for name, p_zone in ZONE_P_CMH2O.items():
        scale = k_ref / k_plant_sqrt(p_zone, ksqrt)
        out[name] = SimcGains(
            kp=base.kp * scale,
            ki=base.ki * scale,
            kd=base.kd * scale,
            tau_i_s=base.tau_i_s,
            tau_d_s=base.tau_d_s,
        )
    return out


def print_report(
    fit: FopdtFit,
    tau_c: float,
    gains: SimcGains,
    zone_gains: dict[str, SimcGains],
    ksqrt: float,
) -> None:
    print("=" * 60)
    print("FOPDT fit (pressure vs RPM step)")
    print("=" * 60)
    print(f"  Step time     : {fit.step_ms} ms")
    print(f"  Delta RPM     : {fit.delta_rpm:+.0f}")
    print(f"  P0 / P_ss     : {fit.p0_cmh2o:.3f} / {fit.p_ss_cmh2o:.3f} cmH2O")
    print(f"  dP steady     : {fit.p_ss_cmh2o - fit.p0_cmh2o:+.3f} cmH2O")
    print(f"  K             : {fit.k_cmh2o_per_rpm:.6f} cmH2O/RPM")
    print(f"  tau           : {fit.tau_s:.3f} s")
    print(f"  theta         : {fit.theta_s:.3f} s")
    print(f"  RMSE          : {fit.rmse_cmh2o:.4f} cmH2O  ({fit.n_samples} samples)")
    k_sqrt = k_plant_sqrt(0.5 * (fit.p0_cmh2o + fit.p_ss_cmh2o), ksqrt)
    print(f"  K_sqrt model  : {k_sqrt:.6f} cmH2O/RPM @ P~{(fit.p0_cmh2o + fit.p_ss_cmh2o) / 2:.1f}")
    print()
    print(f"SIMC PID (tau_c = {tau_c:.2f} s) - matches blower_ctrl.c units")
    print(f"  Kp = {gains.kp:.1f}   Ki = {gains.ki:.1f}   Kd = {gains.kd:.1f}")
    print(f"  Ti = {gains.tau_i_s:.3f} s   Td = {gains.tau_d_s:.3f} s")
    print()
    print("Gain-scheduled SIMC (scale Kp/Ki/Kd by K_sqrt at zone / K at step):")
    print(f"  {'Zone':<10} {'Kp':>8} {'Ki':>8} {'Kd':>8}")
    for name, g in zone_gains.items():
        print(f"  {name:<10} {g.kp:8.0f} {g.ki:8.0f} {g.kd:8.1f}")
    print()
    print("Firmware globals (LOW / MID / HIGH):")
    print(
        f"  g_blower_ctrl_kp_low  = {zone_gains['LOW@8'].kp:.0f}f;\n"
        f"  g_blower_ctrl_ki_low  = {zone_gains['LOW@8'].ki:.0f}f;\n"
        f"  g_blower_ctrl_kd_low  = {zone_gains['LOW@8'].kd:.1f}f;\n"
        f"  g_blower_ctrl_kp_mid  = {zone_gains['MID@12'].kp:.0f}f;\n"
        f"  g_blower_ctrl_ki_mid  = {zone_gains['MID@12'].ki:.0f}f;\n"
        f"  g_blower_ctrl_kd_mid  = {zone_gains['MID@12'].kd:.1f}f;\n"
        f"  g_blower_ctrl_kp_high = {zone_gains['HIGH@20'].kp:.0f}f;\n"
        f"  g_blower_ctrl_ki_high = {zone_gains['HIGH@20'].ki:.0f}f;\n"
        f"  g_blower_ctrl_kd_high = {zone_gains['HIGH@20'].kd:.1f}f;"
    )
    print()
    print("Notes:")
    print("  - Fit is open-loop FOPDT; raise Kp gradually in closed loop.")
    print("  - With sqrt FF on, PID trims only — SIMC is an upper bound for Kp.")
    print("  - Re-fit per zone pressure if tau/theta shift with operating point.")


def maybe_plot(
    t: np.ndarray,
    p: np.ndarray,
    fit: FopdtFit,
    show: bool,
    out_path: str | None,
) -> None:
    import matplotlib.pyplot as plt

    pred = fopdt_step(
        t,
        fit.k_cmh2o_per_rpm,
        fit.tau_s,
        fit.theta_s,
        fit.delta_rpm,
        fit.p0_cmh2o,
    )
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, p, "b.", markersize=4, label="P measured (T lines)")
    ax.plot(t, pred, "r-", linewidth=1.5, label="FOPDT fit")
    ax.axvline(fit.theta_s, color="gray", linestyle="--", alpha=0.6, label=f"theta={fit.theta_s:.3f}s")
    ax.axhline(fit.p0_cmh2o, color="green", linestyle=":", alpha=0.5, label=f"P0={fit.p0_cmh2o:.2f}")
    ax.axhline(fit.p_ss_cmh2o, color="orange", linestyle=":", alpha=0.5, label=f"P_ss={fit.p_ss_cmh2o:.2f}")
    ax.set_xlabel("Time from RPM step (s)")
    ax.set_ylabel("Pressure at blower (cmH2O)")
    ax.set_title(
        f"FOPDT: K={fit.k_cmh2o_per_rpm:.5f} tau={fit.tau_s:.3f}s "
        f"theta={fit.theta_s:.3f}s  RMSE={fit.rmse_cmh2o:.4f}"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    if out_path:
        fig.savefig(out_path, dpi=120)
        print(f"Plot saved: {out_path}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def run_self_test() -> None:
    """Sanity check on synthetic FOPDT data."""
    k_true, tau_true, theta_true = 0.001, 0.32, 0.07
    delta_rpm, p0 = 500.0, 7.9
    t = np.linspace(0, 4.0, 120)
    p = fopdt_step(t, k_true, tau_true, theta_true, delta_rpm, p0)
    p += np.random.default_rng(0).normal(0, 0.008, size=p.shape)
    k, tau, theta, p0_fit, rmse = fit_fopdt(t, p, delta_rpm)
    assert abs(k - k_true) < 0.00025, k
    assert abs(tau - tau_true) < 0.10, tau
    assert abs(theta - theta_true) < 0.12, theta
    print("self-test OK")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit FOPDT (K,tau,theta) from CPAP T/R telemetry logs and compute SIMC PID gains"
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        help="Log file with T,... and R,... lines (default: stdin)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=0,
        help="Which R event to use as the RPM step (0-based, default 0)",
    )
    parser.add_argument(
        "--step-ms",
        type=int,
        default=None,
        help="Step time in ms (overrides R-line timestamp)",
    )
    parser.add_argument(
        "--delta-rpm",
        type=float,
        default=None,
        help="RPM step size (default: inferred from consecutive R lines)",
    )
    parser.add_argument("--pre", type=float, default=1.0, help="Seconds before step (default 1)")
    parser.add_argument("--post", type=float, default=4.0, help="Seconds after step (default 4)")
    parser.add_argument(
        "--tau-c",
        type=float,
        default=0.60,
        help="SIMC closed-loop time constant (default 0.60 s)",
    )
    parser.add_argument(
        "--ksqrt",
        type=float,
        default=KSQRT_DEFAULT,
        help=f"sqrt-FF K for zone scaling (default {KSQRT_DEFAULT:.0f})",
    )
    parser.add_argument("--plot", action="store_true", help="Show fit plot")
    parser.add_argument("--save-plot", type=Path, help="Save fit plot to PNG")
    parser.add_argument("--self-test", action="store_true", help="Run internal sanity check")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0

    if args.input is None:
        text = sys.stdin.read()
        src = "<stdin>"
    else:
        if not args.input.is_file():
            print(f"Not found: {args.input}", file=sys.stderr)
            return 1
        text = args.input.read_text(encoding="utf-8", errors="replace")
        src = str(args.input)

    telem, bench, fmt = parse_log(text)
    if not telem:
        print(f"No T,... lines in {src}", file=sys.stderr)
        print(
            "Tip: use --record for raw lines:\n"
            "  python cpap_telemetry_plot.py --port COM13 --monitor "
            "--record bench_raw.log --monitor-time 60",
            file=sys.stderr,
        )
        return 1

    if "monitor" in fmt:
        print(f"Note: parsed {fmt}")
        print("      For a tighter fit, re-capture with --record (50 Hz raw T/R lines).\n")

    try:
        step_ms = resolve_step_ms(bench, args.step, args.step_ms)
        delta_rpm = bench_delta_rpm(bench, args.step, args.delta_rpm)
        t, p = extract_window(telem, step_ms, args.pre, args.post)
        k, tau, theta, p0, rmse = fit_fopdt(t, p, delta_rpm)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    p_ss = float(np.mean(p[t > 0.5 * float(t[-1])])) if np.any(t > 0) else float(p[-1])
    fit = FopdtFit(
        k_cmh2o_per_rpm=k,
        tau_s=tau,
        theta_s=theta,
        delta_rpm=delta_rpm,
        p0_cmh2o=p0,
        p_ss_cmh2o=p_ss,
        step_ms=step_ms,
        rmse_cmh2o=rmse,
        n_samples=len(t),
    )
    p_oper = max(0.5 * (p0 + p_ss), 1.0)
    gains = simc_pid(k, tau, theta, args.tau_c)
    zone_gains = scaled_zone_gains(gains, p_oper, args.ksqrt)

    print_report(fit, args.tau_c, gains, zone_gains, args.ksqrt)

    if args.plot or args.save_plot:
        maybe_plot(
            t,
            p,
            fit,
            show=args.plot,
            out_path=str(args.save_plot) if args.save_plot else None,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
