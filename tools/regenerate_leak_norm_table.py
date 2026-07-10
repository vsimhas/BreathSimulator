#!/usr/bin/env python3
"""Regenerate CM7/Core/Inc/leak_norm_table.h from bench leak-side P-Q CSV columns."""

import math
import sys
from pathlib import Path

MBAR_PER_CMH2O = 0.980665
OUT = Path(__file__).resolve().parents[1] / "CM7" / "Core" / "Inc" / "leak_norm_table.h"

# Default: user bench RPM sweep (leak-side pressure mbar, flow slm)
BENCH = [
    (1000, 0.008, 0.334), (2000, 0.046, 1.03), (3000, 0.12, 1.85), (4000, 0.234, 2.72),
    (5000, 0.387, 3.67), (6000, 0.586, 4.62), (7000, 0.823, 5.57), (8000, 1.106, 6.55),
    (9000, 1.433, 7.55), (10000, 1.806, 8.575), (11000, 2.221, 9.6), (12000, 2.676, 10.65),
    (13000, 3.176, 11.7), (14000, 3.716, 12.75), (15000, 4.306, 13.85), (16000, 4.926, 14.9),
    (17000, 5.616, 16), (18000, 6.336, 17.1), (19000, 7.096, 18.1), (20000, 7.911, 19.25),
    (21000, 8.756, 20.35), (22000, 9.636, 21.45), (23000, 10.586, 22.5), (24000, 11.561, 23.7),
    (25000, 12.586, 24.8), (26000, 13.686, 25.9), (27000, 14.786, 27), (28000, 15.936, 28),
    (29000, 17.086, 29.1), (30000, 18.386, 30.1), (31000, 19.636, 31.25), (32000, 20.886, 32.25),
    (33000, 22.236, 33.25), (34000, 23.636, 34.3), (35000, 25.086, 35.5),
]


def slm_to_mlps(q_slm: float) -> int:
    return max(0, int(round(q_slm * 1000.0 / 60.0)))


def fit_power_law(pts):
    import numpy as np

    p = np.array([x[0] for x in pts])
    q = np.array([x[1] for x in pts])
    k_sqrt = float(np.sum(q * np.sqrt(p)) / np.sum(p))
    log_p = np.log(p)
    log_q = np.log(q)
    a = np.vstack([np.ones_like(log_p), log_p]).T
    coef, _, _, _ = np.linalg.lstsq(a, log_q, rcond=None)
    alpha = float(coef[1])
    k_pow = float(math.exp(coef[0]))
    return k_pow, alpha, k_sqrt


def main():
    pts = [(p_mbar / MBAR_PER_CMH2O, q) for _, p_mbar, q in BENCH if p_mbar > 0.05]
    k_pow, alpha, k_sqrt = fit_power_law(pts)
    table = [0 if i == 0 else slm_to_mlps(k_pow * ((i * 0.1) ** alpha)) for i in range(621)]

    lines = []
    for i in range(0, 621, 15):
        chunk = ", ".join(str(table[j]) for j in range(i, min(i + 15, 621)))
        lines.append("    " + chunk + ",")

    text = f"""/**
  ******************************************************************************
  * @file    leak_norm_table.h
  * @brief   Bench-characterized leak reference flow vs mask pressure.
  *
  *  Source: leak-side P-Q from 4 mm orifice bench (22 mm tube, occluded end).
  *  Fit (power law): Q_slm = {k_pow:.5f} * P_cmH2O^{alpha:.5f}
  *  Sqrt fit:        Q_slm = {k_sqrt:.5f} * sqrt(P_cmH2O)
  *
  *  Index: 0..620 => mask pressure in 0.1 cmH2O steps (i / 10.0f cmH2O).
  *  Value: reference leak flow [ml/s] at STPD (BTPS applied at runtime).
  *  Regenerate: python tools/regenerate_leak_norm_table.py
  ******************************************************************************
  */

#ifndef LEAK_NORM_TABLE_H
#define LEAK_NORM_TABLE_H

#include <stdint.h>

#define LEAK_NORM_TAB_COUNT       621U
#define LEAK_NORM_PRESS_STEP_CMH2O 0.1f

#define LEAK_NORM_FIT_K_POW       {k_pow:.5f}f
#define LEAK_NORM_FIT_ALPHA       {alpha:.5f}f
#define LEAK_NORM_FIT_K_SQRT      {k_sqrt:.5f}f

static const uint16_t g_leak_norm_tab_mlps[LEAK_NORM_TAB_COUNT] =
{{
{chr(10).join(lines)}
}};

#endif /* LEAK_NORM_TABLE_H */
"""
    OUT.write_text(text, newline="\n")
    print(f"Wrote {OUT}")
    print(f"sqrt: Q = {k_sqrt:.5f} * sqrt(P_cmH2O)")
    print(f"pow:  Q = {k_pow:.5f} * P^{alpha:.5f}")


if __name__ == "__main__":
    main()
