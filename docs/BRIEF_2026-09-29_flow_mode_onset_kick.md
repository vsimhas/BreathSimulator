# Brief — Flow-mode inspiratory onset kick (29 September 2026)

Written from the CPAP_InitialCode session after the first ISO 80601-2-70 dynamic-pressure
pre-check. This is a problem report with a proposed direction, not an authorization to edit,
flash or commit — the BreathSimulator session owns those decisions.

## 1. Symptom

With `control flow`, `waveform sine`, `timing rate_ie`, `ie 1`, `tidal 500`, `rpm_base 8000`,
inspiration does not start like a sine. Flow jumps at the start of every breath and then
plateaus below the target peak.

Measured with an external flow/pressure sensor at the CPAP patient port (between the CPAP vent and
the simulator), 20 BPM, 5 Hz low-pass, median of 14 breaths:

| CPAP set | Vt | Peak flow | Max dQ/dt at onset | ISO sine dQ/dt (500 mL, 20 BPM) |
| --- | --- | --- | --- | --- |
| 4 cmH₂O | 373–404 mL | 30–33 L/min | **131 L/min/s** | 66 L/min/s |
| 8 cmH₂O | 292–305 mL | 20–21 L/min | **104 L/min/s** | 66 L/min/s |
| 12 cmH₂O | 259–265 mL | 17 L/min | 66 L/min/s | 66 L/min/s |

(Vt ranges are external sensor / CPAP internal sensor; they agree within ~10%.)

A 500 mL half-sine at 20 BPM, I:E 1:1 (Ti 1.5 s) has peak 31.4 L/min and a maximum slope of
Qpk·π/Ti ≈ 66 L/min/s — at the zero crossing. At 4 cmH₂O the simulator starts at twice that.
In the earlier run at `tidal 325` the onset was ~120 L/min/s against a sine's ~43 (2.8×).

At 12 cmH₂O the onset looks right only because the breath is volume-limited (see §4).

## 2. Why it matters

The CPAP's pressure swing in the ISO test is dominated by a single notch at the start of
inspiration (~0.8 cmH₂O deep, ~0.3 s wide); the rest of the breath is held within ~±0.2 cmH₂O.
Across the three pressures the swing tracks the onset slope: 0.82 → 0.96 → 1.23 cmH₂O p-p as
onset rises 66 → 104 → 131 L/min/s. So the kick, not the CPAP, currently decides the ISO pass/fail
(ARTP limit < 1.0 cmH₂O p-p). The ISO test specifies a sinusoid; this rig needs to produce one
before its numbers can be used.

## 3. Cause (read from the code)

`BreathSim_FlowControlRpm()` in `CM7/Core/Src/breath_sim.c`:

```c
const float unsat = rpm_base_f + ff + (g_params.flow_kp * err) + g_ctrl_integ_rpm;
```

- During expiration the integrator is deliberately **frozen, not zeroed** (comment at the
  `inspiring == 0U` branch), and the command decays to `rpm_base`.
- At the first inspiratory tick the target is 0 (half-sine at u = 0) and `flow_ff_rpm_per_lpm`
  defaults to 0, so the output is `rpm_base + g_ctrl_integ_rpm` — the whole learned operating
  point, applied as a **step** at t = 0.
- The only thing shaping that step is `BREATH_SLEW_RISE_RPM_PER_S` = 150 000 rpm/s, so a
  ~15 000 rpm integrator value lands in ~0.1 s.
- Flow then overshoots the (still small) target, `kp·err` goes negative and pulls it back —
  giving the fast rise followed by a plateau below peak.

With FF = 0 the integrator carries the entire inspiratory drive, so it is large, and it is added as
a constant offset instead of following the shape.

Not yet confirmed on the rig: the logged `rpm_cmd` at inspiration start should show the step
directly.

## 4. Separate finding — test-lung capacity (not a firmware bug)

The 1 L test lung sits at CPAP pressure plus the simulator's `rpm_base` offset (~9.4 cmH₂O at
14 500, ~3 at 8000 by the fan law) and is nearly full by 12 cmH₂O. Delivered Vt at 20 BPM,
`tidal 500`: ~390 mL at 4, ~300 at 8, ~260 at 12 cmH₂O (the same ~260 at 12 whether `tidal` was
325 or 500). A 500 mL ISO breath needs a larger lung (2–3 L) or a sealed rigid volume (~40 L
drum ≈ 28 mL/cmH₂O adiabatic, so ~18 cmH₂O swing for 500 mL — estimate, untried). Raising
`rpm_base` makes this worse.

## 5. Proposed direction (for this session to evaluate)

Make the integrator scale the shape rather than offset it. In order of effort:

1. **Minimal:** multiply the integrator term by the envelope during inspiration,
   `rpm_base + ff + kp·err + g_ctrl_integ_rpm * g_envelope`, so its contribution is zero at onset
   and full at peak. Integration itself can stay as is.
2. **Better:** use the feed-forward (`flow_ff_rpm_per_lpm`, currently 0) so that the drive
   follows `g_q_target_lpm`, which is zero at onset by construction, and leave only a small
   correction to PI.
3. **Best fit for a repeating breath:** breath-to-breath learning. Adjust an amplitude (or a
   per-phase correction table) once per breath from the Vt/peak error, and command
   `rpm_base + A · shape(u)` within the breath. Nothing then acts as a step at the phase boundary.

Secondary: expiration is passive (exponential decay to `rpm_base`), not the second half of a
sinusoid. ISO specifies a sinusoid for the whole cycle. Worth noting in any test report, and a
candidate for a later active-expiration mode.

## 6. Acceptance check

At 20 BPM, `ie 1`, `tidal 500`, CPAP off or at 4 cmH₂O (where the lung has room):

- onset max dQ/dt within ±20% of Qpk·π/Ti (≈ 66 L/min/s for 500 mL);
- per-breath inspiratory flow within ~10% of peak (RMS) of the half-sine target;
- Vt within ±5% of target, and no SLEW_LIMITED / FLOW_SAT on steady breaths.

The CPAP-side script (`iso_dynamic.py`, CPAP_InitialCode session scratchpad) reports Vt, peak
flow and the pressure trace at the patient port and can be re-run once this is in.
