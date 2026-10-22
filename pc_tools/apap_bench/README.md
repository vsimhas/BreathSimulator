# APAP bench tools

Bench harnesses and analysis used to test the CPAP's APAP (AutoSet) against
the BreathSimulator, and to compare it with a ResMed AirSense 11 on the same
rig. Results and conclusions are in CPAP_InitialCode
`docs/APAP_FINDINGS_2026-10-12_breathsim.md`. These scripts produced its
evidence.

Every script finds its own folder at run time. Code lives here, and run data is
read from and written to `runs/`.

## Rig

```
CPAP (COM13) or AirSense 11 -> hose -> vent -> external sensor (COM4)
  -> collapsible chamber (squeeze blower OFF) -> simulator box (COM15) -> 2 L lung
```

Ports and requirements:

- **COM15 (simulator):** opened by `phase1.py` on import. Its first command-line
  argument would be taken as the port, so harnesses pop their own arguments
  first.
- **COM4 (external sensor):** used by `ext_sensor.py`. Power-cycle its board if
  the liveness check says "frozen", i.e. one value repeated at ~12 Hz.
- **Python packages:** `pyserial`, `numpy`, `matplotlib`. No EDF library is
  needed: `airsense_edf.py` is self-contained.

## Harnesses (drive the hardware)

| script | what it runs | output |
|---|---|---|
| `apap_trial8c.py` | **reference APAP trial**: simulator breathing first, then CPAP AutoSet 8-12; 7 apneas, 50/55/60 % hypopneas, arousal apnea (955 s) | `runs/apap_trial8c_*` |
| `apap_trial8b.py` | the same script with the R11-build probe | `runs/apap_trial8b_*` |
| `apap_hyp4555.py` | 3 × 45 % and 3 × 55 % hypopneas (hypopnea threshold test) | `runs/apap_hyp4555_*` |
| `apap_flshape.py` | flattened breaths at full volume: plateau 90 %, M 90 %, plateau 60 % | `runs/apap_flshape_*` |
| `airsense_run.py 8c\|threshold\|flshape\|obstruct` | the same scripts with an **AirSense 11** in place of the CPAP; `obstruct` = five 30 s apneas, 2/4/5 with the chamber squeezed by hand on PC beeps (low = get ready, long high = squeeze MAX, two short = OFF) | `runs/airsense_<script>_<HHMM>_raw.pkl` |
| `spike_source.py` | 3-sensor capture: CPAP / external / simulator flow (found the R6 needles) | `runs/spike_*` |
| `leak_source.py` | 3-sensor capture of normal / hypopnea breathing (R9) | `runs/leak_*` |
| `total_source.py` | as `leak_source.py`, plus the CPAP's total flow (R10) | `runs/total_*` |

How the CPAP harnesses behave:

- **They talk to the CPAP over COM13.** They start therapy, restore the CPAP's
  original mode, target and range afterwards, and switch its motor telemetry
  back off.
- **Safety guards:**
  - CPAP supply current > 0.8 A held for 0.7 s;
  - Vbus < 21 V;
  - drive fault or mask-off;
  - simulator commanded above 36 000 rpm;
  - the **FOT watchdog**: continuous oscillation > 60 s, which the current guard
    can't see;
  - sensor liveness.
- **Every abort or finish takes the safe path:** verified simulator ramp-down,
  then CPAP setpoint 4 cmH₂O, then stop below 14.5k rpm. An unknown CPAP status
  also stops it.
- **Firmware probe:** each harness sends one `apap set ...` with the build's own
  default value and refuses to run unless it is acknowledged.
- `add_fot_watchdog.py <harness.py>` adds the FOT watchdog to an older harness.

`airsense_run.py` never talks to the AirSense:

1. Start and stop therapy by hand. The harness detects therapy from the external
   pressure (> 5 cmH₂O).
2. Settings: AutoSet 8-12, EPR off, ramp and AutoRamp off, response standard.
3. Afterwards copy the SD card's `DATALOG/<date>` folder to
   `BreathSimulator/airsense_data/` and run `airsense_compare.py`.

## Analysis (offline)

| script | use |
|---|---|
| `airsense_compare.py <raw.pkl> <DATALOG dir or files> [--ours <apap raw.pkl>]` | AirSense events, pressure and FL index vs the simulator's ground truth (and our CPAP). Aligns the AirSense clock on flow. Pass **only one session's files** when a date folder holds several. |
| `airsense_edf.py <file.edf>...` | EDF / EDF+ reader (ResMed EVE / BRP / PLD); prints the signals and annotations |
| `test_airsense_compare.py` | self-test: fake SD-card files with a known clock offset, built from `runs/total_raw.pkl` |
| `cpap_fl_report.py <apap raw.pkl>` | per flattened-breath block: FL scoring, pressure target onset → end → +60 s, rise/fall rates |
| `hyp_ratio.py <apap raw.pkl>` | per hypopnea: depth as the CPAP saw it vs normal, and what was scored |
| `epr_replay.py <apap raw.pkl>` | replays the CPAP's EPR breath detector on the logged 50 Hz patient flow (approximate: 50 Hz, not 200 Hz) |
| `needle_count.py <apap raw.pkl>` | counts flow-sensor needles and tests them against the 17.75 s clock-beat grid |
| `spike_analysis.py`, `leak_analysis.py`, `total_analysis.py` | analysis for the three 3-sensor captures (re-run with the harness's `--analyze`, or import) |

## Data

- **`runs/`** holds the raw pickles, logs, plots and CSVs of every run cited in the
  findings doc (trials 7-8c, the threshold test, our CPAP's flattened-breath run,
  both AirSense runs, and the three 3-sensor captures).
  - That is **~60 MB, mostly the `*_raw.pkl` files**.
  - Decide whether those go in git, Git LFS, or stay local; the scripts and small
    files can be committed without them.
- **The AirSense SD-card files** are in `BreathSimulator/airsense_data/20261020`.
  Session `190837` is the 8c run; `203445` is the flattened-breath run.
