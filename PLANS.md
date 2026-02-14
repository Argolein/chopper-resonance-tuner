# PLANS.md — Improved Chopper Tuner (Klipper) for TMC2240/5160 (Snapmaker U1)

## Goal
Implement a Klipper extension that auto-calibrates **SpreadCycle** parameters for **TMC2240** using accelerometer measurements on **Snapmaker U1**.

**North-star:** in **minimum time**, get the **best repeatable result** under real motion limits.
- Use the upstream **adaptive** strategy from the eoyilmaz fork as the baseline (avoid brute-force “measure every frequency” loops).
- Still measure **all relevant registers** (TBL/TOFF/HSTRT/HEND/TPFD) end-to-end with **one command**.


## Efficiency principles (do not regress)
1. **No brute force frequency scans.** Prefer adaptive peak finding / limited sampling.
2. Cache reusable measurements within a run (baseline, speed sweep curve, repeated points).
3. QUICK mode must complete fast (target: minutes, not hours) while staying safe and producing usable params.
4. Always prefer “good enough + validated” over exhaustive search.

## Constraints
- Must run on U1 host (embedded Linux). Assume **no third-party Python packages** (no numpy/scipy).
- Python is “fairly current” on U1; log detected Python+Klipper versions at runtime into JSON.
- Uses Klipper’s existing LIS2DW config as-is (do **not** change data_rate/ODR or accelerometer config).
- LIS2DW is on **Tool 0** (toolchanger); tuning must select Tool 0.
- Safety: must respect axis limits + inset; homing via `G28` is required by default.

## Machine facts (from printer.cfg)
- Kinematics: **CoreXY**
- X travel: `0..271` mm
- Y travel: `0..335` mm
- Preferred “safe scan box”: if `[bed_mesh]` exists, use `mesh_min..mesh_max` (here `3..267` on X/Y) for all tuning moves to avoid docks/edges.
- Drivers: **TMC2240** on X/Y only.
- Motion limits: `max_velocity=500`, `max_accel=20000`, `square_corner_velocity=8`.
- Accelerometer: `[lis2dw e0_lis2dw]` (tool0), and `[resonance_tester] accel_chip: lis2dw e0_lis2dw`.

## Deliverables
- `klippy/extras/chopper_tune.py` (single file).
- GCode: `CHOPPER_TUNE` (+ `CHOPPER_TUNE_DEBUG` optional).
- Writes tuned fields to `printer.cfg` via `configfile.set()`, then prompts `SAVE_CONFIG`.
- JSON log of all measurements + chosen params + validation results.

## Storage policy (U1 overlay FS)
All generated files MUST go under:
- `/data/gcodes/chopper-tuner/`

If the directory does not exist, create it. Do not write large files to `/tmp`.

Suggested layout:
- `logs/` (runtime logs)
- `results/` (JSON/CSV measurements, chosen params, validation)
- `plots/` (optional; default off)
- `runs/<timestamp>/` (one folder per run; symlink `latest` optional)

## Measurement & scoring
### Signal pipeline
- Baseline: standing ADXL mean (configurable dwell).
- Magnitude: `sqrt(x²+y²+z²)` after baseline subtraction.
- Low-pass filter: **pure-Python** IIR (single-pole or biquad) with configurable cutoff.
- Percentile trim (20–80%) + median computed with `statistics` / sorting (pure Python).

### Bidirectional
For each measurement point: run forward + reverse, score = `max(fwd, rev)`.

### Multi-speed
Select 3 speeds; score = `0.50*peak + 0.30*mid + 0.20*high`.

### Optional noise penalty
Heuristic penalty for low estimated chopper frequency (configurable thresholds/weights).
Default should be conservative until validated on U1.

## Single-command flow
### Phase 0 — Setup
- Select **Tool 0** via `T0` (required; accelerometer is configured on tool0 MCU as `lis2dw e0_lis2dw`). If `T0` fails, abort.
- Home via `G28 X Y` (required), then move to safe start.
- Detect driver (target 2240/5160; fail early on others unless overridden).
- Resolve axis→steppers (CoreXY diagonal isolation supported).
- Compute travel distance for constant-speed sampling.
- Measure static baseline using existing ADXL config.
- (Optional) apply “static best-practice” fields behind `APPLY_STATIC=1`.

### Phase 1 — Find resonance speeds (adaptive)
- Use the upstream adaptive strategy (eoyilmaz fork): identify resonance peaks with minimal sampling (no dense brute-force sweep).
- Measure magnitude at candidate speeds using current config (no register changes).
- Pick:
  - `speed_peak` (strongest peak),
  - `speed_mid` (2nd peak or midpoint),
  - `speed_high` (~80% max).

### Phase 2 — Coarse scan (TBL×TOFF)
- Grid: `TBL 0..3` × `TOFF 1..8` (=32).
- Evaluate at 3 speeds, bidirectional (192 measurements).
- Rank by multi-speed score + optional noise penalty.
- Keep Top-K (default K=3).

### Phase 3 — Fine scan (HSTRT×HEND) on Top-K
- Sweep valid pairs with constraint `HSTRT+HEND <= 16`.
- Measure at `speed_peak`, bidirectional.
- Choose best overall.

### Phase 4 — TPFD scan (2240/5160 only)
- Sweep `TPFD 0..15` at `speed_peak`, bidirectional.
- Skip for drivers without TPFD.

### Phase 5 — Validation
- Baseline = “current config at start of command”.
- Measure baseline vs tuned at all 3 speeds, bidirectional.
- Report improvement per speed + overall.

## GCode interface
```gcode
CHOPPER_TUNE AXIS=X
  [TOOL=0]       # implemented as `T0` on U1
  [NOISE_PENALTY=none|moderate|strict]
  [ITERATIONS=1]
  [QUICK=0|1]
  [ACCEL_CHIP=lis2dw e0_lis2dw]  # matches [resonance_tester]
  [LPF_CUTOFF_HZ=150]
  [INSET=10]
  [BASELINE_DWELL=5.0]
  [HOME=1]        # default required (can allow HOME=0 later if you insist)
  [APPLY_STATIC=0]
  [LOG_PATH=/data/gcodes/chopper-tuner]  # required on U1
```

### QUICK mode (fast, still end-to-end)
- Keep full register coverage (TBL/TOFF/HSTRT/HEND/TPFD), but reduce measurements:
  - Phase 1: adaptive only (fewer samples).
  - Phase 2: evaluate only `speed_peak`.
  - Phase 3: only Top-1 from Phase 2.
  - Phase 4: keep TPFD sweep (small).
- Keep bidirectional unless explicitly disabled.

## Code structure (single file, clear sections)
- Driver detection + register apply/read helpers
- ADXL measurement context manager
- Motion/coord generator (forward/reverse)
- Signal processing (magnitude calc)
- Phased orchestrator
- Persistence (printer.cfg + JSON)

## Acceptance criteria
- Loads as Klipper extra; registers `CHOPPER_TUNE`.
- Runs end-to-end on U1 without crashes; produces JSON output.
- Writes **all artifacts** under `/data/gcodes/chopper-tuner/`.
- Writes tuned fields to `printer.cfg` safely.
- Respects limits/inset/travel-distance constraints.
- CoreXY isolation correct.
- Validation can show improvement or “no improvement” with safe fallback.
- Default settings are practical on U1; QUICK mode finishes in minutes (not hours).

## Repo context (Codex guidance)
- Work is done in a **fork** of `eoyilmaz/chopper-resonance-tuner`, which is itself based on `MRX8024/chopper-resonance-tuner`.
- The original upstream brute-forced frequency measurement and could take **hours**.
- The eoyilmaz fork already improved runtime via an **adaptive** approach; treat that as the baseline and optimize further **without losing accuracy**.
- Goal is “fast best result”: fewer measurements, better point selection, but still measure all relevant registers in one run.
- Prefer small, reviewable diffs: isolate U1/toolchanger specifics (`T0`, storage path, bounds) behind helpers.

## Defaults locked for U1
- Homing: use `G28 X Y` (no Z).
- Tool selection: always `T0` (deterministic, no retries/checks needed).
- Motion bounds: restrict all tuning moves to the `[bed_mesh]` area (`mesh_min..mesh_max` = `3..267` on both X/Y), plus an `INSET` margin.
- Travel distance: compute per move so constant-speed segment fits fully inside bounds; if requested distance does not fit, reduce distance automatically (do not exceed bounds).

## Decisions
- Scope: implement the `PLANS.md` phased model inside the existing `CHOPPER_TUNE` command (no parallel `*_U1` command). (User: 1A)
- GCode interface is `CHOPPER_TUNE` (no extra alias commands).
- Dependencies: runtime must be Python stdlib only (no numpy/scipy/plotly/etc). (User: 2A)
- CoreXY UI: keep `AXIS=X|Y` user interface. (User: 4A)
- Logging: write summary JSON only by default; raw samples only behind an explicit flag (e.g. `RAW=1`). (User: 6A)
- Robustness: fine (HSTRT/HEND) and TPFD phases are measured at `peak` first, then the top candidates are re-scored at `high` (and also `mid` when not in QUICK mode) to avoid "single-speed wins".
- Firmware env: Python on U1 is `3.11.8`. (User report)
- Homing/tool order: run `G28 X Y` first, then select tool 0. (User: 2)
- Accelerometer: U1 firmware includes `klippy/extras/lis2dw.py` that implements `start_internal_client()` and uses `adxl345.AccelQueryHelper` samples. (Repo analysis: `SnapermakerU1-Firmware`)
- Toolchange syntax: U1 firmware commonly issues toolchange as `T{n} A0` (e.g. in `resonance_tester.py`). Prefer matching that unless it proves incompatible. (Repo analysis: `SnapermakerU1-Firmware`)

## Open questions
- (resolved) Storage path: `/data` is a symlink to `/userdata`, and `/data/gcodes` is owned by `lava:lava` and is writable by Klipper (`-u lava`). Use `/data/gcodes/chopper-tuner/` exclusively.
- (resolved) Tool selection command: use `T0 A0` (matches U1 firmware conventions and user preference).

## Implementation status
- [ ] Not started
- [ ] In progress
- [x] Done
