# Snapmaker U1 Chopper Resonance Tuner (Klipper)

U1-focused fork for tuning **TMC2240** SpreadCycle chopper parameters on **Snapmaker U1** using the built-in **LIS2DW** accelerometer on **Tool 0**.

Hard constraints on U1:
- Klipper runtime is **Python stdlib only** (no numpy/scipy).
- Artifacts are written only under `/data/gcodes/chopper-tuner/` (`/data` is a symlink to `/userdata` on U1).
- `SAVE_CONFIG` is not usable on U1 (overlay FS + included vendor configs). Persist by using a late override config file.

## Installation (U1)

Put this repo onto the printer first. The paths below assume:

- Save location: `/home/lava/chopper-resonance-tuner/`

Manual install (symlink into Klipper):

```sh
# on the U1
ln -sf /home/lava/chopper-resonance-tuner/chopper_tune.py /home/lava/klipper/klippy/extras/chopper_tune.py
cp -f /home/lava/chopper-resonance-tuner/chopper_tune.cfg /home/lava/printer_data/config/chopper_tune.cfg
```

If you use a different repo path, update the `ln -sf` / `cp -f` paths accordingly.

Ensure your config includes `chopper_tune.cfg`.

If you include from `printer.cfg`:

```ini
[include chopper_tune.cfg]
```

If you include from a file inside `/home/lava/printer_data/config/extended/klipper/*.cfg`,
includes are resolved relative to the including file, so use:

```ini
[include ../../chopper_tune.cfg]
```

Restart Klipper.

## Recommended Workflow (U1)

1. Home and select tool 0:

```gcode
G28 X Y
T0 A0
```

2. Tune X, then tune Y (keep X tuned while tuning Y; CoreXY uses both motors):

```gcode
CHOPPER_TUNE AXIS=X QUICK=1 HOME=0
CHOPPER_TUNE AXIS=Y QUICK=1 HOME=0
```

3. Persist results (U1, no `SAVE_CONFIG`):

Create a late override file that is included last, for example:
`/home/lava/printer_data/config/extended/klipper/99_chopper_tune_overrides.cfg`

Paste the tuned fields printed at the end of each run (or take them from `run.json`):

```ini
[tmc2240 stepper_x]
driver_tbl:  <X>
driver_toff: <X>
driver_hstrt: <X>
driver_hend: <X>
driver_tpfd: <X>

[tmc2240 stepper_y]
driver_tbl:  <Y>
driver_toff: <Y>
driver_hstrt: <Y>
driver_hend: <Y>
driver_tpfd: <Y>
```

Restart Klipper to apply the override.

4. Effect on sensorless homing:
Sensorless homing might need re-adjusting.

Example:
```ini
[tmc2240 stepper_x]
driver_SGT: 2 # Default is 1 / higher = less sensitivity 

[tmc2240 stepper_y]
driver_SGT: 2 # Default is 1 / higher = less sensitivity
```

## Command Reference (U1)

Main command:

```gcode
CHOPPER_TUNE AXIS=X QUICK=0 HOME=0
CHOPPER_TUNE AXIS=Y QUICK=0 HOME=0
```

Parameters:

- `AXIS=X|Y`: which axis to tune (U1 CoreXY tunes `stepper_x` or `stepper_y`)
- `QUICK=0|1`: runtime reduction knob
  When `QUICK=1`: coarse scan scores only at the peak speed, Top-K=1, fewer speed samples (fast, recommended).
  When `QUICK=0`: coarse scan scores peak+mid+high and keeps Top-K=3 (slower, more robust).
  In both modes, the fine scan (HSTRT/HEND) and TPFD scan are measured at `peak` first, then the top candidates are re-scored at `high` (and also `mid` when `QUICK=0`) to avoid picking a setting that only helps at one speed.
- `HOME=0|1`: if `1`, the command runs `G28 X Y` internally; recommended to do manual `G28 X Y` and use `HOME=0`
- `TOOL=0`: must be tool 0 (U1 uses `T0 A0`)
- `INSET=<mm>`: margin inside the safe X/Y box used for all tuning moves
- `BASELINE_DWELL=<seconds>`: how long to measure static baseline noise
- `LPF_CUTOFF_HZ=<hz>`: low-pass cutoff applied to the magnitude signal (noise smoothing)
- `ITERATIONS=<n>`: repeats each measurement `n` times and averages (slower, can reduce variance)
- `NOISE_PENALTY=none|moderate|strict`: optional penalty against low chopper frequencies
  Candidates get a multiplicative factor `>= 1.0` applied to their score (lower is better).
  Penalty applies when estimated `f_chop < 20 kHz`.
  `moderate` uses weight `0.10`, `strict` uses weight `0.25`.
  Internals (current implementation): `factor = 1 + weight * ((20000/f_chop) - 1)` when `f_chop < 20000`.
  Use `none` if you only care about the accelerometer score; use `moderate/strict` if you want to bias away from audible chopper frequencies.
- `RAW=0|1`: when `1`, writes a small amount of raw CSV (baseline+tuned at peak speed, fwd+rev; 4 files total)
- `LOG_PATH=/data/gcodes/chopper-tuner`: output root (must be exactly this on U1)

## Output / Logs (U1)

Each run writes:

- JSON summary: `/data/gcodes/chopper-tuner/runs/<timestamp>/results/run.json`
- Optional raw CSVs when `RAW=1`: `/data/gcodes/chopper-tuner/runs/<timestamp>/results/raw/*.csv`

At the end of the command, the console prints:
- tuned `driver_tbl/driver_toff/driver_hstrt/driver_hend/driver_tpfd`
- validation deltas (baseline minus tuned; positive means improvement)

## Upstream Notes

This started as a fork of `MRX8024/chopper-resonance-tuner` and `eoyilmaz/chopper-resonance-tuner`.
For the original generic documentation, see `wiki/wiki.md`.
