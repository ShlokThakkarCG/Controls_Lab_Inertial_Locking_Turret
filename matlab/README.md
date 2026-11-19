# MATLAB tools

Run from the `matlab/` folder (or add it to the path).

| What | How |
|---|---|
| Live dashboard | `start_dashboard("COM7")`. Close any Python console first: only one program can own the port |
| Replay a saved run (no hardware) | In the dashboard: **Replay a log…**, or `app = start_dashboard; app.loadReplay("../logs/<run>.csv")` |
| Report figures for one run | `S = turret_report("../logs/<run>.csv")`. Saves PNGs to `analysis/matlab_<run>_*.png` and returns the per-mode table |
| Load a log as a struct | `L = turret_load_log("../logs/<run>.csv")` |

## Dashboard

- **3D view:** the turret moves with the hardware. Green ray = locked direction (fixed in space). Red dashed ray = barrel direction measured by the barrel IMU.
- **Plots** (last 15 s):
  - base rotation vs barrel pointing error (green band = locked)
  - error components
  - joint command vs measured
- **Live score:** RMS and max pointing error, base motion, and % reduction over the last 10 s of locked data.
- **Controls:**
  - LOCK (same as the BOOT button)
  - mode (0 HOLD, 1 FF, 2 FF+LEAD, 3 FULL, 4 FB only)
  - lead, feedback Kp/Ki, center, torque off
- **Record:** writes `logs/<stamp>_<tag>.csv` + `_raw.txt` in the same format as `tools/turret_console.py`, so every analysis tool works on dashboard recordings.

## Files

| File | Role |
|---|---|
| `TurretMath.m` | Frame conventions and kinematics (identical to the firmware) |
| `TurretLink.m` | Serial link: parses H/D/S/E telemetry, sends commands, records |
| `TurretState.m` | Lock reference, display pose, rolling buffers, live metrics (no UI) |
| `TurretModel3D.m` | Animated 3D turret model |
| `TurretDashboard.m` | The app |
| `turret_report.m`, `turret_load_log.m` | Offline analysis |
| `tests/` | Headless checks: `test_offline.m` (no hardware), `test_live_link.m`, `test_dashboard_replay.m` |
| `sim_1.m` | Earlier pure simulation (kept for reference) |
