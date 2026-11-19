"""
analyze_stab.py - score a turret_stab run.

Splits the log into segments of constant (locked, mode, lead) and reports for each
locked segment how much the base moved and how well the barrel held its aim:
  base_rot   = rotation angle of the base relative to its attitude at lock [deg]
  err_tot    = angle between the barrel IMU's forward axis and the locked direction [deg]
Also saves a figure in analysis/.

    python tools/analyze_stab.py logs/<run>.csv
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MODE_NAMES = {0: "HOLD", 1: "FF", 2: "FF+LEAD", 3: "FULL", 4: "FB"}

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
t = (c["t_ms"] - c["t_ms"][0]) / 1000.0
# base rotation magnitude since lock from the logged yaw/pitch/roll (small helper: compose)
yaw, pitch, roll = np.radians(c["base_yaw"]), np.radians(c["base_pitch"]), np.radians(c["base_roll"])
# R = Rz(yaw) Rx(pitch) Ry(roll); rotation angle = acos((trace - 1) / 2)
cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
trace = (cy * cr - sy * sp * sr) + (cy * cp) + (cp * cr)
base_rot = np.degrees(np.arccos(np.clip((trace - 1) / 2, -1, 1)))
rate = np.sqrt(c["bw_x"] ** 2 + c["bw_y"] ** 2 + c["bw_z"] ** 2)

key = np.stack([c["locked"], c["mode"], c["lead_ms"]], 1)
edges = [0] + [i for i in range(1, len(t)) if (key[i] != key[i - 1]).any()] + [len(t)]
print(f"{os.path.basename(path)}\n")
print(f"{'t0':>6s} {'dur':>5s} {'mode':8s} {'lead':>5s} | {'base rot max':>12s} {'base rate p95':>13s} | "
      f"{'err rms':>7s} {'err p95':>7s} {'err max':>7s} | {'reduction':>9s}")
segs = []
for a, b in zip(edges[:-1], edges[1:]):
    if c["locked"][a] != 1 or b - a < 50:
        continue
    s = slice(a + 20, b)          # skip the first 0.2 s after a switch
    e = c["err_tot"][s]
    br = base_rot[s]
    red = 100 * (1 - np.sqrt(np.mean(e ** 2)) / max(np.sqrt(np.mean(br ** 2)), 1e-6))
    m = int(c["mode"][a])
    segs.append((t[a], t[b - 1], m))
    print(f"{t[a]:6.1f} {t[b - 1] - t[a]:5.1f} {MODE_NAMES.get(m, m):8s} {c['lead_ms'][a]:5.0f} | "
          f"{br.max():10.1f}deg {np.percentile(rate[s], 95):10.0f}d/s | "
          f"{np.sqrt(np.mean(e ** 2)):7.2f} {np.percentile(e, 95):7.2f} {e.max():7.2f} | {red:8.0f}%")
print("\nreduction = how much smaller the barrel's pointing error is than the base's rotation (RMS)")

fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
ax[0].plot(t, base_rot, color="0.5", label="base rotation since lock")
ax[0].plot(t, c["err_tot"], color="C3", label="barrel pointing error")
ax[0].set_ylabel("angle [deg]")
ax[0].legend(loc="upper right", fontsize=8)
ax[0].set_title("Stabilizer run: base disturbance vs barrel pointing error")
ax[1].plot(t, c["cmd_az"], "C0", label="az command")
ax[1].plot(t, c["meas_az"], "C0--", lw=0.8, label="az measured")
ax[1].plot(t, c["cmd_el"], "C1", label="el command")
ax[1].plot(t, c["meas_el"], "C1--", lw=0.8, label="el measured")
ax[1].set_ylabel("joint [deg]")
ax[1].legend(loc="upper right", fontsize=8, ncol=2)
ax[2].plot(t, c["err_az"], "C0", label="err az (barrel frame)")
ax[2].plot(t, c["err_el"], "C1", label="err el")
ax[2].set_ylabel("error [deg]")
ax[2].set_xlabel("time [s]")
ax[2].legend(loc="upper right", fontsize=8)
for t0, t1, m in segs:
    for a in ax:
        a.axvspan(t0, t1, color=f"C{m}", alpha=0.07)
    ax[0].text((t0 + t1) / 2, ax[0].get_ylim()[1] * 0.92, MODE_NAMES.get(m, m), ha="center", fontsize=9)
for a in ax:
    a.grid(alpha=0.3)
root = os.path.dirname(os.path.dirname(os.path.abspath(path)))
os.makedirs(os.path.join(root, "analysis"), exist_ok=True)
png = os.path.join(root, "analysis", os.path.splitext(os.path.basename(path))[0] + ".png")
fig.tight_layout()
fig.savefig(png, dpi=110)
print(f"figure: {png}")
