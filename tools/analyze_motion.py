"""
analyze_motion.py - what does a hand-held demo ask of the turret?

From a probe log (tared while the base was at rest, servos at az=el=0):
  * base attitude change (yaw / pitch / roll) and angular rates
  * the joint angles the turret WOULD need to keep the barrel pointing at the
    original direction (inverse kinematics), and their speeds
  * how often those needs exceed the joint limits or a speed budget
Saves a figure next to the log in analysis/.

    python tools/analyze_motion.py logs/<run>.csv
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AZ_LIM = (-75, 110)
EL_LIM = (-20, 55)

path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
t = (c["t_ms"] - c["t_ms"][0]) / 1000.0


def quat_to_R(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


Rb = np.array([quat_to_R(*[c[f"bq_{k}"][i] for k in "wxyz"]) for i in range(len(t))])
# The firmware's tare (z) happened at the start; use the first sample as reference.
R0 = Rb[0]
target_world = R0 @ np.array([0.0, 1.0, 0.0])           # barrel forward at tare (az = el = 0)
need = np.einsum("nji,j->ni", Rb, target_world)          # target expressed in the current base frame
need_az = np.degrees(np.arctan2(-need[:, 0], need[:, 1]))
need_el = np.degrees(np.arctan2(need[:, 2], np.hypot(need[:, 0], need[:, 1])))

yaw, pitch, roll = c["base_yaw"], c["base_pitch"], c["base_roll"]


def rate(x):
    # central difference on unwrapped angle, light smoothing (5 samples = 0.1 s)
    d = np.gradient(np.unwrap(np.radians(x)), t)
    k = np.ones(5) / 5
    return np.degrees(np.convolve(d, k, mode="same"))


def dominant_freq(x):
    x = x - x.mean()
    dt = np.median(np.diff(t))
    f = np.fft.rfftfreq(len(x), dt)
    p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    m = (f > 0.1) & (f < 10)
    cum = np.cumsum(p[m]) / p[m].sum()
    return f[m][np.argmax(p[m])], f[m][np.searchsorted(cum, 0.9)]


print(f"{os.path.basename(path)}: {t[-1]:.1f} s, {len(t)} samples\n")
print(f"{'signal':12s} {'min':>7s} {'max':>7s} {'p95|rate|':>10s} {'max|rate|':>10s} {'peak Hz':>8s} {'90% power <':>12s}")
for name, x in [("base yaw", yaw), ("base pitch", pitch), ("base roll", roll),
                ("need az", need_az), ("need el", need_el)]:
    r = np.abs(rate(x))
    fp, f90 = dominant_freq(x)
    print(f"{name:12s} {x.min():+7.1f} {x.max():+7.1f} {np.percentile(r, 95):8.0f}/s {r.max():8.0f}/s {fp:8.2f} {f90:10.2f} Hz")

out_az = np.mean((need_az < AZ_LIM[0]) | (need_az > AZ_LIM[1])) * 100
out_el = np.mean((need_el < EL_LIM[0]) | (need_el > EL_LIM[1])) * 100
print(f"\nneeded joint angle outside limits: az {out_az:.1f}% of the time, el {out_el:.1f}% of the time")
for budget in (100, 200, 300):
    fast = np.mean((np.abs(rate(need_az)) > budget) | (np.abs(rate(need_el)) > budget)) * 100
    print(f"needed joint speed above {budget} deg/s: {fast:.1f}% of the time")

# ---- figure ----
fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
ax[0].plot(t, yaw, label="yaw (+ = nose left)")
ax[0].plot(t, pitch, label="pitch (+ = nose up)")
ax[0].plot(t, roll, label="roll (+ = right side down)")
ax[0].set_ylabel("base attitude [deg]")
ax[0].set_title("Hand-held base motion and what the turret must do to hold its aim")
ax[0].legend(loc="upper right", fontsize=8)
ax[1].plot(t, need_az, label="azimuth needed")
ax[1].plot(t, need_el, label="elevation needed")
for lo, hi, col in [(*AZ_LIM, "C0"), (*EL_LIM, "C1")]:
    ax[1].axhline(lo, color=col, ls=":", lw=0.8)
    ax[1].axhline(hi, color=col, ls=":", lw=0.8)
ax[1].set_ylabel("joint angle [deg]")
ax[1].legend(loc="upper right", fontsize=8)
ax[2].plot(t, rate(need_az), label="azimuth speed needed")
ax[2].plot(t, rate(need_el), label="elevation speed needed")
ax[2].set_ylabel("joint speed [deg/s]")
ax[2].set_xlabel("time [s]")
ax[2].legend(loc="upper right", fontsize=8)
for a in ax:
    a.grid(alpha=0.3)
os.makedirs(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(path))), "analysis"), exist_ok=True)
png = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(path))), "analysis",
                   os.path.splitext(os.path.basename(path))[0] + ".png")
fig.tight_layout()
fig.savefig(png, dpi=120)
print(f"\nfigure: {png}")
