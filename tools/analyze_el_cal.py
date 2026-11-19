"""Elevation calibration from a bidirectional 'tel' sweep log.
True elevation = barrel forward-axis elevation minus base pitch (both from gravity).
Each commanded tick should be visited once moving down and once moving up; the
two are averaged to cancel the servo deadband. Prints table + linear fit."""
import csv, sys
import numpy as np

rows = list(csv.DictReader(open(sys.argv[1])))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}

def elev(p):
    w, x, y, z = [c[f"{p}_{k}"] for k in "wxyz"]
    return np.degrees(np.arcsin(np.clip(2 * (y * z + x * w), -1, 1)))

rel = elev("rq") - elev("bq")
tick, meas = c["cmd_el_tick"], c["meas_el_tick"]
edges = [0] + [i for i in range(1, len(tick)) if tick[i] != tick[i - 1]] + [len(tick)]
pts = {}
for a, b in zip(edges[:-1], edges[1:]):
    if b - a < 20:
        continue
    s = slice(a + int(0.6 * (b - a)), b)
    pts.setdefault(int(tick[a]), []).append((meas[s].mean(), rel[s].mean()))
print("cmd_tick | meas ticks | elev each visit | hyst | mean")
K, E = [], []
for k in sorted(pts):
    v = pts[k]
    e = [x[1] for x in v]
    print(f"  {k:4d} | {' '.join(f'{x[0]:6.1f}' for x in v):14s} | {' '.join(f'{x:+6.2f}' for x in e):14s} | "
          f"{(max(e) - min(e)) if len(e) > 1 else 0:4.2f} | {np.mean(e):+6.2f}")
    K.append(k); E.append(np.mean(e))
K, E = np.array(K), np.array(E)
p = np.polyfit(K, E, 1)
res = E - np.polyval(p, K)
print(f"\nfit: elev = {p[0]:.5f} * tick {p[1]:+.2f}  ->  {abs(p[0]):.4f} deg/tick, level at tick {-p[1] / p[0]:.1f}, "
      f"residual rms {res.std():.2f} max {abs(res).max():.2f} deg")
