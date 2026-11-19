"""Summarise a turret_stab bench log: state at the end of each lock/aim segment."""
import csv, sys
import numpy as np
path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
raw = open(path.replace(".csv", "_raw.txt")).read().splitlines()
ev = []
for l in raw:
    if "<< E," not in l:
        continue
    body = l.split("<< E,")[1]
    tm, kind = body.split(",")[0], body.split(",")[1]
    if kind in ("aim", "lock", "unlock", "mode"):
        ev.append((int(tm), body))
t = c["t_ms"]
print(" event                      | mode  cmd_az cmd_el | meas_az meas_el | err_az err_el  tot")
for i, (te, name) in enumerate(ev):
    tn = ev[i + 1][0] if i + 1 < len(ev) else t[-1]
    m = (t > tn - 200) & (t < tn - 10)
    if not m.any():
        continue
    g = lambda k: c[k][m].mean()
    print(f" {name[:26]:26s} | {int(g('mode'))}  {g('cmd_az'):+6.1f} {g('cmd_el'):+6.1f} | {g('meas_az'):+6.1f} {g('meas_el'):+6.1f} | "
          f"{g('err_az'):+5.2f} {g('err_el'):+5.2f} {g('err_tot'):5.2f}")
