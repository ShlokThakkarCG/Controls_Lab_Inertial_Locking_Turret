"""
probe_limits.py - safely find a servo's mechanical travel limit.

Creeps the servo in small tick steps at low speed and stops as soon as
  * |load| exceeds --max-load (servo pushing against something), or
  * the measured position lags the command by more than --max-lag ticks.
Then backs off and returns to the start tick. Everything is logged like a
normal console session.

    python tools/probe_limits.py --axis el --start 926 --stop 1010 --step 5
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import turret_console as tc  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--port", default=None)
ap.add_argument("--axis", choices=["az", "el"], required=True)
ap.add_argument("--start", type=int, required=True)
ap.add_argument("--stop", type=int, required=True, help="furthest tick to try")
ap.add_argument("--step", type=int, default=5)
ap.add_argument("--max-load", type=float, default=350)
ap.add_argument("--max-lag", type=float, default=12)
ap.add_argument("--settle", type=float, default=0.6)
args = ap.parse_args()

con = tc.Console(args.port or tc.find_port(), 921600, f"limit_{args.axis}_{args.stop}", show_data=False, show_status=False)
import threading  # noqa: E402
threading.Thread(target=con.reader, daemon=True).start()
time.sleep(0.3)
con.send("stream on")
con.send("rate 50")
con.send("spd 300")
cmd = "tel" if args.axis == "el" else "taz"
meas_key = f"meas_{args.axis}_tick"
load_key = f"{args.axis}_load"

con.send(f"{cmd} {args.start}")
time.sleep(1.5)
direction = 1 if args.stop > args.start else -1
last_ok = args.start
reason = "reached --stop"
try:
    for tick in range(args.start, args.stop + direction, direction * args.step):
        con.send(f"{cmd} {tick}")
        time.sleep(args.settle)
        for _ in range(5):                     # tolerate a missed reply / empty first sample
            d = con.latest
            meas, load = d.get(meas_key, -1), d.get(load_key, 0)
            if meas >= 0:
                break
            time.sleep(0.1)
        lag = abs(tick - meas)
        print(f"  cmd {tick:5d}  meas {meas:6.0f}  lag {lag:4.0f}  load {load:+5.0f}  brl_pitch {d.get('brl_pitch', 0):+6.2f}")
        if meas < 0:
            reason = "servo read failed"
            break
        if abs(load) > args.max_load:
            reason = f"load {load:+.0f} > {args.max_load}"
            break
        if lag > args.max_lag:
            reason = f"lag {lag:.0f} ticks > {args.max_lag}"
            break
        last_ok = tick
finally:
    con.send(f"{cmd} {last_ok}")
    time.sleep(0.5)
    con.send(f"{cmd} {args.start}")
    time.sleep(1.5)
    con.send("spd 1200")
    print(f"\nSTOPPED: {reason}. Last clean tick = {last_ok}")
    con.close()
