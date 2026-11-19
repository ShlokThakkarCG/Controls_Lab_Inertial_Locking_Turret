"""
turret_console.py - serial console + logger for the ENR 312 turret firmware.

Every session is recorded to logs/:
    <stamp>_<tag>.csv      the D (data) lines, with the firmware's H header -> MATLAB readtable
    <stamp>_<tag>_raw.txt  every line received/sent, prefixed with host time  -> full record

Interactive use (type firmware commands, Enter to send):
    python tools/turret_console.py
    python tools/turret_console.py --port COM5 --tag tilt_test

Scripted use (runs the commands, records, then exits):
    python tools/turret_console.py --tag az_step --script "z; wait 1; az 20; wait 2; az 0; wait 2"
    python tools/turret_console.py --tag hand_tilt --script "z" --duration 15

Local console commands (not sent to the board):
    :show    toggle the live data summary
    :quit    exit (Ctrl+C works too)
"""

import argparse
import datetime as dt
import os
import sys
import threading
import time

import serial
import serial.tools.list_ports

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(REPO_ROOT, "logs")
ESPRESSIF_VID = 0x303A


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == ESPRESSIF_VID:
            return p.device
    usb = [p for p in ports if p.vid is not None]
    if len(usb) == 1:
        return usb[0].device
    names = ", ".join(f"{p.device} ({p.description})" for p in ports) or "none"
    sys.exit(f"Could not pick a port automatically. Ports found: {names}. Use --port COMx.")


class Console:
    def __init__(self, port, baud, tag, show_data, show_status):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.1
        self.ser.dtr = False        # do not reset the board on connect (UART-bridge port)
        self.ser.rts = False
        self.ser.open()

        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{stamp}_{tag}" if tag else stamp
        self.csv_path = os.path.join(LOG_DIR, base + ".csv")
        self.raw_path = os.path.join(LOG_DIR, base + "_raw.txt")
        self.csv = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.raw = open(self.raw_path, "w", encoding="utf-8")
        self.header = None
        self.header_written = False
        self.n_rows = 0
        self.show_data = show_data
        self.show_status = show_status
        self.last_show = 0.0
        self.running = True
        self.latest = {}            # most recent D line as {column: float}, for scripts
        self.lock = threading.Lock()
        self.t_start = time.time()

    # ---------- I/O ----------
    def send(self, cmd):
        with self.lock:
            self.raw.write(f"{time.time() - self.t_start:10.3f} >> {cmd}\n")
        self.ser.write((cmd + "\n").encode())

    def reader(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(4096)
            except serial.SerialException as e:
                print(f"\n[serial error: {e}]")
                self.running = False
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.handle(line.decode(errors="replace").strip())

    def handle(self, line):
        if not line:
            return
        t = time.time() - self.t_start
        with self.lock:
            self.raw.write(f"{t:10.3f} << {line}\n")
            if line.startswith("H,"):
                new_header = line[2:]
                if not self.header_written:
                    self.header = new_header
                    self.csv.write(new_header + "\n")
                    self.header_written = True
                elif new_header != self.header:
                    print("[warning: firmware header changed mid-log; columns may not match]")
                return
            if line.startswith("D,"):
                if self.header_written:
                    self.csv.write(line[2:] + "\n")
                    self.n_rows += 1
                    vals = line[2:].split(",")
                    cols = self.header.split(",")
                    if len(vals) == len(cols):
                        try:
                            self.latest = dict(zip(cols, map(float, vals)))
                        except ValueError:
                            pass
                if self.show_data and time.time() - self.last_show > 0.5:
                    self.last_show = time.time()
                    self.print_summary(line[2:])
                return
        if line.startswith("S,") and not self.show_status:
            return
        print(line)

    def print_summary(self, data):
        if not self.header:
            return
        cols = self.header.split(",")
        vals = data.split(",")
        if len(vals) != len(cols):
            return
        d = dict(zip(cols, vals))
        try:
            print(f"  base ypr {float(d['base_yaw']):+7.1f} {float(d['base_pitch']):+7.1f} {float(d['base_roll']):+7.1f}"
                  f" | barrel ypr {float(d['brl_yaw']):+7.1f} {float(d['brl_pitch']):+7.1f} {float(d['brl_roll']):+7.1f}"
                  f" | joint imu az/el {float(d['imu_az']):+6.1f} {float(d['imu_el']):+6.1f}"
                  f" servo {float(d['meas_az']):+6.1f} {float(d['meas_el']):+6.1f}")
        except (KeyError, ValueError):
            pass

    def close(self):
        self.running = False
        time.sleep(0.2)
        with self.lock:
            self.csv.close()
            self.raw.close()
        self.ser.close()
        print(f"\nSaved {self.n_rows} data rows:\n  {self.csv_path}\n  {self.raw_path}")


def run_script(con, script):
    for step in [s.strip() for s in script.split(";") if s.strip()]:
        if step.startswith("wait "):
            time.sleep(float(step[5:]))
        else:
            print(f">> {step}")
            con.send(step)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--tag", default="", help="label added to the log file names")
    ap.add_argument("--script", help='commands separated by ";", e.g. "z; wait 2; az 10"')
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to keep recording after the script")
    ap.add_argument("--no-data", action="store_true", help="hide the live data summary")
    ap.add_argument("--status", action="store_true", help="show the 1 Hz S health lines")
    args = ap.parse_args()

    port = args.port or find_port()
    con = Console(port, args.baud, args.tag, show_data=not args.no_data, show_status=args.status)
    print(f"Connected to {port}. Logging to {con.csv_path}")
    threading.Thread(target=con.reader, daemon=True).start()
    time.sleep(0.3)
    con.send("stream on")   # makes the firmware re-print its CSV header
    con.send("info")

    try:
        if args.script is not None:
            run_script(con, args.script)
            time.sleep(args.duration)
        else:
            print("Type commands (help, z, az 10, ...). ':show' toggles data, ':quit' exits.")
            while con.running:
                cmd = input()
                if cmd.strip() == ":quit":
                    break
                if cmd.strip() == ":show":
                    con.show_data = not con.show_data
                    continue
                if cmd.strip():
                    con.send(cmd.strip())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        con.close()


if __name__ == "__main__":
    main()
