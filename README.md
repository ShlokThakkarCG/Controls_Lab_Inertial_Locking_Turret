# ENR 312: 2-axis inertially stabilized turret

watch here -> [https://youtu.be/zV6PJhuFrNA](url)
more vids -> [https://drive.google.com/drive/folders/1tQs0ntFzKtMpE5NhNUlBaBvzWU9CKt9a?usp=sharing](url)

ESP32-C6-DevKitC-1 · 2× Feetech SC15 bus servos (Waveshare adapter, 1 Mbps) · 2× BNO08x IMUs (Game Rotation Vector)

Goal: hold the barrel's **pointing direction** fixed in space while the base plate is moved by hand in roll, pitch and yaw.
With two joints (azimuth, elevation) the direction can be held, but barrel roll about its own axis cannot be corrected.

## Layout

| Path | What |
|---|---|
| `firmware/turret_probe/` | Phase 0 diagnostics: streams both IMUs, servo feedback and IMU-measured joint angles. No control. |
| `firmware/legacy_gemini_v0/` | Earlier controller (reference only). |
| `tools/turret_console.py` | Serial console + logger. Every run goes to `logs/` as CSV (for MATLAB) plus a raw text log. |
| `tools/flash.ps1` | Compile / upload with the Arduino IDE's bundled CLI. |
| `matlab/` | Simulation and analysis. |
| `logs/` | Recorded runs. |

## Quick start

```powershell
.\tools\flash.ps1 -Port COM5
python tools\turret_console.py --tag first_look
```

Board setting: **USB CDC On Boot = Enabled**; plug into the board's **USB** connector. Serial is 921600 baud (ignored over native USB).

## Conventions

All frames are +X right, +Y forward (barrel), +Z up. Angles are in degrees.
yaw + = nose left (same sense as +azimuth), pitch + = nose up (same as +elevation), roll + = right side down.
