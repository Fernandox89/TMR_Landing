# TMR Landing Challenge - TELLO

This project implements an autonomous landing system on a mobile platform for the Mexican Robotics Tournament (TMR) - Autonomous Drones Category.

The system uses a **DJI Tello** drone equipped with a 45-degree mirror to simulate a downward-facing camera, computer vision (OpenCV) to detect **ArUco Markers**, and PID controllers for dynamic alignment.

## Hardware and Modifications

* **Drone:** DJI Tello.
* **Optical Modification:** A mirror mounted at 45° in front of the forward-facing camera to redirect the field of view downwards. *(Note: The software compensates for the image inversion caused by the mirror).*
* **Target:** A printed ArUco marker placed on a mobile/fixed platform.
* **Processing:** Base computer running the script (via WiFi to the Tello).

## Project Architecture

```text
TMR_Landing/
├── main.py                    # Full mission: FSM-driven loop (takeoff, search, track, descend, land)
├── config/
│   ├── settings.json          # ALL tunable parameters (keys starting with "_" are comments)
│   └── loader.py              # Loads settings.json
├── vision/
│   ├── image_processor.py     # Mirror correction (flip) + color conversion
│   ├── aruco_tracker.py       # ArUco detection + PnP pose (cm)
│   ├── corner_tracker.py      # Optical-flow corner tracking when detection fails (glare)
│   └── vision_system.py       # Facade: raw frame -> clean frame + position
├── control/
│   ├── tello_driver.py        # djitellopy wrapper (rc, takeoff, land, emergency, telemetry)
│   ├── pid_controller.py      # PID with anti-windup, derivative-on-measurement, feedforward
│   └── kalman.py              # Kalman per axis: relative position + PLATFORM velocity
├── logic/
│   ├── navigator.py           # Kalman + PID + feedforward -> roll/pitch commands
│   ├── state_machine.py       # Mission states (decides only, sends no commands)
│   └── reversal_predictor.py  # Learns when the back-and-forth platform reverses
├── ui/telemetry_hud.py        # On-screen HUD
├── utils/flight_logger.py     # CSV + video per flight in logs/
├── tools/
│   ├── analyze_log.py         # Post-flight analysis: velocity scale/sign, feedforward gain
│   ├── calibrate_camera.py    # Chessboard calibration through the mirror
│   └── sim_mission.py         # Full-mission simulator using the same control code
├── tests/
│   ├── unit/                  # Automated tests (pytest, no drone needed)
│   └── test_*.py              # Manual hardware scripts
└── docs/
    ├── CAMBIOS.md             # What changed and how everything works (Spanish)
    └── GUIA_LABORATORIO.md    # Step-by-step lab procedure (Spanish)
```

## Quick Start

```bash
python3 -m pytest tests/unit -q          # sanity check, no drone
python3 tools/sim_mission.py --speed 20  # simulated mission
python3 main.py --dry-run                # drone in hand, motors never start
python3 main.py --solo-seguimiento       # fly + track, never descend (calibration)
python3 main.py                          # full mission
python3 tools/analyze_log.py --plot      # analyze the last flight
```

Full mission in the [Tello-Sim-Lab](https://github.com/DroneOps/Tello-Sim-Lab) simulator (cloned next to this repo):

```bash
PYTHONPATH=../Tello-Sim-Lab python3 -m tellosim.run --world tmr_landing --viewer --keys t@5 main.py
```

Keys: `t` takeoff · `SPACE` emergency motor stop · `l` land · `b` abort descent · `q` land and quit.

See [docs/GUIA_LABORATORIO.md](docs/GUIA_LABORATORIO.md) for the lab procedure and
[docs/CAMBIOS.md](docs/CAMBIOS.md) for a full explanation of the design.

## Why this structure?

1. **Hardware Isolation (Mirror):** All geometric mirror correction happens exclusively in `image_processor.py`. Note: `flip_code` and the `axis_sign_*` parameters go together (a wrong flip still detects the marker, rotated 180°, with inverted X/Y signs).
2. **State Machine:** Landing on a moving target requires clear phases. `state_machine.py` decides the phase; `main.py` maps each phase to commands.
3. **Moving target:** The Kalman filter estimates the platform velocity, used as feedforward, and the reversal predictor avoids starting the blind final descent right before the platform changes direction.
4. **Safe Testing Environment:** Unit tests and the simulator catch logic/sign errors without flying; `--dry-run` and `--solo-seguimiento` allow staged validation.

## Next Steps (To-Do)

* [✓] Image correction module and vision tests.
* [✓] PID controller.
* [✓] State machine logic.
* [✓] Telemetry and logging module.
* [✓] Integrate modules into the main program.
* [✓] Kalman with platform velocity + feedforward + reversal predictor.
* [ ] Validate velocity scale/sign and feedforward in flight (`tools/analyze_log.py`).
* [ ] Full landing on static platform, then on moving platform.
* [ ] Optional: nested ArUco markers to shrink the blind spot.
* [ ] Check TMR 2027 rules.
