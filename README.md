# TMR Landing Challenge - TELLO

This project implements an autonomous landing system on a mobile platform for the Mexican Robotics Tournament (TMR) - Autonomous Drones Category.

The system uses a **DJI Tello** drone equipped with a 45-degree mirror to simulate a downward-facing camera, computer vision (OpenCV) to detect **ArUco Markers**, and PID controllers for dynamic alignment.

## Hardware and Modifications

* **Drone:** DJI Tello.
* **Optical Modification:** A mirror mounted at 45° in front of the forward-facing camera to redirect the field of view downwards. *(Note: The software compensates for the image inversion caused by the mirror).*
* **Target:** A printed ArUco marker placed on a mobile/fixed platform.
* **Processing:** Base computer running the script (via WiFi to the Tello).

## Project Architecture

The project is designed in a modular way to separate computer vision, flight control, and decision-making:

```text
tello_aruco_landing/
│
├── main.py                   # Entry point. Initializes modules and runs the main loop.
│
├── config/
│   └── settings.json         # Constants: max speeds, ArUco ID, marker size, PID gains.
│
├── vision/
│   ├── image_processor.py    # Frame capture and correction (straighten and invert the mirror image).
│   └── aruco_tracker.py      # ArUco detection and pose estimation (X, Y, Z, rotation).
│
├── control/
│   ├── tello_driver.py       # djitellopy wrapper (connection, takeoff, velocity commands).
│   └── pid_controller.py     # PID controllers to smooth and calculate the alignment velocity.
│
├── logic/
│   └── state_machine.py      # State machine: SEARCH, ALIGN, TRACK, LAND.
│
├── tests/
│   ├── test_vision_only.py   # Camera and mirror correction tests (without turning on motors).
│   ├── test_pid.py           # Simulation to tune PID gains.
│   └── test_telemetry.py     # Tello battery and sensor reading.
│
└── logs/                     # Debugging files and video recording of the tests.

```

## Why this structure?

1. **Hardware Isolation (Mirror):** All geometric mirror correction happens exclusively in `image_processor.py`. The rest of the system receives clean images and coordinates.
2. **State Machine:** Landing on a moving target requires clear phases. `state_machine.py` prevents the drone from attempting to descend before matching the platform's speed.
3. **Safe Testing Environment:** The `tests/` folder allows testing visual detection with the drone in hand, minimizing the risk of accidents before actual flights.

## Next Steps (To-Do)

* [✓] Write the image correction module (`image_processor.py`) and test it with `test_vision_only.py`.
* [✓] Write the PID controller.
* [ ] Write the state machine logic.
* [ ] Write the telemetry and logging module.
* [ ] Implement modules into the main program.
