README

Project: UAV Autopilot (Point A → Point B with precise landing)

Description:
This project implements a custom autopilot script using DroneKit and ArduPilot SITL.
The drone performs:
- Takeoff
- Flight from Point A to Point B in STABILIZE mode
- Constant yaw during entire flight
- Wind compensation
- Precise landing at target point (accuracy < 0.5 m)

Key Features:
- RC Override control (no GUIDED navigation)
- Continuous GPS-based correction
- Wind drift compensation using velocity feedback
- Adaptive control for different flight phases:
  - Cruise
  - Brake
  - Near
  - Lock
  - Land
- Pre-landing stabilization (“lock” phase)
- Re-lock if drift detected during descent

Coordinates:
Point A (Start):
50.450739, 30.461242

Point B (Target):
50.443326, 30.448078

Target altitude:
200 meters

Environment Setup:

1. Install Mission Planner:
https://ardupilot.org/planner/

2. Run SITL:
cd ~/ardupilot
build/sitl/bin/arducopter --model + --speedup 1 --slave 0 \
--defaults Tools/autotest/default_params/copter.parm \
--sim-address=127.0.0.1 -I0 \
--home 50.450739,30.461242,0,0

3. Run MAVProxy:
mavproxy.py --master=tcp:127.0.0.1:5760 \
--out=tcpin:0.0.0.0:5770 \
--out=127.0.0.1:14651

4. Connect Mission Planner:
Type: TCP
Host: 127.0.0.1
Port: 5770

5. Set wind parameters (in MAVProxy):
param set SIM_WIND_SPD 4
param set SIM_WIND_DIR 30
param set SIM_WIND_TURB 2

6. Python environment:
Python 3.9 required (DroneKit compatibility)

Install:
pip install dronekit pymavlink

7. Run script:
python point_a_to_b_v11.py

Results:
- Stable flight under wind conditions
- Continuous trajectory correction
- Final landing accuracy: < 0.5 m from target

Notes:
- Wind is simulated via SITL parameters
- Script compensates drift using velocity feedback
- Landing is performed only after stable lock on target

Deliverables:
- Source code (GitHub)
- Flight video (Google Drive)

