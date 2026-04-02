# Drone Autopilot Test Task

## Description
Python script for copter control using DroneKit + ArduPilot SITL.

Scenario:
- takeoff
- flight from point A to point B
- descend
- landing at target point

## Stack
- Python
- DroneKit
- ArduPilot SITL
- Mission Planner

## Flight points
- Point A: `50.450739, 30.461242`
- Point B: `50.443326, 30.448078`

## Wind settings
The test was run with the following wind parameters in SITL:
- `SIM_WIND_SPD = 4`
- `SIM_WIND_DIR = 30`
- `SIM_WIND_TURB = 2`

If `SIM_WIND_TURB_FREQ` is not available in the current SITL build, the available wind parameters from the simulator version are used.

## Main file
- `point_a_to_b_test.py`

## Run
1. Start ArduPilot SITL
2. Connect Mission Planner
3. Make sure wind parameters are set
4. Run the script:

```bash
python -u point_a_to_b_test.py