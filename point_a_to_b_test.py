from time import sleep, time
from math import cos, radians
from dronekit import connect, VehicleMode

CONNECTION_STRING = "127.0.0.1:14651"

POINT_A_LAT = 50.450739
POINT_A_LON = 30.461242

POINT_B_LAT = 50.443326
POINT_B_LON = 30.448078

TARGET_ALT = 200.0
MAX_TIME = 1800

# More aggressive climb / altitude control
BASE_THROTTLE = 1490
ALT_KP = 52.0
ALT_KD = 46.0

THROTTLE_MIN = 1320
THROTTLE_MAX = 1620

YAW_HOLD = 1500

ROLL_NEUTRAL = 1500
PITCH_NEUTRAL = 1500

TARGET_RADIUS_M = 0.8
TARGET_HOLD_SEC = 0.0

# More aggressive far cruise
FAR_PITCH_POS = 1150
FAR_PITCH_NEG = 1850
FAR_ROLL_POS = 1850
FAR_ROLL_NEG = 1150

FAR_TO_MID_DIST_M = 30.0
MID_TO_NEAR_DIST_M = 8.0
NEAR_TO_MID_BACK_DIST_M = 10.0

LAND_RADIUS_M = 0.8
LAND_MAX_GS = 0.18
LAND_MAX_ERR_RATE = 0.18
LAND_STABLE_HOLD_SEC = 0.45

# Fast controlled descent to this height, then current final landing profile
PRELAND_ALT = 10.0


def clamp(v, mn, mx):
    return max(mn, min(mx, v))


def apply_min_correction(cmd, neutral, min_delta):
    delta = cmd - neutral
    if delta == 0:
        return cmd
    if 0 < abs(delta) < min_delta:
        return neutral + min_delta if delta > 0 else neutral - min_delta
    return cmd


def alt(vehicle):
    return round(vehicle.location.global_relative_frame.alt, 3)


def lat_lon(vehicle):
    loc = vehicle.location.global_relative_frame
    return loc.lat, loc.lon


def wait_for_ekf_and_position(vehicle, timeout=90):
    print("Waiting for EKF and valid position...")
    started = time()

    while time() - started < timeout:
        loc = vehicle.location.global_relative_frame
        lat_ok = loc.lat is not None and abs(loc.lat) > 0.0001
        lon_ok = loc.lon is not None and abs(loc.lon) > 0.0001

        ekf_ok = True
        try:
            ekf_ok = bool(vehicle.ekf_ok)
        except Exception:
            pass

        is_armable = bool(vehicle.is_armable)

        print(
            f"EKF wait: armable={is_armable} ekf_ok={ekf_ok} "
            f"lat={loc.lat} lon={loc.lon} alt={loc.alt}"
        )

        if lat_ok and lon_ok and ekf_ok and is_armable:
            print("EKF and position are ready")
            return True

        sleep(1)

    raise TimeoutError("EKF/position not ready in time")


def distance_xy_m(lat1, lon1, lat2, lon2):
    dlat = (lat2 - lat1) * 111320.0
    dlon = (lon2 - lon1) * 111320.0 * cos(radians((lat1 + lat2) / 2.0))
    return dlat, dlon


def set_override(vehicle, **kwargs):
    vehicle.channels.overrides = {str(k): v for k, v in kwargs.items()}


def clear(vehicle):
    vehicle.channels.overrides = {}


def compute_near_controls(
    north_error,
    east_error,
    north_rate,
    east_rate,
    dt,
    n_int,
    e_int,
    min_error_for_push=0.7,
    min_delta=24,
):
    n_int += north_error * dt
    e_int += east_error * dt

    n_int = clamp(n_int, -6.0, 6.0)
    e_int = clamp(e_int, -6.0, 6.0)

    kp = 9.0
    kd = 1.5
    ki = 3.2

    pitch_cmd = (
        PITCH_NEUTRAL
        - kp * north_error
        - kd * north_rate
        - ki * n_int
    )
    roll_cmd = (
        ROLL_NEUTRAL
        + kp * east_error
        - kd * east_rate
        + ki * e_int
    )

    pitch = int(clamp(pitch_cmd, 1435, 1565))
    roll = int(clamp(roll_cmd, 1435, 1565))

    if abs(north_error) > min_error_for_push:
        pitch = apply_min_correction(pitch, PITCH_NEUTRAL, min_delta)
    if abs(east_error) > min_error_for_push:
        roll = apply_min_correction(roll, ROLL_NEUTRAL, min_delta)

    pitch = int(clamp(pitch, 1435, 1565))
    roll = int(clamp(roll, 1435, 1565))

    return roll, pitch, n_int, e_int


print("Connecting...")
vehicle = connect(CONNECTION_STRING, wait_ready=True, heartbeat_timeout=60)
print("Connected")

vehicle.mode = VehicleMode("STABILIZE")
sleep(2)

wait_for_ekf_and_position(vehicle)

initial_heading = vehicle.heading
print(f"Initial heading: {initial_heading}")

start_lat, start_lon = lat_lon(vehicle)
print(f"Actual start lat={start_lat}, lon={start_lon}")
print(f"Reference A lat={POINT_A_LAT}, lon={POINT_A_LON}")
print(f"Target B lat={POINT_B_LAT}, lon={POINT_B_LON}")

print("Arming...")
vehicle.armed = True
while not vehicle.armed:
    sleep(1)

phase = "takeoff"
target_hold_started_at = None
land_stable_started_at = None
target_zone_logged = False

prev_alt = alt(vehicle)
prev_t = time()

prev_n_err = 0.0
prev_e_err = 0.0

nav_mode = "far"
n_int = 0.0
e_int = 0.0

start_time = time()

while time() - start_time < MAX_TIME:
    current_alt = alt(vehicle)
    now = time()
    dt = max(now - prev_t, 0.05)

    climb_rate = (current_alt - prev_alt) / dt
    alt_error = TARGET_ALT - current_alt

    throttle = int(BASE_THROTTLE + ALT_KP * alt_error - ALT_KD * climb_rate)
    throttle = clamp(throttle, THROTTLE_MIN, THROTTLE_MAX)

    cur_lat, cur_lon = lat_lon(vehicle)
    north_error, east_error = distance_xy_m(cur_lat, cur_lon, POINT_B_LAT, POINT_B_LON)

    north_rate = (north_error - prev_n_err) / dt
    east_rate = (east_error - prev_e_err) / dt

    distance_to_target = (north_error ** 2 + east_error ** 2) ** 0.5
    groundspeed = vehicle.groundspeed

    roll = ROLL_NEUTRAL
    pitch = PITCH_NEUTRAL

    if phase == "takeoff":
        if current_alt >= TARGET_ALT - 1.0:
            print("→ MOVE TO B")
            phase = "move"

    elif phase == "move":
        if distance_to_target > FAR_TO_MID_DIST_M:
            nav_mode = "far"
        elif nav_mode == "far" and distance_to_target <= FAR_TO_MID_DIST_M:
            nav_mode = "mid"
        elif nav_mode == "mid" and distance_to_target <= MID_TO_NEAR_DIST_M:
            nav_mode = "near"
        elif nav_mode == "near" and distance_to_target >= NEAR_TO_MID_BACK_DIST_M:
            nav_mode = "mid"

        if nav_mode == "far":
            kp = 5.5
            kd = 3.5

            pitch = int(PITCH_NEUTRAL - kp * north_error - kd * north_rate)
            roll = int(ROLL_NEUTRAL + kp * east_error - kd * east_rate)

            pitch = clamp(pitch, 1200, 1800)
            roll = clamp(roll, 1200, 1800)

            n_int = 0.0
            e_int = 0.0

        elif nav_mode == "mid":
            kp = 7.0
            kd = 6.0

            pitch = int(PITCH_NEUTRAL - kp * north_error - kd * north_rate)
            roll = int(ROLL_NEUTRAL + kp * east_error - kd * east_rate)

            pitch = clamp(pitch, 1350, 1650)
            roll = clamp(roll, 1350, 1650)

            n_int = 0.0
            e_int = 0.0

        elif nav_mode == "near":
            roll, pitch, n_int, e_int = compute_near_controls(
                north_error=north_error,
                east_error=east_error,
                north_rate=north_rate,
                east_rate=east_rate,
                dt=dt,
                n_int=n_int,
                e_int=e_int,
                min_error_for_push=0.7,
                min_delta=24,
            )

        if distance_to_target <= TARGET_RADIUS_M:
            if not target_zone_logged:
                print("→ TARGET ZONE REACHED")
                target_zone_logged = True
            if target_hold_started_at is None:
                target_hold_started_at = time()
        else:
            target_hold_started_at = None
            land_stable_started_at = None
            target_zone_logged = False

        if distance_to_target <= 5.0:
            print("→ DESCEND TO PRELAND ALT")
            phase = "descend"
            n_int = 0.0
            e_int = 0.0

    elif phase == "descend":
        # Keep precise horizontal correction, but descend aggressively until 10 m
        roll, pitch, n_int, e_int = compute_near_controls(
            north_error=north_error,
            east_error=east_error,
            north_rate=north_rate,
            east_rate=east_rate,
            dt=dt,
            n_int=n_int,
            e_int=e_int,
            min_error_for_push=0.7,
            min_delta=24,
        )

        if current_alt > PRELAND_ALT + 60.0:
            throttle = 1425
        elif current_alt > PRELAND_ALT + 25.0:
            throttle = 1432
        elif current_alt > PRELAND_ALT + 10.0:
            throttle = 1438
        elif current_alt > PRELAND_ALT + 1.0:
            throttle = 1446
        else:
            print("→ LAND")
            phase = "land"

    elif phase == "land":
        # Final landing: keep correcting the whole time.
        # More aggressive early reaction below 10 m.
        roll, pitch, n_int, e_int = compute_near_controls(
            north_error=north_error,
            east_error=east_error,
            north_rate=north_rate,
            east_rate=east_rate,
            dt=dt,
            n_int=n_int,
            e_int=e_int,
            min_error_for_push=0.1,
            min_delta=24,
        )

        # Keep the currently successful lower-than-10m landing profile
        if current_alt > 8.0:
            throttle = 1442
        elif current_alt > 3.0:
            throttle = 1435
        elif current_alt > 1.0:
            throttle = 1420
        else:
            throttle = 1100

    set_override(vehicle, **{
        "1": roll,
        "2": pitch,
        "3": throttle,
        "4": YAW_HOLD,
    })

    vx, vy, vz = vehicle.velocity if vehicle.velocity is not None else (None, None, None)

    print(
        f"phase={phase} nav={nav_mode} alt={current_alt:.2f} "
        f"dist={distance_to_target:.2f}m "
        f"n_err={north_error:.2f} e_err={east_error:.2f} "
        f"n_rate={north_rate:.2f} e_rate={east_rate:.2f} "
        f"roll={roll} pitch={pitch} thr={throttle} "
        f"gs={groundspeed} vel=({vx}, {vy}, {vz}) "
        f"heading={vehicle.heading}"
    )

    prev_alt = current_alt
    prev_t = now
    prev_n_err = north_error
    prev_e_err = east_error

    sleep(0.25)

    if phase == "land" and current_alt < 0.1:
        print("Landing altitude reached")
        break

print(f"Final heading: {vehicle.heading}")
print("Disarming...")
vehicle.armed = False
sleep(2)
clear(vehicle)
vehicle.close()
print("Done")
