from time import sleep, time
from math import cos, radians, sqrt
from dronekit import connect, VehicleMode

CONNECTION_STRING = "127.0.0.1:14651"

POINT_A_LAT = 50.450739
POINT_A_LON = 30.461242
POINT_B_LAT = 50.443326
POINT_B_LON = 30.448078

TARGET_ALT = 200.0
MAX_TIME = 1800

YAW_HOLD = 1500
ROLL_NEUTRAL = 1500
PITCH_NEUTRAL = 1500

THROTTLE_MIN = 1320
THROTTLE_MAX = 1620
BASE_THROTTLE = 1490
ALT_KP = 52.0
ALT_KD = 42.0

CRUISE_SPEED = 12.0
BRAKE_SPEED = 7.0
NEAR_SPEED = 3.5
LOCK_SPEED = 0.8

BRAKE_RADIUS = 120.0
NEAR_RADIUS = 30.0
LOCK_RADIUS = 6.0
LAND_START_RADIUS = 0.45

LINE_PULL_CRUISE = 0.12
LINE_PULL_BRAKE = 0.18
LINE_PULL_NEAR = 0.24
LINE_PULL_LOCK = 0.22

TO_TARGET_GAIN_CRUISE = 0.50
TO_TARGET_GAIN_BRAKE = 0.95
TO_TARGET_GAIN_NEAR = 1.45
TO_TARGET_GAIN_LOCK = 2.20

BRAKE_DECEL = 2.0
NEAR_DECEL = 2.7
LOCK_DECEL = 3.2

FORWARD_KP = 82.0
RIGHT_KP = 96.0
VEL_DAMP_F = 22.0
VEL_DAMP_R = 26.0

WIND_BIAS_KI_F = 1.5
WIND_BIAS_KI_R = 2.0
WIND_BIAS_LIMIT_F = 150.0
WIND_BIAS_LIMIT_R = 210.0

LOCK_BIAS_LIMIT_F = 70.0
LOCK_BIAS_LIMIT_R = 95.0
LAND_BIAS_LIMIT_F = 40.0
LAND_BIAS_LIMIT_R = 55.0
LOCK_BIAS_LEAK = 0.93
LAND_BIAS_LEAK = 0.86

ROLL_LIMIT_CRUISE = 460
PITCH_LIMIT_CRUISE = 460
ROLL_LIMIT_BRAKE = 470
PITCH_LIMIT_BRAKE = 470
ROLL_LIMIT_NEAR = 430
PITCH_LIMIT_NEAR = 430
ROLL_LIMIT_LOCK = 280
PITCH_LIMIT_LOCK = 280
ROLL_LIMIT_LAND = 150
PITCH_LIMIT_LAND = 150

LOCK_REQUIRED_TIME = 1.6
LOCK_REQUIRED_SPEED = 0.25
LOW_ALT_SPEED_LIMIT = 0.15
LAND_HOLD_ALT = 10.0


def clamp(v, mn, mx):
    return max(mn, min(mx, v))


def latlon(vehicle):
    loc = vehicle.location.global_relative_frame
    return loc.lat, loc.lon


def alt(vehicle):
    return vehicle.location.global_relative_frame.alt


def distance_ne(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * 111320.0
    de = (lon2 - lon1) * 111320.0 * cos(radians((lat1 + lat2) / 2.0))
    return dn, de


def earth_to_body(north, east, yaw_deg):
    y = radians(yaw_deg)
    s = __import__("math").sin(y)
    c = cos(y)
    forward = north * c + east * s
    right = -north * s + east * c
    return forward, right


def velocity_ne(vehicle):
    vx, vy, vz = vehicle.velocity if vehicle.velocity is not None else (0.0, 0.0, 0.0)
    north_v = float(vx) if vx is not None else 0.0
    east_v = float(vy) if vy is not None else 0.0
    down_v = float(vz) if vz is not None else 0.0
    return north_v, east_v, down_v


def set_override(vehicle, roll, pitch, thr):
    vehicle.channels.overrides = {
        "1": int(roll),
        "2": int(pitch),
        "3": int(thr),
        "4": YAW_HOLD,
    }


def clear_override(vehicle):
    vehicle.channels.overrides = {}


def wait_ready_position(vehicle, timeout=90):
    print("Waiting for EKF and GPS position...")
    t0 = time()
    while time() - t0 < timeout:
        loc = vehicle.location.global_relative_frame
        ok = (
            vehicle.is_armable
            and loc.lat is not None
            and loc.lon is not None
            and abs(loc.lat) > 0.0001
            and abs(loc.lon) > 0.0001
        )
        print(
            f"ready_check armable={vehicle.is_armable} "
            f"lat={loc.lat} lon={loc.lon} alt={loc.alt}"
        )
        if ok:
            return
        sleep(1)
    raise TimeoutError("Vehicle not ready")


def stopping_distance(speed, decel):
    return (speed * speed) / max(2.0 * decel, 0.1)


def guidance_mode(dist_to_b):
    if dist_to_b <= LOCK_RADIUS:
        return "lock"
    if dist_to_b <= NEAR_RADIUS:
        return "near"
    if dist_to_b <= BRAKE_RADIUS:
        return "brake"
    return "cruise"


def line_guidance(cur_n, cur_e, to_b_n, to_b_e, track_n, track_e, dist_to_b, mode, horiz_speed):
    cross = cur_n * (-track_e) + cur_e * track_n

    if mode == "cruise":
        speed = CRUISE_SPEED
        line_pull = LINE_PULL_CRUISE
        to_b_gain = TO_TARGET_GAIN_CRUISE
        decel = BRAKE_DECEL
    elif mode == "brake":
        speed = BRAKE_SPEED
        line_pull = LINE_PULL_BRAKE
        to_b_gain = TO_TARGET_GAIN_BRAKE
        decel = BRAKE_DECEL
    elif mode == "near":
        speed = NEAR_SPEED
        line_pull = LINE_PULL_NEAR
        to_b_gain = TO_TARGET_GAIN_NEAR
        decel = NEAR_DECEL
    else:
        speed = LOCK_SPEED
        line_pull = LINE_PULL_LOCK
        to_b_gain = TO_TARGET_GAIN_LOCK
        decel = LOCK_DECEL

    dist_safe = max(dist_to_b, 0.001)
    u_bn = to_b_n / dist_safe
    u_be = to_b_e / dist_safe

    d_stop = stopping_distance(horiz_speed, decel)
    if dist_to_b < d_stop + 8.0:
        speed *= clamp((dist_to_b - 1.0) / max(d_stop + 8.0, 1.0), 0.20, 1.0)

    corr_n = (-cross * line_pull) * (-track_e)
    corr_e = (-cross * line_pull) * (track_n)

    vn_cmd = speed * u_bn + corr_n + to_b_gain * to_b_n
    ve_cmd = speed * u_be + corr_e + to_b_gain * to_b_e

    # On final stages, stop following the route and hold the point itself
    if mode == "lock":
        vn_cmd = 2.40 * to_b_n
        ve_cmd = 2.40 * to_b_e
    elif mode == "near" and dist_to_b < 10.0:
        vn_cmd = 1.70 * to_b_n
        ve_cmd = 1.70 * to_b_e

    if mode == "near":
        vn_cmd = clamp(vn_cmd, -4.5, 4.5)
        ve_cmd = clamp(ve_cmd, -4.5, 4.5)
    elif mode == "lock":
        vn_cmd = clamp(vn_cmd, -0.9, 0.9)
        ve_cmd = clamp(ve_cmd, -0.9, 0.9)
    else:
        vn_cmd = clamp(vn_cmd, -11.5, 11.5)
        ve_cmd = clamp(ve_cmd, -11.5, 11.5)

    return vn_cmd, ve_cmd, cross, d_stop


print("connect...")
vehicle = connect(CONNECTION_STRING, wait_ready=True, heartbeat_timeout=60)

vehicle.mode = VehicleMode("STABILIZE")
sleep(2)

wait_ready_position(vehicle)

a_to_b_n, a_to_b_e = distance_ne(POINT_A_LAT, POINT_A_LON, POINT_B_LAT, POINT_B_LON)
track_len = sqrt(a_to_b_n * a_to_b_n + a_to_b_e * a_to_b_e)
track_n = a_to_b_n / track_len
track_e = a_to_b_e / track_len

print(f"track_len={track_len:.2f}m track_n={track_n:.4f} track_e={track_e:.4f}")

print("arm...")
vehicle.armed = True
while not vehicle.armed:
    sleep(1)

phase = "takeoff"
lock_started_at = None

bias_f = 0.0
bias_r = 0.0

prev_alt = alt(vehicle)
prev_t = time()
started = time()

while time() - started < MAX_TIME:
    now = time()
    dt = max(now - prev_t, 0.05)

    current_alt = alt(vehicle)
    climb_rate = (current_alt - prev_alt) / dt
    alt_err = TARGET_ALT - current_alt

    thr = int(BASE_THROTTLE + ALT_KP * alt_err - ALT_KD * climb_rate)
    thr = clamp(thr, THROTTLE_MIN, THROTTLE_MAX)

    lat, lon = latlon(vehicle)
    cur_n_from_a, cur_e_from_a = distance_ne(POINT_A_LAT, POINT_A_LON, lat, lon)
    to_b_n, to_b_e = distance_ne(lat, lon, POINT_B_LAT, POINT_B_LON)
    dist_to_b = sqrt(to_b_n * to_b_n + to_b_e * to_b_e)

    north_v, east_v, down_v = velocity_ne(vehicle)
    yaw = vehicle.heading if vehicle.heading is not None else 0
    horiz_speed = sqrt(north_v * north_v + east_v * east_v)

    roll = ROLL_NEUTRAL
    pitch = PITCH_NEUTRAL
    nav = phase
    d_stop = 0.0
    cross = 0.0

    if phase == "takeoff":
        if current_alt >= TARGET_ALT - 2.0:
            print("→ NAV")
            phase = "nav"
    else:
        base_mode = guidance_mode(dist_to_b)

        if phase in ("nav", "lock"):
            if base_mode == "lock":
                phase = "lock"
            else:
                phase = "nav"

        vn_cmd, ve_cmd, cross, d_stop = line_guidance(
            cur_n_from_a, cur_e_from_a, to_b_n, to_b_e, track_n, track_e, dist_to_b, base_mode, horiz_speed
        )

        evn = vn_cmd - north_v
        eve = ve_cmd - east_v

        a_f_cmd, a_r_cmd = earth_to_body(evn, eve, yaw)
        f_vel_body, r_vel_body = earth_to_body(north_v, east_v, yaw)

        if phase == "lock":
            bias_f = clamp(bias_f * LOCK_BIAS_LEAK + WIND_BIAS_KI_F * a_f_cmd * dt, -LOCK_BIAS_LIMIT_F, LOCK_BIAS_LIMIT_F)
            bias_r = clamp(bias_r * LOCK_BIAS_LEAK + WIND_BIAS_KI_R * a_r_cmd * dt, -LOCK_BIAS_LIMIT_R, LOCK_BIAS_LIMIT_R)
        elif phase == "land":
            bias_f = clamp(bias_f * LAND_BIAS_LEAK + 0.8 * WIND_BIAS_KI_F * a_f_cmd * dt, -LAND_BIAS_LIMIT_F, LAND_BIAS_LIMIT_F)
            bias_r = clamp(bias_r * LAND_BIAS_LEAK + 0.8 * WIND_BIAS_KI_R * a_r_cmd * dt, -LAND_BIAS_LIMIT_R, LAND_BIAS_LIMIT_R)
        else:
            bias_f += WIND_BIAS_KI_F * a_f_cmd * dt
            bias_r += WIND_BIAS_KI_R * a_r_cmd * dt
            bias_f = clamp(bias_f, -WIND_BIAS_LIMIT_F, WIND_BIAS_LIMIT_F)
            bias_r = clamp(bias_r, -WIND_BIAS_LIMIT_R, WIND_BIAS_LIMIT_R)

        if phase == "nav":
            if base_mode == "cruise":
                roll_limit = ROLL_LIMIT_CRUISE
                pitch_limit = PITCH_LIMIT_CRUISE
            elif base_mode == "brake":
                roll_limit = ROLL_LIMIT_BRAKE
                pitch_limit = PITCH_LIMIT_BRAKE
            else:
                roll_limit = ROLL_LIMIT_NEAR
                pitch_limit = PITCH_LIMIT_NEAR
        elif phase == "lock":
            roll_limit = ROLL_LIMIT_LOCK
            pitch_limit = PITCH_LIMIT_LOCK
        else:
            roll_limit = ROLL_LIMIT_LAND
            pitch_limit = PITCH_LIMIT_LAND

        pitch = int(PITCH_NEUTRAL - (FORWARD_KP * a_f_cmd + VEL_DAMP_F * (-f_vel_body) + bias_f))
        roll = int(ROLL_NEUTRAL + (RIGHT_KP * a_r_cmd + VEL_DAMP_R * (-r_vel_body) + bias_r))

        pitch = int(clamp(pitch, PITCH_NEUTRAL - pitch_limit, PITCH_NEUTRAL + pitch_limit))
        roll = int(clamp(roll, ROLL_NEUTRAL - roll_limit, ROLL_NEUTRAL + roll_limit))

        if phase == "nav" and base_mode == "lock":
            print("→ LOCK")
            phase = "lock"
            lock_started_at = None

        if phase == "lock":
            hold_alt_error = LAND_HOLD_ALT - current_alt
            thr = int(BASE_THROTTLE + 35.0 * hold_alt_error - 28.0 * climb_rate)
            thr = clamp(thr, 1410, 1505)

            speed_gate = LOW_ALT_SPEED_LIMIT if current_alt < 3.0 else LOCK_REQUIRED_SPEED
            if dist_to_b < LAND_START_RADIUS and horiz_speed < speed_gate:
                if lock_started_at is None:
                    lock_started_at = now
                elif now - lock_started_at >= LOCK_REQUIRED_TIME:
                    print("→ LAND")
                    phase = "land"
            else:
                lock_started_at = None

        if phase == "land":
            speed_gate = LOW_ALT_SPEED_LIMIT if current_alt < 3.0 else LOCK_REQUIRED_SPEED
            if dist_to_b > 0.50 or horiz_speed > max(speed_gate + 0.18, 0.32):
                print("→ RELOCK")
                phase = "lock"
                lock_started_at = None
                thr = int(BASE_THROTTLE + 35.0 * (LAND_HOLD_ALT - current_alt) - 28.0 * climb_rate)
                thr = clamp(thr, 1410, 1505)
            else:
                if current_alt > 10.0:
                    thr = 1428
                elif current_alt > 5.0:
                    thr = 1420
                elif current_alt > 2.0:
                    thr = 1412
                elif current_alt > 0.8:
                    thr = 1406
                else:
                    thr = 1100

                if current_alt < 0.15:
                    print("LAND DONE")
                    break

    set_override(vehicle, roll, pitch, thr)

    print(
        f"phase={phase} nav={nav} dist={dist_to_b:.2f} alt={current_alt:.2f} "
        f"to_b_n={to_b_n:.2f} to_b_e={to_b_e:.2f} "
        f"from_a_n={cur_n_from_a:.2f} from_a_e={cur_e_from_a:.2f} "
        f"vn={north_v:.2f} ve={east_v:.2f} hs={horiz_speed:.2f} d_stop={d_stop:.2f} "
        f"cross={cross:.2f} roll={roll} pitch={pitch} thr={thr} "
        f"bias_f={bias_f:.1f} bias_r={bias_r:.1f} heading={yaw}"
    )

    prev_alt = current_alt
    prev_t = now
    sleep(0.2)

clear_override(vehicle)
vehicle.armed = False
vehicle.close()
print("done")
