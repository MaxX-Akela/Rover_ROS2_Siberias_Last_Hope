"""Frame conversions and /cmd_vel mapping (pure functions, no ROS imports).

PX4 uses NED world / FRD body, ROS uses ENU world / FLU body (REP 103).
Quaternions are (w, x, y, z).
"""
import math

_S = math.sqrt(0.5)
Q_ENU_NED = (0.0, _S, _S, 0.0)  # rotation NED -> ENU
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)  # rotation FRD -> FLU (180 deg about x)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    r = quat_mul(quat_mul(q, (0.0, *v)), quat_conj(q))
    return (r[1], r[2], r[3])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def attitude_enu_flu(q_ned_frd):
    """Body FRD -> world NED quaternion  =>  body FLU -> world ENU quaternion."""
    return quat_mul(quat_mul(Q_ENU_NED, q_ned_frd), Q_FRD_FLU)


def ned_to_enu(v):
    return (v[1], v[0], -v[2])


def frd_to_flu(v):
    return (v[0], -v[1], -v[2])


def direct_setpoints(vx, wz, max_linear, max_angular, invert_steering=False):
    """/cmd_vel -> (throttle, steering) for RoverThrottleSetpoint / RoverSteeringSetpoint.

    throttle: -1 (backwards) .. 1 (forwards). steering: -1 (left) .. 1 (right), so a
    counter-clockwise (left) wz > 0 gives negative steering.
    """
    throttle = clamp(vx / max_linear, -1.0, 1.0) if max_linear > 0.0 else 0.0
    steering = clamp(-wz / max_angular, -1.0, 1.0) if max_angular > 0.0 else 0.0
    return throttle, -steering if invert_steering else steering


def velocity_setpoint(vx, wz, heading_ned, horizon, min_speed=0.02):
    """/cmd_vel -> NED velocity vector for the PX4 differential rover offboard mode.

    That mode turns a velocity vector into speed = |v| (never negative) and a heading
    setpoint = atan2(v_e, v_n). Hence: no reverse, and the yaw rate is realised as a heading
    `wz * horizon` ahead of the current one. `min_speed` keeps the vector non-zero so the
    heading stays defined (a zero vector would command heading 0 = north).
    """
    speed = max(vx, 0.0, min_speed)
    offset = clamp(-wz * horizon, -math.pi / 2, math.pi / 2)  # ENU ccw+ -> NED yaw cw+
    heading = heading_ned + offset
    return speed * math.cos(heading), speed * math.sin(heading)
