"""MANUAL_CONTROL axes -> (linear, angular) velocity. No ROS/MAVLink imports, easy to test."""

AXES = ('x', 'y', 'z', 'r')
INVALID = 32767  # INT16_MAX marks an unused axis in MANUAL_CONTROL


def axis_value(raw, axis, z_center=500):
    """Normalise a raw MANUAL_CONTROL axis to [-1, 1].

    x, y, r are -1000..1000. z (throttle) is 0..1000 with `z_center` as zero
    (500 for "center stick = zero", 0 if QGC sends negative thrust as -1000..1000).
    """
    if raw == INVALID:
        return 0.0
    if axis == 'z':
        scale = max(1000 - z_center, z_center, 1)
        value = (raw - z_center) / scale
    else:
        value = raw / 1000.0
    return max(-1.0, min(1.0, value))


def deadzone(value, width):
    """Zero inside +-width, rescaled so the output still reaches +-1."""
    if abs(value) <= width:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - width) / (1.0 - width)


def to_velocity(axes, linear_axis, angular_axis, max_linear, max_angular,
                dead_zone=0.05, invert_linear=False, invert_angular=False, z_center=500):
    """axes: dict with raw 'x', 'y', 'z', 'r'. Returns (vx [m/s], wz [rad/s]).

    Stick forward (x > 0) is forward motion. Yaw stick right (r > 0) is a clockwise turn,
    which is negative angular.z in ROS (REP 103, counter-clockwise positive).
    """
    lin = deadzone(axis_value(axes[linear_axis], linear_axis, z_center), dead_zone)
    ang = deadzone(axis_value(axes[angular_axis], angular_axis, z_center), dead_zone)
    if invert_linear:
        lin = -lin
    ang = -ang
    if invert_angular:
        ang = -ang
    return lin * max_linear, ang * max_angular
