import math

import pytest

from rover_fcu_bridge.conversions import (
    attitude_enu_flu, direct_setpoints, frd_to_flu, ned_to_enu, quat_rotate, velocity_setpoint,
    yaw_of)


def yaw_quat(psi):
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


@pytest.mark.parametrize('heading_ned', [0.0, math.pi / 2, math.pi / 4, -1.0, 3.0])
def test_yaw_ned_to_enu(heading_ned):
    # NED yaw 0 = north, clockwise positive; ENU yaw 0 = east, counter-clockwise positive
    q = attitude_enu_flu(yaw_quat(heading_ned))
    expected = math.atan2(math.sin(math.pi / 2 - heading_ned), math.cos(math.pi / 2 - heading_ned))
    assert yaw_of(q) == pytest.approx(expected)


def test_body_axes_follow_the_vehicle():
    # facing north: body x (forward) is world north = ENU +y
    q = attitude_enu_flu(yaw_quat(0.0))
    assert quat_rotate(q, (1, 0, 0)) == pytest.approx((0, 1, 0))
    # body y (left) is west = ENU -x
    assert quat_rotate(q, (0, 1, 0)) == pytest.approx((-1, 0, 0))
    # facing east
    q = attitude_enu_flu(yaw_quat(math.pi / 2))
    assert quat_rotate(q, (1, 0, 0)) == pytest.approx((1, 0, 0))


def test_vectors():
    assert ned_to_enu((1, 2, 3)) == (2, 1, -3)
    assert frd_to_flu((1, 2, 3)) == (1, -2, -3)


def test_direct():
    assert direct_setpoints(0.5, 0.0, 1.0, 2.0) == (0.5, 0.0)
    assert direct_setpoints(-5.0, 1.0, 1.0, 2.0) == (-1.0, -0.5)  # left turn = negative
    assert direct_setpoints(0.0, 1.0, 1.0, 2.0, invert_steering=True) == (0.0, 0.5)


def test_velocity_stop_keeps_heading():
    vn, ve = velocity_setpoint(0.0, 0.0, math.pi / 2, 0.5)  # facing east, stopped
    assert math.hypot(vn, ve) == pytest.approx(0.02)
    assert math.atan2(ve, vn) == pytest.approx(math.pi / 2)


def test_velocity_turn_and_no_reverse():
    vn, ve = velocity_setpoint(-1.0, 1.0, 0.0, 0.5)  # reverse request, ccw turn
    assert math.hypot(vn, ve) == pytest.approx(0.02)          # no reverse
    assert math.atan2(ve, vn) == pytest.approx(-0.5)          # ccw = NED yaw decreases
