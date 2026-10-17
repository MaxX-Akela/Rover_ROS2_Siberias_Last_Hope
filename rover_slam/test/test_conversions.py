import math

import pytest

from rover_fcu_bridge.conversions import attitude_enu_flu, ned_to_enu, yaw_of
from rover_slam.conversions import attitude_ned_frd, enu_to_ned


def yaw_quat(psi):
    return (math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2))


def test_position_roundtrip():
    p = (1.0, 2.0, 3.0)
    assert enu_to_ned(p) == (2.0, 1.0, -3.0)
    assert ned_to_enu(enu_to_ned(p)) == pytest.approx(p)


def test_east_is_ned_y():
    assert enu_to_ned((1.0, 0.0, 0.0)) == (0.0, 1.0, -0.0)


@pytest.mark.parametrize('yaw_enu', [0.0, math.pi / 2, -1.0, 2.5])
def test_attitude_roundtrip(yaw_enu):
    q_ned = attitude_ned_frd(yaw_quat(yaw_enu))
    assert attitude_enu_flu(q_ned) == pytest.approx(yaw_quat(yaw_enu), abs=1e-9) or \
        attitude_enu_flu(q_ned) == pytest.approx(tuple(-v for v in yaw_quat(yaw_enu)), abs=1e-9)


@pytest.mark.parametrize('yaw_enu', [0.0, math.pi / 2, -1.0, 2.5])
def test_heading_ned(yaw_enu):
    # facing east (ENU yaw 0) = NED yaw +90 deg; NED yaw is clockwise from north
    q_ned = attitude_ned_frd(yaw_quat(yaw_enu))
    expected = math.atan2(math.sin(math.pi / 2 - yaw_enu), math.cos(math.pi / 2 - yaw_enu))
    assert yaw_of(q_ned) == pytest.approx(expected)
