import pytest

from rover_teleop.mapping import axis_value, deadzone, to_velocity

ZERO = {'x': 0, 'y': 0, 'z': 500, 'r': 0}


def vel(axes, **kw):
    return to_velocity({**ZERO, **axes}, 'x', 'r', 2.0, 3.0, **kw)


def test_center_is_zero():
    assert vel({}) == (0.0, 0.0)


def test_full_forward_and_turn_right():
    assert vel({'x': 1000}) == (2.0, 0.0)
    # yaw stick right = clockwise = negative angular.z
    assert vel({'r': 1000}) == (0.0, -3.0)


def test_deadzone_and_rescale():
    assert vel({'x': 40}) == (0.0, 0.0)
    assert deadzone(1.0, 0.1) == pytest.approx(1.0)
    assert deadzone(0.55, 0.1) == pytest.approx(0.5)


def test_invert():
    assert vel({'x': 1000}, invert_linear=True) == (-2.0, 0.0)
    assert vel({'r': 1000}, invert_angular=True) == (0.0, 3.0)


def test_z_axis_and_invalid():
    assert axis_value(500, 'z') == 0.0
    assert axis_value(1000, 'z') == 1.0
    assert axis_value(0, 'z', z_center=0) == 0.0
    assert axis_value(-1000, 'z', z_center=0) == -1.0
    assert axis_value(32767, 'x') == 0.0
