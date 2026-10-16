"""Python API for the rover, in the style of Clover.

Example::

    from rover import arm, navigate, get_telemetry

    arm()
    navigate(x=2.0, y=1.0, yaw=0.0, speed=0.5, frame_id='map')
    print(get_telemetry().x)
"""
from .api import (
    arm,
    disarm,
    get_telemetry,
    navigate,
    set_velocity,
    stop,
)

__all__ = ['arm', 'disarm', 'get_telemetry', 'navigate', 'set_velocity', 'stop']
