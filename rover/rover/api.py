import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rover_msgs.srv import ArmDisarm, GetTelemetry, Navigate, SetVelocity, Stop

SERVICE_PREFIX = '/rover_fcu_bridge'
DEFAULT_TIMEOUT = 5.0

_lock = threading.Lock()
_node = None
_executor = None
_thread = None
_clients = {}


class RoverError(RuntimeError):
    """Raised when a rover_fcu_bridge service is unavailable or reports a failure."""


def _ensure_node():
    global _node, _executor, _thread
    with _lock:
        if _node is not None:
            return _node
        if not rclpy.ok():
            rclpy.init()
        _node = Node('rover_api', start_parameter_services=False)
        _executor = SingleThreadedExecutor()
        _executor.add_node(_node)
        _thread = threading.Thread(target=_executor.spin, daemon=True)
        _thread.start()
        return _node


def _call(srv_type, name, request, timeout=DEFAULT_TIMEOUT):
    node = _ensure_node()
    key = (srv_type, name)
    with _lock:
        client = _clients.get(key)
        if client is None:
            client = node.create_client(srv_type, f'{SERVICE_PREFIX}/{name}')
            _clients[key] = client
    if not client.wait_for_service(timeout_sec=timeout):
        raise RoverError(f'service {SERVICE_PREFIX}/{name} is not available')
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise RoverError(f'service {SERVICE_PREFIX}/{name} timed out')
    response = future.result()
    if response is None or not response.success:
        message = getattr(response, 'message', '')
        raise RoverError(f'{name} failed: {message}')
    return response


def navigate(x=0.0, y=0.0, yaw=float('nan'), speed=0.0, frame_id='map'):
    """Drive to a point. Returns immediately; poll :func:`get_telemetry` to track progress.

    :param x, y: target position, m
    :param yaw: target heading, rad (NaN = keep heading)
    :param speed: cruise speed, m/s (0 = FCU default)
    :param frame_id: ``'map'`` (local ENU) or ``'body'`` (relative to the rover)
    """
    req = Navigate.Request(x=float(x), y=float(y), yaw=float(yaw),
                           speed=float(speed), frame_id=frame_id)
    return _call(Navigate, 'navigate', req)


def set_velocity(vx=0.0, yaw_rate=0.0):
    """Command forward speed (m/s) and turn rate (rad/s) in the body frame."""
    req = SetVelocity.Request(vx=float(vx), yaw_rate=float(yaw_rate))
    return _call(SetVelocity, 'set_velocity', req)


def get_telemetry(frame_id='map'):
    """Return a ``rover_msgs/State`` with the pose expressed in ``frame_id``."""
    return _call(GetTelemetry, 'get_telemetry', GetTelemetry.Request(frame_id=frame_id)).state


def arm():
    """Arm the rover."""
    return _call(ArmDisarm, 'arm_disarm', ArmDisarm.Request(arm=True))


def disarm():
    """Disarm the rover."""
    return _call(ArmDisarm, 'arm_disarm', ArmDisarm.Request(arm=False))


def stop():
    """Stop immediately and hold the current position."""
    return _call(Stop, 'stop', Stop.Request())
