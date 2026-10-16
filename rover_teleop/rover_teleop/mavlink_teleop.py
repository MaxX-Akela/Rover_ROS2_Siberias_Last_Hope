"""Listens to QGroundControl through mavlink-router and publishes MANUAL_CONTROL as /cmd_vel."""
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from pymavlink import mavutil
from rclpy.node import Node

from rover_teleop.mapping import AXES, to_velocity

HEARTBEAT_PERIOD = 1.0


class MavlinkTeleop(Node):

    def __init__(self):
        super().__init__('rover_teleop')
        self.declare_parameter('connection', 'udpin:127.0.0.1:14560')
        self.declare_parameter('source_system', 1)
        self.declare_parameter('source_component', mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_axis', 'x')
        self.declare_parameter('angular_axis', 'r')
        self.declare_parameter('max_linear', 1.0)    # m/s at full stick
        self.declare_parameter('max_angular', 1.0)   # rad/s at full stick
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('z_center', 500)
        self.declare_parameter('timeout', 0.5)       # s without MANUAL_CONTROL -> zero Twist
        self.declare_parameter('publish_rate', 20.0)

        p = self.get_parameter
        for name in ('linear_axis', 'angular_axis'):
            if p(name).value not in AXES:
                raise ValueError(f'{name} must be one of {AXES}')
        self._timeout = p('timeout').value

        self._lock = threading.Lock()
        self._axes = None
        self._last_rx = None   # monotonic time of the last MANUAL_CONTROL
        self._stale = True

        self._pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)
        self.create_timer(1.0 / p('publish_rate').value, self._on_timer)

        self._running = True
        self._link = mavutil.mavlink_connection(
            p('connection').value,
            source_system=p('source_system').value,
            source_component=p('source_component').value)
        self._thread = threading.Thread(target=self._receive, daemon=True)
        self._thread.start()
        self.get_logger().info(f"listening for MANUAL_CONTROL on {p('connection').value}")

    def _receive(self):
        last_heartbeat = 0.0
        while self._running:
            # The router only forwards messages addressed to a system it has seen on this
            # endpoint, so announce ourselves as an onboard computer of the vehicle.
            now = time.monotonic()
            if now - last_heartbeat >= HEARTBEAT_PERIOD:
                last_heartbeat = now
                self._link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            try:
                msg = self._link.recv_match(type='MANUAL_CONTROL', blocking=True, timeout=0.2)
            except Exception as exc:  # noqa: BLE001 - keep the link alive
                self.get_logger().error(f'MAVLink receive failed: {exc}', throttle_duration_sec=5.0)
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            with self._lock:
                self._axes = {'x': msg.x, 'y': msg.y, 'z': msg.z, 'r': msg.r}
                self._last_rx = time.monotonic()

    def _on_timer(self):
        p = self.get_parameter
        with self._lock:
            axes, last_rx = self._axes, self._last_rx
        twist = Twist()
        fresh = last_rx is not None and time.monotonic() - last_rx <= self._timeout
        if fresh:
            vx, wz = to_velocity(
                axes, p('linear_axis').value, p('angular_axis').value,
                p('max_linear').value, p('max_angular').value, p('deadzone').value,
                p('invert_linear').value, p('invert_angular').value, p('z_center').value)
            twist.linear.x, twist.angular.z = vx, wz
        if fresh == self._stale:
            self._stale = not fresh
            self.get_logger().info(
                'MANUAL_CONTROL received' if fresh else
                f'no MANUAL_CONTROL for {self._timeout} s, publishing zero /cmd_vel')
        self._pub.publish(twist)

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._link.close()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
