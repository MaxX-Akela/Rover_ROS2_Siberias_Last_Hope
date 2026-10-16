import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rover_msgs.msg import State
from rover_msgs.srv import ArmDisarm, GetTelemetry, Navigate, SetVelocity, Stop

NAN = float('nan')
FRAMES = ('map', 'body')

NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: 'MANUAL',
    VehicleStatus.NAVIGATION_STATE_POSCTL: 'POSCTL',
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: 'MISSION',
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: 'HOLD',
    VehicleStatus.NAVIGATION_STATE_ACRO: 'ACRO',
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: 'OFFBOARD',
}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FcuBridgeNode(Node):
    """Exposes rover_msgs services and translates them to PX4 offboard setpoints.

    Local frames: PX4 uses NED, the public API uses ENU ('map') or 'body'.
    """

    def __init__(self):
        super().__init__('rover_fcu_bridge')

        self.declare_parameter('px4_namespace', '/fmu')
        self.declare_parameter('vehicle_status_topic', 'vehicle_status_v1')
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('carrot_lookahead', 1.0)
        self.declare_parameter('target_system', 1)
        ns = self.get_parameter('px4_namespace').value.rstrip('/')
        self._target_system = self.get_parameter('target_system').value
        self._lookahead = self.get_parameter('carrot_lookahead').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self._status = None
        self._local_pos = None
        self._battery = NAN

        # PX4 -> ROS
        status_topic = self.get_parameter('vehicle_status_topic').value
        self.create_subscription(VehicleStatus, f'{ns}/out/{status_topic}',
                                 lambda m: setattr(self, '_status', m), px4_qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/out/vehicle_local_position',
                                 lambda m: setattr(self, '_local_pos', m), px4_qos)
        self.create_subscription(BatteryStatus, f'{ns}/out/battery_status',
                                 lambda m: setattr(self, '_battery', m.voltage_v), px4_qos)

        # ROS -> PX4
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/in/offboard_control_mode', px4_qos)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', px4_qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/in/vehicle_command', px4_qos)

        # setpoint state: 'hold' | 'position' | 'velocity'
        self._mode = 'hold'
        self._target = (0.0, 0.0, NAN, 0.0)  # NED x, y, yaw, speed
        self._velocity = (0.0, 0.0)  # vx (body), yaw_rate

        self.create_service(Navigate, '~/navigate', self._on_navigate)
        self.create_service(SetVelocity, '~/set_velocity', self._on_set_velocity)
        self.create_service(GetTelemetry, '~/get_telemetry', self._on_get_telemetry)
        self.create_service(ArmDisarm, '~/arm_disarm', self._on_arm_disarm)
        self.create_service(Stop, '~/stop', self._on_stop)

        self.create_timer(1.0 / self.get_parameter('setpoint_rate').value, self._on_timer)
        self.get_logger().info('rover_fcu_bridge started')

    # ---- helpers -------------------------------------------------------
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _pose_ned(self):
        p = self._local_pos
        if p is None or not (p.xy_valid and math.isfinite(p.heading)):
            return None
        return p.x, p.y, p.heading

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self._target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._command_pub.publish(msg)

    # ---- setpoint streaming -------------------------------------------
    def _on_timer(self):
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [NAN] * 3
        sp.velocity = [NAN] * 3
        sp.acceleration = [NAN] * 3
        sp.yaw = NAN
        sp.yawspeed = NAN

        pose = self._pose_ned()
        if self._mode == 'position' and pose is not None:
            mode.position = True
            tx, ty, tyaw, speed = self._target
            dx, dy = tx - pose[0], ty - pose[1]
            dist = math.hypot(dx, dy)
            reach = speed * self._lookahead
            if speed > 0.0 and dist > reach > 0.0:
                k = reach / dist
                tx, ty = pose[0] + dx * k, pose[1] + dy * k
            sp.position = [tx, ty, 0.0]
            sp.yaw = tyaw if math.isfinite(tyaw) else math.atan2(dy, dx) if dist > 0.3 else NAN
        elif self._mode == 'velocity' and pose is not None:
            mode.velocity = True
            vx, yaw_rate = self._velocity
            sp.velocity = [vx * math.cos(pose[2]), vx * math.sin(pose[2]), 0.0]
            sp.yawspeed = yaw_rate
        else:
            mode.velocity = True
            sp.velocity = [0.0, 0.0, 0.0]
            sp.yawspeed = 0.0

        self._offboard_pub.publish(mode)
        self._setpoint_pub.publish(sp)

    # ---- services ------------------------------------------------------
    def _on_navigate(self, req, res):
        pose = self._pose_ned()
        if req.frame_id not in FRAMES:
            res.message = f"unknown frame_id '{req.frame_id}', expected one of {FRAMES}"
            return res
        if pose is None:
            res.message = 'no valid local position from FCU'
            return res
        if req.frame_id == 'map':  # ENU -> NED
            nx, ny = req.y, req.x
            nyaw = wrap(math.pi / 2 - req.yaw) if math.isfinite(req.yaw) else NAN
        else:  # body: x forward, y left
            c, s = math.cos(pose[2]), math.sin(pose[2])
            nx = pose[0] + req.x * c + req.y * s
            ny = pose[1] + req.x * s - req.y * c
            nyaw = wrap(pose[2] - req.yaw) if math.isfinite(req.yaw) else NAN
        self._target = (nx, ny, nyaw, max(req.speed, 0.0))
        self._mode = 'position'
        res.success = True
        return res

    def _on_set_velocity(self, req, res):
        self._velocity = (req.vx, req.yaw_rate)
        self._mode = 'velocity'
        res.success = True
        return res

    def _on_stop(self, req, res):
        self._mode = 'hold'
        res.success = True
        return res

    def _on_arm_disarm(self, req, res):
        if req.arm:
            # offboard setpoints are already streaming; switch mode, then arm
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        else:
            self._mode = 'hold'
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                           1.0 if req.arm else 0.0)
        res.success = True
        return res

    def _on_get_telemetry(self, req, res):
        if req.frame_id not in FRAMES:
            res.message = f"unknown frame_id '{req.frame_id}', expected one of {FRAMES}"
            return res
        pose = self._pose_ned()
        if pose is None or self._status is None:
            res.message = 'no telemetry from FCU yet'
            return res
        p = self._local_pos
        st = res.state
        st.armed = self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        st.mode = NAV_STATE_NAMES.get(self._status.nav_state, str(self._status.nav_state))
        if req.frame_id == 'map':  # NED -> ENU
            st.x, st.y = float(pose[1]), float(pose[0])
            st.yaw = float(wrap(math.pi / 2 - pose[2]))
        else:
            st.x, st.y, st.yaw = 0.0, 0.0, 0.0
        st.vx = float(p.vx * math.cos(pose[2]) + p.vy * math.sin(pose[2]))  # forward speed
        st.battery = float(self._battery)
        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)
    node = FcuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
