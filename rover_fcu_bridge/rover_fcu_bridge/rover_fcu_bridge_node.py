import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    RoverSteeringSetpoint,
    RoverThrottleSetpoint,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rover_msgs.msg import State
from rover_msgs.srv import ArmDisarm, GetTelemetry, Navigate, SetVelocity, Stop
from tf2_ros import TransformBroadcaster

from rover_fcu_bridge.conversions import (
    attitude_enu_flu, direct_setpoints, frd_to_flu, ned_to_enu, quat_rotate, velocity_setpoint)

NAN = float('nan')
CONTROL_MODES = ('direct', 'velocity')
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
    """Exposes rover_msgs services and /cmd_vel and translates them to PX4 offboard setpoints.

    Local frames: PX4 uses NED, the public API uses ENU ('map') or 'body'.

    /cmd_vel (geometry_msgs/Twist, linear.x + angular.z) is handled in one of two modes
    (parameter control_mode):
      direct    RoverThrottleSetpoint + RoverSteeringSetpoint (OffboardControlMode.thrust_and_torque).
                Normalised throttle/steering, no odometry or local position needed.
      velocity  TrajectorySetpoint velocity (OffboardControlMode.velocity). PX4 closes the speed
                and heading loops, so it needs a valid local position. No reverse.
    No /cmd_vel for cmd_vel_timeout seconds -> the rover is stopped. Publishing /cmd_vel
    cancels a running Navigate/SetVelocity service command and vice versa.
    PX4 must be armed and in Offboard mode (QGC flight mode menu, or the arm_disarm service).
    """

    def __init__(self):
        super().__init__('rover_fcu_bridge')

        self.declare_parameter('px4_namespace', '/fmu')
        self.declare_parameter('vehicle_status_topic', 'vehicle_status_v1')
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('carrot_lookahead', 1.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('control_mode', 'direct')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('max_linear', 1.0)     # m/s <-> throttle 1.0 (direct)
        self.declare_parameter('max_angular', 1.0)    # rad/s <-> steering 1.0 (direct)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('velocity_yaw_horizon', 0.5)  # s, heading lead per rad/s (velocity)
        self.declare_parameter('odometry_topic', 'vehicle_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self._control_mode = self.get_parameter('control_mode').value
        if self._control_mode not in CONTROL_MODES:
            raise ValueError(f'control_mode must be one of {CONTROL_MODES}')
        self._cmd_timeout = self.get_parameter('cmd_vel_timeout').value
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

        odom_topic = self.get_parameter('odometry_topic').value
        self.create_subscription(VehicleOdometry, f'{ns}/out/{odom_topic}',
                                 self._on_vehicle_odometry, px4_qos)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = (TransformBroadcaster(self)
                                if self.get_parameter('publish_tf').value else None)

        # ROS -> PX4
        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/in/offboard_control_mode', px4_qos)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/in/trajectory_setpoint', px4_qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/in/vehicle_command', px4_qos)
        self._throttle_pub = self.create_publisher(
            RoverThrottleSetpoint, f'{ns}/in/rover_throttle_setpoint', px4_qos)
        self._steering_pub = self.create_publisher(
            RoverSteeringSetpoint, f'{ns}/in/rover_steering_setpoint', px4_qos)

        # setpoint state: 'hold' | 'position' | 'velocity'
        self._mode = 'hold'
        self._target = (0.0, 0.0, NAN, 0.0)  # NED x, y, yaw, speed
        self._velocity = (0.0, 0.0)  # vx (body), yaw_rate

        # /cmd_vel state: latest command, time it arrived, and whether it is in charge
        self._cmd = (0.0, 0.0)
        self._cmd_time = None
        self._cmd_active = False
        self._cmd_stale = True
        self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value,
                                 self._on_cmd_vel, 10)

        self.create_service(Navigate, '~/navigate', self._on_navigate)
        self.create_service(SetVelocity, '~/set_velocity', self._on_set_velocity)
        self.create_service(GetTelemetry, '~/get_telemetry', self._on_get_telemetry)
        self.create_service(ArmDisarm, '~/arm_disarm', self._on_arm_disarm)
        self.create_service(Stop, '~/stop', self._on_stop)

        self.create_timer(1.0 / self.get_parameter('setpoint_rate').value, self._on_timer)
        self.get_logger().info(f'rover_fcu_bridge started, control_mode={self._control_mode}')

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

    # ---- /cmd_vel ------------------------------------------------------
    def _on_cmd_vel(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_time = self.get_clock().now()
        if not self._cmd_active:
            self._cmd_active = True
            self._mode = 'hold'  # /cmd_vel takes over from Navigate/SetVelocity

    def _cmd_vel_now(self):
        """Latest (vx, wz), or (0, 0) when /cmd_vel stopped arriving (watchdog)."""
        age = (self.get_clock().now() - self._cmd_time).nanoseconds * 1e-9
        stale = age > self._cmd_timeout
        if stale != self._cmd_stale:
            self._cmd_stale = stale
            if stale:
                self.get_logger().warning(f'no /cmd_vel for {self._cmd_timeout} s, stopping')
            else:
                self.get_logger().info('/cmd_vel received')
        return (0.0, 0.0) if stale else self._cmd

    # ---- odometry ------------------------------------------------------
    def _on_vehicle_odometry(self, m):
        # message arrays hold numpy scalars, rclpy setters want plain floats
        pos, q = [float(v) for v in m.position], [float(v) for v in m.q]
        if not (all(map(math.isfinite, pos)) and all(map(math.isfinite, q))):
            return
        # NED/FRD -> ENU/FLU
        x, y, z = ned_to_enu(pos)
        q_enu = attitude_enu_flu(q)  # (w, x, y, z), body FLU -> world ENU

        vel = [float(v) if math.isfinite(v) else 0.0 for v in m.velocity]
        if m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            v_body = frd_to_flu(vel)
        else:  # NED (FRD navigation frames are treated as NED, like the pose)
            v_enu = ned_to_enu(vel)
            v_body = quat_rotate((q_enu[0], -q_enu[1], -q_enu[2], -q_enu[3]), v_enu)
        w_body = frd_to_flu([float(v) if math.isfinite(v) else 0.0 for v in m.angular_velocity])

        stamp = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
        o = odom.pose.pose.orientation
        o.w, o.x, o.y, o.z = q_enu
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = v_body
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = w_body
        # covariance diagonals (x, y, z, roll, pitch, yaw); NED variances -> ENU axes
        pv, ov, vv = ([float(v) for v in a] for a in (
            m.position_variance, m.orientation_variance, m.velocity_variance))
        pose_var = [pv[1], pv[0], pv[2], ov[0], ov[1], ov[2]]
        twist_var = [vv[0], vv[1], vv[2], 0.0, 0.0, 0.0]
        for i in range(6):
            odom.pose.covariance[i * 7] = pose_var[i] if math.isfinite(pose_var[i]) else 0.0
            odom.twist.covariance[i * 7] = twist_var[i] if math.isfinite(twist_var[i]) else 0.0
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = odom_frame
            tf.child_frame_id = base_frame
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.translation.z = z
            tf.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf)

    # ---- setpoint streaming -------------------------------------------
    def _publish_direct(self, throttle, steering):
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.thrust_and_torque = True
        thr = RoverThrottleSetpoint()
        thr.timestamp = mode.timestamp
        thr.throttle_body_x = float(throttle)
        thr.throttle_body_y = NAN  # mecanum only
        steer = RoverSteeringSetpoint()
        steer.timestamp = mode.timestamp
        steer.normalized_steering_setpoint = float(steering)
        self._offboard_pub.publish(mode)
        self._throttle_pub.publish(thr)
        self._steering_pub.publish(steer)

    def _publish_velocity(self, vx, yaw_rate, pose):
        """Offboard velocity: speed and heading from a NED velocity vector (see conversions)."""
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.velocity = True
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [NAN] * 3
        sp.acceleration = [NAN] * 3
        sp.yaw = NAN
        sp.yawspeed = NAN
        if pose is None:
            self.get_logger().warning('velocity control needs a valid local position',
                                      throttle_duration_sec=5.0)
            sp.velocity = [0.0, 0.0, 0.0]
        else:
            vn, ve = velocity_setpoint(
                vx, yaw_rate, pose[2], self.get_parameter('velocity_yaw_horizon').value)
            sp.velocity = [vn, ve, 0.0]
        self._offboard_pub.publish(mode)
        self._setpoint_pub.publish(sp)

    def _on_timer(self):
        pose = self._pose_ned()
        idle = self._mode == 'hold' or (self._mode == 'position' and pose is None)
        if self._cmd_active or idle:
            # /cmd_vel or idle: zero command when /cmd_vel stops arriving (watchdog)
            vx, wz = self._cmd_vel_now() if self._cmd_active else (0.0, 0.0)
            if self._control_mode == 'direct':
                p = self.get_parameter
                self._publish_direct(*direct_setpoints(
                    vx, wz, p('max_linear').value, p('max_angular').value,
                    p('invert_steering').value))
            else:
                self._publish_velocity(vx, wz, pose)
            return

        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [NAN] * 3
        sp.velocity = [NAN] * 3
        sp.acceleration = [NAN] * 3
        sp.yaw = NAN
        sp.yawspeed = NAN

        if self._mode == 'position':
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
            self._offboard_pub.publish(mode)
            self._setpoint_pub.publish(sp)
        else:  # 'velocity' service command
            vx, yaw_rate = self._velocity
            self._publish_velocity(vx, yaw_rate, pose)

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
        self._cmd_active = False
        self._mode = 'position'
        res.success = True
        return res

    def _on_set_velocity(self, req, res):
        self._velocity = (req.vx, req.yaw_rate)
        self._cmd_active = False
        self._mode = 'velocity'
        res.success = True
        return res

    def _on_stop(self, req, res):
        self._cmd_active = False
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
