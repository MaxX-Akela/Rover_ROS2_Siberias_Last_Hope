import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from px4_msgs.msg import VehicleOdometry

from rover_slam.conversions import attitude_ned_frd, enu_to_ned

NAN = float('nan')


class VisionToPx4(Node):
    """SLAM pose (TF map -> base_link, ENU/FLU) -> /fmu/in/vehicle_visual_odometry (NED/FRD).

    PX4 side: EKF2_EV_CTRL = 9 (horizontal position + yaw), EKF2_GPS_CTRL = 0.
    Each TF sample is sent once; samples older than max_age are dropped.
    """

    def __init__(self):
        super().__init__('vision_to_px4')
        self.declare_parameter('px4_namespace', '/fmu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('max_age', 0.5)                # s, drop poses older than this
        self.declare_parameter('position_variance', 0.01)     # m^2
        self.declare_parameter('orientation_variance', 0.01)  # rad^2
        self._map = self.get_parameter('map_frame').value
        self._base = self.get_parameter('base_frame').value
        self._max_age = self.get_parameter('max_age').value
        self._pos_var = self.get_parameter('position_variance').value
        self._ori_var = self.get_parameter('orientation_variance').value
        ns = self.get_parameter('px4_namespace').value.rstrip('/')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self._pub = self.create_publisher(
            VehicleOdometry, f'{ns}/in/vehicle_visual_odometry', px4_qos)
        self._last_stamp = None
        self._waiting = True
        self.create_timer(1.0 / self.get_parameter('rate').value, self._on_timer)
        self.get_logger().info(
            f'vision_to_px4: TF {self._map} -> {self._base} => {ns}/in/vehicle_visual_odometry')

    def _on_timer(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map, self._base, Time())
        except TransformException as e:
            if not self._waiting:
                self.get_logger().warning(f'no TF {self._map} -> {self._base}: {e}',
                                          throttle_duration_sec=5.0)
            return
        if self._waiting:
            self._waiting = False
            self.get_logger().info('SLAM pose received')
        stamp = Time.from_msg(tf.header.stamp)
        if stamp == self._last_stamp:
            return
        now = self.get_clock().now()
        if now - stamp > Duration(seconds=self._max_age):
            self.get_logger().warning('SLAM pose is stale, not sent', throttle_duration_sec=5.0)
            return
        self._last_stamp = stamp

        t, r = tf.transform.translation, tf.transform.rotation
        q = (r.w, r.x, r.y, r.z)
        if not all(math.isfinite(v) for v in (t.x, t.y, t.z, *q)):
            return
        msg = VehicleOdometry()
        msg.timestamp = int(now.nanoseconds / 1000)
        msg.timestamp_sample = int(stamp.nanoseconds / 1000)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(v) for v in enu_to_ned((t.x, t.y, t.z))]
        msg.q = [float(v) for v in attitude_ned_frd(q)]
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [NAN] * 3
        msg.angular_velocity = [NAN] * 3
        msg.position_variance = [self._pos_var, self._pos_var, self._pos_var]
        msg.orientation_variance = [self._ori_var] * 3
        msg.velocity_variance = [NAN] * 3
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
