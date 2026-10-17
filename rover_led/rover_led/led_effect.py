import math

import rclpy
from led_msgs.msg import LEDState
from led_msgs.srv import SetLEDs
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rover_msgs.srv import SetLEDEffect

from px4_msgs.msg import BatteryStatus, VehicleStatus

from rover_led.effects import EFFECTS, EffectEngine
from rover_led.notify import RoverState, low_battery, merge_table, nest, pick_event

MODES = {
    'NAVIGATION_STATE_MANUAL': 'manual',
    'NAVIGATION_STATE_STAB': 'stabilized',
    'NAVIGATION_STATE_POSCTL': 'position',
    'NAVIGATION_STATE_OFFBOARD': 'offboard',
    'NAVIGATION_STATE_AUTO_LOITER': 'hold',
}
# disarmed by the kill switch / lockdown / failure detector = emergency stop
KILL_REASONS = {getattr(VehicleStatus, n) for n in (
    'ARM_DISARM_REASON_KILL_SWITCH', 'ARM_DISARM_REASON_LOCKDOWN',
    'ARM_DISARM_REASON_FAILURE_DETECTOR') if hasattr(VehicleStatus, n)}


class LedEffect(Node):
    """Shows PX4 state (vehicle_status, battery_status over uXRCE-DDS) on the strip.

    The event -> effect table is the `notify` parameter group (config/led_notify.yaml); the idle
    effect (disarmed) is rainbow. Service led/set_effect overrides the effect until the next event
    change. Frames go to the led_driver through led/set_leds.
    """

    def __init__(self):
        super().__init__('led_effect', automatically_declare_parameters_from_overrides=True)
        self._count = self._param('led_count', 30)
        self._timeout = self._param('fcu_timeout', 2.0)
        self._cells = self._param('battery_cells', 0)  # 0 = cell_count from battery_status
        ns = self._param('px4_namespace', '/fmu').rstrip('/')
        status_topic = self._param('vehicle_status_topic', 'vehicle_status_v1')
        self._table = merge_table(nest(
            {k: p.value for k, p in self.get_parameters_by_prefix('notify').items()}))

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self._state = RoverState()
        self._last_status = None
        self._battery_low = False
        self.create_subscription(VehicleStatus, f'{ns}/out/{status_topic}',
                                 self._on_status, px4_qos)
        self.create_subscription(BatteryStatus, f'{ns}/out/battery_status',
                                 self._on_battery, px4_qos)

        self._engine = EffectEngine(self._count)
        self._event = None
        self._sent = [None] * self._count
        self._pending = None
        self._client = self.create_client(SetLEDs, 'led/set_leds')
        self.create_service(SetLEDEffect, 'led/set_effect', self._on_set_effect)
        self.create_timer(1.0 / self._param('rate', 25.0), self._on_timer)
        self.get_logger().info('led_effect started')

    def _param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- PX4 ----------------------------------------------------------
    def _on_status(self, m):
        s = self._state
        self._last_status = self._now()
        s.seen = True
        s.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        s.mode = next((name for const, name in MODES.items()
                       if getattr(VehicleStatus, const, None) == m.nav_state), '')
        s.failsafe = bool(m.failsafe)
        s.gcs_lost = bool(getattr(m, 'gcs_connection_lost', False))
        terminated = m.nav_state == getattr(VehicleStatus, 'NAVIGATION_STATE_TERMINATION', -1)
        s.emergency = terminated or (
            not s.armed and getattr(m, 'latest_disarming_reason', None) in KILL_REASONS)

    def _on_battery(self, m):
        if not math.isfinite(m.voltage_v):
            return
        cells = self._cells or m.cell_count
        threshold = self._table['low_battery'].get('threshold', 3.6)
        self._battery_low = low_battery(
            m.voltage_v, cells, m.warning, threshold,
            getattr(BatteryStatus, 'WARNING_LOW', None))

    # ---- effects ------------------------------------------------------
    def _apply(self, entry, now):
        color = (int(entry.get('r', 0)), int(entry.get('g', 0)), int(entry.get('b', 0)))
        try:
            self._engine.set(entry.get('effect', 'fill'), color, now)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)

    def _on_set_effect(self, req, res):
        if req.effect not in EFFECTS:
            res.message = f"unknown effect '{req.effect}', expected one of {EFFECTS}"
            return res
        self._engine.set(req.effect, (req.r, req.g, req.b), self._now())
        res.success = True
        return res

    def _on_timer(self):
        now = self._now()
        s = self._state
        s.connected = self._last_status is not None and now - self._last_status < self._timeout
        s.low_battery = self._battery_low
        event = pick_event(s, self._table)
        if event != self._event:
            self._event = event
            self.get_logger().info(f'LED event: {event}')
            self._apply(self._table[event], now)
        self._send(self._engine.frame(now))

    def _send(self, pixels):
        if self._pending is not None and not self._pending.done():
            return
        if not self._client.service_is_ready():
            self._sent = [None] * self._count  # driver (re)started: resend everything
            return
        req = SetLEDs.Request()
        for i, px in enumerate(pixels):
            if px != self._sent[i]:
                req.leds.append(LEDState(index=i, r=px[0], g=px[1], b=px[2]))
        if not req.leds:
            return
        self._pending = self._client.call_async(req)
        self._pending.add_done_callback(lambda f, p=list(pixels): self._on_sent(f, p))

    def _on_sent(self, future, pixels):
        if future.result() is not None and future.result().success:
            self._sent = pixels


def main(args=None):
    rclpy.init(args=args)
    node = LedEffect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
