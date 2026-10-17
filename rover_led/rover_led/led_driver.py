import rclpy
from led_msgs.msg import LEDState, LEDStateArray
from led_msgs.srv import SetLEDs
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rover_led.ws2812 import SPI_SPEED_HZ, encode

MAX_LEDS = 160  # spidev transfers are limited to bufsiz (4096 bytes by default): 24 bytes per LED


class LedDriver(Node):
    """WS2812 strip on SPI MOSI (spidev, works on Raspberry Pi 4 and 5).

    Service led/set_leds (led_msgs/SetLEDs) sets the colours, led/state (LEDStateArray, latched)
    publishes the current ones. Colours are shown scaled by `brightness`; led/state holds the
    unscaled values.
    """

    def __init__(self):
        super().__init__('led_driver')
        self.declare_parameter('led_count', 30)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', SPI_SPEED_HZ)
        self.declare_parameter('brightness', 0.3)
        count = self.get_parameter('led_count').value
        if not 0 < count <= MAX_LEDS:
            raise ValueError(f'led_count must be 1..{MAX_LEDS}')
        self._brightness = min(max(self.get_parameter('brightness').value, 0.0), 1.0)
        self._pixels = [(0, 0, 0)] * count
        self._spi = self._open_spi()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self._state_pub = self.create_publisher(LEDStateArray, 'led/state', qos)
        self.create_service(SetLEDs, 'led/set_leds', self._on_set_leds)
        self._show()
        self.get_logger().info(f'led driver started: {count} LEDs, brightness {self._brightness}')

    def _open_spi(self):
        bus, dev = self.get_parameter('spi_bus').value, self.get_parameter('spi_device').value
        try:
            import spidev
            spi = spidev.SpiDev()
            spi.open(bus, dev)
            spi.max_speed_hz = self.get_parameter('spi_speed_hz').value
            spi.mode = 0
            return spi
        except (ImportError, OSError) as e:
            self.get_logger().error(f'cannot open /dev/spidev{bus}.{dev} ({e}); running without output')
            return None

    def _show(self):
        if self._spi is not None:
            try:
                self._spi.writebytes2(encode(self._pixels, self._brightness))
            except OSError as e:
                self.get_logger().error(f'SPI write failed: {e}', throttle_duration_sec=5.0)
        msg = LEDStateArray()
        for i, (r, g, b) in enumerate(self._pixels):
            msg.leds.append(LEDState(index=i, r=r, g=g, b=b))
        self._state_pub.publish(msg)

    def _on_set_leds(self, req, res):
        count = len(self._pixels)
        bad = [led.index for led in req.leds if led.index >= count]
        if bad:
            res.message = f'LED index {bad[0]} out of range (0..{count - 1})'
            return res
        for led in req.leds:
            self._pixels[led.index] = (led.r, led.g, led.b)
        self._show()
        res.success = True
        return res

    def destroy_node(self):
        if self._spi is not None:
            self._pixels = [(0, 0, 0)] * len(self._pixels)
            self._show()
            self._spi.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
