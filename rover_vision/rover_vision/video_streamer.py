"""RTSP video server: republishes ROS image topics as H.264 streams for QGroundControl.

One RTSP mount point per camera (/front, /rear, ...), frames come from ROS topics only.
Pipeline per mount: appsrc -> videoconvert -> x264enc -> rtph264pay.
"""
import threading

import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402

# ROS image encoding -> (GStreamer raw format, bytes per pixel)
FORMATS = {
    'rgb8': ('RGB', 3),
    'bgr8': ('BGR', 3),
    'rgba8': ('RGBA', 4),
    'bgra8': ('BGRA', 4),
    'mono8': ('GRAY8', 1),
    'yuv422_yuy2': ('YUY2', 2),
}

LAUNCH = (
    '( appsrc name=src is-live=true format=time do-timestamp=true '
    'max-buffers=2 leaky-type=downstream '
    '! videoconvert '
    '! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} key-int-max={fps} '
    '! video/x-h264,profile=baseline '
    '! h264parse '
    '! rtph264pay name=pay0 pt=96 config-interval=1 )'
)


def mount_points(topics):
    """Map every topic to a unique RTSP mount: /front/camera/image_raw -> /front."""
    mounts = {}
    for topic in topics:
        parts = [p for p in topic.split('/') if p]
        if not parts:
            continue
        mount = '/' + parts[0]
        if mount in mounts.values():
            mount = '/' + '_'.join(parts)
        mounts[topic] = mount
    return mounts


def gst_stride(fmt, width):
    """Row stride GStreamer expects for a tightly packed raw video frame."""
    bpp = {'RGB': 3, 'BGR': 3, 'RGBA': 4, 'BGRA': 4, 'GRAY8': 1, 'YUY2': 2}[fmt]
    return (width * bpp + 3) & ~3


def pack_rows(data, step, row_bytes, stride, height):
    """Re-pack rows from the ROS stride (`step`) to the GStreamer stride."""
    pad = bytes(stride - row_bytes)
    out = bytearray()
    for y in range(height):
        out += data[y * step:y * step + row_bytes]
        out += pad
    return bytes(out)


class Stream:
    """State of one camera: its topic, mount point and the currently attached appsrc."""

    def __init__(self, topic, mount, fps):
        self.topic = topic
        self.mount = mount
        self.min_interval = 1.0 / fps if fps > 0 else 0.0
        self.lock = threading.Lock()
        self.appsrc = None
        self.caps_key = None
        self.last_caps = None   # (format, width, height) of the last frame seen
        self.last_push = 0.0
        self.have_frame = threading.Event()


class VideoStreamer(Node):

    def __init__(self):
        super().__init__('video_streamer')
        self.declare_parameter('port', 8554)
        self.declare_parameter('bitrate', 2000)  # kbit/s
        self.declare_parameter('fps', 30)
        self.declare_parameter('topics', ['/front/camera/image_raw'])
        self.declare_parameter('host', '')  # only used to print the URLs

        self.port = int(self.get_parameter('port').value)
        self.bitrate = int(self.get_parameter('bitrate').value)
        self.fps = int(self.get_parameter('fps').value)
        topics = list(self.get_parameter('topics').value)
        host = self.get_parameter('host').value or '<rover-ip>'

        Gst.init(None)
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(self.port))
        mounts = self.server.get_mount_points()

        self.streams = {}
        for topic, mount in mount_points(topics).items():
            stream = Stream(topic, mount, self.fps)
            self.streams[topic] = stream

            factory = GstRtspServer.RTSPMediaFactory()
            factory.set_launch(LAUNCH.format(bitrate=self.bitrate, fps=self.fps))
            factory.set_shared(True)  # one encoder for any number of viewers
            factory.connect('media-configure', self.on_media_configure, stream)
            mounts.add_factory(mount, factory)

            self.create_subscription(
                Image, topic, lambda msg, s=stream: self.on_image(s, msg), qos_profile_sensor_data)
            self.get_logger().info(f'{topic} -> rtsp://{host}:{self.port}{mount}')

        self.server.attach(None)
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()

    # ---- GLib thread -------------------------------------------------------------------
    def on_media_configure(self, _factory, media, stream):
        appsrc = media.get_element().get_by_name('src')
        # caps must be known before the pipeline starts, otherwise the SDP cannot be built
        if not stream.have_frame.wait(timeout=5.0):
            self.get_logger().warning(f'no frames on {stream.topic} yet, stream may not start')
        with stream.lock:
            stream.appsrc = appsrc
            stream.caps_key = None
            if stream.last_caps:
                self.set_caps(stream, *stream.last_caps)
        media.connect('unprepared', self.on_media_unprepared, stream)
        self.get_logger().info(f'client connected to {stream.mount}')

    def on_media_unprepared(self, _media, stream):
        with stream.lock:
            stream.appsrc = None
        self.get_logger().info(f'no more clients on {stream.mount}')

    def set_caps(self, stream, fmt, width, height):
        caps = Gst.Caps.from_string(
            f'video/x-raw,format={fmt},width={width},height={height},framerate={self.fps}/1')
        stream.appsrc.set_property('caps', caps)
        stream.caps_key = (fmt, width, height)

    # ---- ROS thread --------------------------------------------------------------------
    def on_image(self, stream, msg):
        info = FORMATS.get(msg.encoding)
        if info is None:
            self.get_logger().error(
                f'{stream.topic}: unsupported encoding {msg.encoding}', throttle_duration_sec=10.0)
            return
        fmt, bpp = info
        key = (fmt, msg.width, msg.height)
        stream.last_caps = key
        stream.have_frame.set()

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stream.last_push < stream.min_interval:
            return
        with stream.lock:
            if stream.appsrc is None:
                return
            stride = gst_stride(fmt, msg.width)
            data = bytes(msg.data)
            if msg.step != stride:
                data = pack_rows(data, msg.step, msg.width * bpp, stride, msg.height)
            if stream.caps_key != key:
                self.set_caps(stream, *key)
            stream.last_push = now
            stream.appsrc.emit('push-buffer', Gst.Buffer.new_wrapped(data))

    def shutdown(self):
        self.loop.quit()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
