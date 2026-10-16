"""Pure helpers of video_streamer; skipped where GStreamer bindings are missing."""
import pytest

pytest.importorskip('gi')
try:
    from rover_vision.video_streamer import gst_stride, mount_points, pack_rows
except ValueError:  # gi installed, but no Gst typelibs
    pytest.skip('GStreamer typelibs missing', allow_module_level=True)


def test_mount_points():
    assert mount_points(['/front/camera/image_raw', '/rear/camera/image_raw']) == {
        '/front/camera/image_raw': '/front', '/rear/camera/image_raw': '/rear'}


def test_mount_points_collision():
    mounts = mount_points(['/front/a/image_raw', '/front/b/image_raw'])
    assert len(set(mounts.values())) == 2


def test_stride():
    assert gst_stride('RGB', 1280) == 3840
    assert gst_stride('RGB', 641) == 1924
    assert gst_stride('GRAY8', 5) == 8


def test_pack_rows():
    padded = b'aaa\0\0\0' + b'ccc\0\0\0'
    assert pack_rows(padded, 6, 3, 4, 2) == b'aaa\0ccc\0'
