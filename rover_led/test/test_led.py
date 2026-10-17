import pytest

from rover_led.effects import EffectEngine, render
from rover_led.notify import DEFAULT_TABLE, RoverState, low_battery, merge_table, nest, pick_event
from rover_led.ws2812 import BIT0, BIT1, RESET_BYTES, encode


def test_encode_grb_order_and_brightness():
    out = encode([(255, 0, 0)], brightness=1.0)
    assert len(out) == 24 + RESET_BYTES
    assert set(out[:8]) == {BIT0}                 # green
    assert set(out[8:16]) == {BIT1}               # red
    assert set(out[16:24]) == {BIT0}              # blue
    assert set(encode([(255, 255, 255)], 0.0)[:24]) == {BIT0}


def test_render_basic():
    assert render('fill', 0, 3, (1, 2, 3)) == [(1, 2, 3)] * 3
    assert render('blink', 0.0, 2, (9, 9, 9)) == [(9, 9, 9)] * 2
    assert render('blink', 0.3, 2, (9, 9, 9)) == [(0, 0, 0)] * 2
    assert render('wipe', 0.0, 4, (9, 9, 9)) == [(0, 0, 0)] * 4
    assert render('wipe', 10.0, 4, (9, 9, 9)) == [(9, 9, 9)] * 4
    assert render('fade', 10.0, 2, (10, 20, 30), [(0, 0, 0)] * 2) == [(10, 20, 30)] * 2
    assert render('rainbow', 0, 6, None)[0] != render('rainbow', 0, 6, None)[3]
    assert len(set(render('rainbow_fill', 1.0, 6, None))) == 1
    assert render('nope', 0, 1, (0, 0, 0)) is None


def test_flash_reverts_to_previous_effect():
    e = EffectEngine(2)
    e.set('fill', (0, 255, 0), 0.0)
    assert e.frame(0.0) == [(0, 255, 0)] * 2
    e.set('flash', (255, 0, 0), 1.0)
    assert e.frame(1.0) == [(255, 0, 0)] * 2
    assert e.frame(1.15) == [(0, 0, 0)] * 2
    assert e.frame(2.0) == [(0, 255, 0)] * 2


def test_engine_rejects_unknown_effect():
    with pytest.raises(ValueError):
        EffectEngine(1).set('nope', (0, 0, 0), 0.0)


def test_pick_event_priority():
    table = DEFAULT_TABLE
    s = RoverState()
    assert pick_event(s, table) == 'startup'
    s.seen = True
    assert pick_event(s, table) == 'disconnected'
    s.connected = True
    assert pick_event(s, table) == 'disarmed'
    s.armed, s.mode = True, 'offboard'
    assert pick_event(s, table) == 'offboard'
    s.mode = 'acro'
    assert pick_event(s, table) == 'armed'
    s.gcs_lost = True
    assert pick_event(s, table) == 'gcs_lost'
    s.low_battery = True
    assert pick_event(s, table) == 'low_battery'
    s.failsafe = True
    assert pick_event(s, table) == 'failsafe'
    s.emergency = True
    assert pick_event(s, table) == 'emergency_stop'


def test_table_merge():
    table = merge_table(nest({'armed.r': 7, 'custom.effect': 'fill'}))
    assert table['armed']['r'] == 7 and table['armed']['g'] == DEFAULT_TABLE['armed']['g']
    assert table['custom'] == {'effect': 'fill'}
    assert merge_table({})['disarmed']['effect'] == 'rainbow'


def test_low_battery():
    assert low_battery(10.8, 3, 0, 3.6, 1) is False
    assert low_battery(10.5, 3, 0, 3.6, 1) is True
    assert low_battery(12.0, 3, 1, 3.6, 1) is True      # PX4 warning
    assert low_battery(0.0, 3, 0, 3.6, 1) is False      # no battery connected
    assert low_battery(10.0, 0, 0, 3.6, 1) is False     # unknown cell count
