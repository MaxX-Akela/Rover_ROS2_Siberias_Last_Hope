"""PX4 state -> LED event (pure functions, no ROS imports)."""
from dataclasses import dataclass

DEFAULT_TABLE = {
    'startup': {'effect': 'fill', 'r': 255, 'g': 255, 'b': 255},
    'disconnected': {'effect': 'blink', 'r': 255, 'g': 50, 'b': 50},
    'gcs_lost': {'effect': 'blink', 'r': 255, 'g': 120, 'b': 0},
    'disarmed': {'effect': 'rainbow'},
    'armed': {'effect': 'fill', 'r': 0, 'g': 255, 'b': 0},
    'manual': {'effect': 'fill', 'r': 245, 'g': 155, 'b': 0},
    'stabilized': {'effect': 'fill', 'r': 30, 'g': 180, 'b': 50},
    'position': {'effect': 'fill', 'r': 50, 'g': 100, 'b': 220},
    'offboard': {'effect': 'fill', 'r': 220, 'g': 20, 'b': 250},
    'hold': {'effect': 'fill', 'r': 255, 'g': 255, 'b': 40},
    'failsafe': {'effect': 'blink_fast', 'r': 255, 'g': 0, 'b': 0},
    'low_battery': {'effect': 'blink_fast', 'r': 255, 'g': 0, 'b': 0, 'threshold': 3.6},
    'emergency_stop': {'effect': 'flash', 'r': 255, 'g': 0, 'b': 0},
}


@dataclass
class RoverState:
    seen: bool = False          # vehicle_status received at least once
    connected: bool = False     # ... and still arriving
    armed: bool = False
    mode: str = ''              # manual, stabilized, position, offboard, hold, ... or ''
    failsafe: bool = False
    gcs_lost: bool = False
    low_battery: bool = False
    emergency: bool = False


def pick_event(s, table):
    """Most important active event; returns a key of `table`."""
    if not s.seen:
        return 'startup'
    if not s.connected:
        return 'disconnected'
    if s.emergency:
        return 'emergency_stop'
    if s.failsafe:
        return 'failsafe'
    if s.low_battery:
        return 'low_battery'
    if s.gcs_lost:
        return 'gcs_lost'
    if not s.armed:
        return 'disarmed'
    return s.mode if s.mode in table else 'armed'


def merge_table(overrides):
    """DEFAULT_TABLE updated per event with `overrides` ({event: {key: value}})."""
    table = {k: dict(v) for k, v in DEFAULT_TABLE.items()}
    for event, entry in overrides.items():
        table.setdefault(event, {}).update(entry)
    return table


def nest(flat):
    """{'armed.effect': 'fill', 'armed.r': 1} -> {'armed': {'effect': 'fill', 'r': 1}}."""
    out = {}
    for key, value in flat.items():
        event, _, field = key.partition('.')
        out.setdefault(event, {})[field] = value
    return out


def low_battery(voltage, cells, warning, threshold, warning_low):
    """Per-cell voltage under threshold, or PX4's own battery warning."""
    if warning_low is not None and warning >= warning_low:
        return True
    return cells > 0 and 1.0 < voltage / cells < threshold
