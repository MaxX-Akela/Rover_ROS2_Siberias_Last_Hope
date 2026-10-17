"""LED effects (pure functions, no ROS imports). Pixels are (r, g, b) tuples, 0..255."""
import colorsys

EFFECTS = ('fill', 'blink', 'blink_fast', 'fade', 'wipe', 'flash', 'rainbow', 'rainbow_fill')

BLINK_HALF_PERIOD = 0.25
BLINK_FAST_HALF_PERIOD = 0.1
FADE_TIME = 0.5
WIPE_TIME = 0.8
FLASH_PULSE = 0.1    # s on / s off, twice
FLASH_TIME = 4 * FLASH_PULSE
RAINBOW_PERIOD = 4.0  # s per full cycle


def hsv(h):
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, 1.0, 1.0)
    return (round(r * 255), round(g * 255), round(b * 255))


def mix(a, b, k):
    return tuple(round(x + (y - x) * k) for x, y in zip(a, b))


def render(effect, t, count, color, start=None):
    """Frame of `effect` `t` seconds after it started.

    start: pixels shown before the effect (fade begins from them). Returns None for an unknown effect.
    """
    off = (0, 0, 0)
    if effect == 'fill':
        return [color] * count
    if effect in ('blink', 'blink_fast'):
        half = BLINK_HALF_PERIOD if effect == 'blink' else BLINK_FAST_HALF_PERIOD
        return [color if int(t / half) % 2 == 0 else off] * count
    if effect == 'fade':
        k = min(t / FADE_TIME, 1.0)
        start = start or [off] * count
        return [mix(s, color, k) for s in start]
    if effect == 'wipe':
        lit = int(min(t / WIPE_TIME, 1.0) * count + 0.999)
        return [color if i < lit else off for i in range(count)]
    if effect == 'flash':
        on = t < FLASH_TIME and int(t / FLASH_PULSE) % 2 == 0
        return [color if on else off] * count
    if effect == 'rainbow':
        return [hsv(t / RAINBOW_PERIOD + i / max(count, 1)) for i in range(count)]
    if effect == 'rainbow_fill':
        return [hsv(t / RAINBOW_PERIOD)] * count
    return None


def finished(effect, t):
    """One-shot effects end on their own; flash reverts to the previous effect."""
    return effect == 'flash' and t >= FLASH_TIME


class EffectEngine:
    """Current effect with its start time; flash is transient and falls back to the previous one."""

    def __init__(self, count):
        self.count = count
        self.pixels = [(0, 0, 0)] * count
        self._effect = ('fill', (0, 0, 0))
        self._previous = self._effect
        self._start = list(self.pixels)
        self._since = 0.0

    def set(self, effect, color, now):
        if render(effect, 0.0, self.count, color) is None:
            raise ValueError(f"unknown effect '{effect}', expected one of {EFFECTS}")
        if effect != 'flash':
            self._previous = (effect, color)
        elif self._effect[0] != 'flash':
            self._previous = self._effect
        self._start = list(self.pixels)
        self._effect = (effect, color)
        self._since = now

    def frame(self, now):
        effect, color = self._effect
        t = now - self._since
        if finished(effect, t):
            self._effect = self._previous
            self._start = list(self.pixels)
            self._since = now
            effect, color = self._effect
            t = 0.0
        self.pixels = render(effect, t, self.count, color, self._start)
        return self.pixels
