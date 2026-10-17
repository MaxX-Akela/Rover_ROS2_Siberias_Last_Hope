"""WS2812 over SPI (pure functions, no hardware imports).

SPI runs at 6.4 MHz: one SPI byte (1.25 us) is one WS2812 data bit, 0xC0 = "0", 0xF8 = "1".
"""
SPI_SPEED_HZ = 6_400_000
BIT0 = 0xC0
BIT1 = 0xF8
RESET_BYTES = 100  # > 80 us low

_BYTE_TO_SPI = [bytes(BIT1 if value & (0x80 >> i) else BIT0 for i in range(8))
                for value in range(256)]


def scale(color, brightness):
    return tuple(min(255, max(0, round(c * brightness))) for c in color)


def encode(pixels, brightness=1.0):
    """[(r, g, b), ...] -> bytes for spidev (WS2812 colour order is G, R, B)."""
    out = bytearray()
    for r, g, b in pixels:
        r, g, b = scale((r, g, b), brightness)
        out += _BYTE_TO_SPI[g] + _BYTE_TO_SPI[r] + _BYTE_TO_SPI[b]
    out += bytes(RESET_BYTES)
    return bytes(out)
