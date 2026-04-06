from rpi_ws281x import PixelStrip, Color
import time
import sys

LED_COUNT = 4
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 80
LED_INVERT = False
LED_CHANNEL = 0

strip = PixelStrip(
    LED_COUNT,
    LED_PIN,
    LED_FREQ_HZ,
    LED_DMA,
    LED_INVERT,
    LED_BRIGHTNESS,
    LED_CHANNEL,
)

try:
    strip.begin()
except RuntimeError as exc:
    msg = str(exc)
    if "Hardware revision is not supported" in msg:
        print("rpi_ws281x does not support this board revision (likely Raspberry Pi 5).")
        print("Use one of these options:")
        print("1) Use an SPI-based NeoPixel driver on GPIO10 (MOSI)")
        print("2) Use pigpio wave output (works on GPIO22)")
        print("3) Drive LEDs from a small external microcontroller")
    else:
        print(f"WS2812 init error: {msg}")
    sys.exit(1)


def fill(c):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, c)
    strip.show()


try:
    while True:
        fill(Color(255, 0, 0))  # Red
        time.sleep(0.5)
        fill(Color(0, 255, 0))  # Green
        time.sleep(0.5)
        fill(Color(0, 0, 255))  # Blue
        time.sleep(0.5)
        fill(Color(255, 255, 255))  # White
        time.sleep(0.5)
        fill(Color(0, 0, 0))  # Off
        time.sleep(0.5)
except KeyboardInterrupt:
    fill(Color(0, 0, 0))

