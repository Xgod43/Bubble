from __future__ import annotations

import time

try:
    import RPi.GPIO as GPIO
except Exception as exc:  # pragma: no cover - only available on Raspberry Pi
    GPIO = None
    _GPIO_IMPORT_ERROR = exc
else:
    _GPIO_IMPORT_ERROR = None


class HX711:
    """Small HX711 driver compatible with the legacy hx711v0_5_1 API."""

    _GAIN_PULSES = {
        ("A", 128): 1,
        ("B", 32): 2,
        ("A", 64): 3,
    }

    def __init__(self, dout_pin, pd_sck_pin, gain=128):
        if GPIO is None:
            raise RuntimeError(f"RPi.GPIO-compatible module is required: {_GPIO_IMPORT_ERROR}")

        self.dout_pin = int(dout_pin)
        self.pd_sck_pin = int(pd_sck_pin)
        self.gain = int(gain)
        self.byte_format = "MSB"
        self.bit_format = "MSB"
        self.offset = {"A": 0.0, "B": 0.0}
        self.reference_unit = {"A": 1.0, "B": 1.0}
        self.readyCallbackEnabled = False

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pd_sck_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dout_pin, GPIO.IN)
        self._configure_gain(self.gain)

    def setReadingFormat(self, byte_format="MSB", bit_format="MSB"):
        self.byte_format = str(byte_format).upper()
        self.bit_format = str(bit_format).upper()
        if self.byte_format != "MSB" or self.bit_format != "MSB":
            raise ValueError("This HX711 driver supports MSB/MSB reading format only.")

    def setGain(self, gain=128):
        self._configure_gain(gain)

    def _configure_gain(self, gain=128):
        gain = int(gain)
        if ("A", gain) not in self._GAIN_PULSES:
            raise ValueError("HX711 channel A supports gain 128 or 64.")
        self.gain = gain
        self._gain_pulses = self._GAIN_PULSES[("A", self.gain)]

    def setOffset(self, offset, channel="A"):
        self.offset[self._normalize_channel(channel)] = float(offset)

    def setReferenceUnit(self, reference_unit, channel="A"):
        reference_unit = float(reference_unit)
        if abs(reference_unit) < 1e-12:
            raise ValueError("reference_unit must be non-zero.")
        self.reference_unit[self._normalize_channel(channel)] = reference_unit

    def reset(self):
        GPIO.output(self.pd_sck_pin, GPIO.LOW)
        time.sleep(0.001)
        self.powerDown()
        self.powerUp()

    def powerDown(self):
        GPIO.output(self.pd_sck_pin, GPIO.LOW)
        GPIO.output(self.pd_sck_pin, GPIO.HIGH)
        time.sleep(0.00008)

    def powerUp(self):
        GPIO.output(self.pd_sck_pin, GPIO.LOW)
        time.sleep(0.001)

    def isReady(self):
        return GPIO.input(self.dout_pin) == GPIO.LOW

    def getLong(self, channel="A"):
        channel = self._normalize_channel(channel)
        pulses = self._pulses_for_channel(channel)
        return self._read_raw(pulses)

    def getWeight(self, channel="A"):
        channel = self._normalize_channel(channel)
        value = self.getLong(channel)
        return (float(value) - self.offset[channel]) / self.reference_unit[channel]

    def disableReadyCallback(self):
        self.readyCallbackEnabled = False

    def _normalize_channel(self, channel):
        channel = str(channel).upper()
        if channel not in {"A", "B"}:
            raise ValueError("HX711 channel must be A or B.")
        return channel

    def _pulses_for_channel(self, channel):
        if channel == "B":
            return self._GAIN_PULSES[("B", 32)]
        return self._GAIN_PULSES[("A", self.gain)]

    def _wait_ready(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while GPIO.input(self.dout_pin) != GPIO.LOW:
            if time.monotonic() >= deadline:
                raise TimeoutError("HX711 is not ready.")
            time.sleep(0.001)

    def _read_raw(self, gain_pulses):
        self._wait_ready()
        value = 0

        for _ in range(24):
            GPIO.output(self.pd_sck_pin, GPIO.HIGH)
            value = (value << 1) | int(GPIO.input(self.dout_pin))
            GPIO.output(self.pd_sck_pin, GPIO.LOW)

        for _ in range(gain_pulses):
            GPIO.output(self.pd_sck_pin, GPIO.HIGH)
            GPIO.output(self.pd_sck_pin, GPIO.LOW)

        if value & 0x800000:
            value -= 0x1000000
        return value
