from __future__ import annotations

import os
import time

try:
    import RPi.GPIO as GPIO
except Exception as exc:  # pragma: no cover - only available on Raspberry Pi
    GPIO = None
    _GPIO_IMPORT_ERROR = exc
else:
    _GPIO_IMPORT_ERROR = None

try:
    import lgpio
except Exception as exc:  # pragma: no cover - only available on Raspberry Pi
    lgpio = None
    _LGPIO_IMPORT_ERROR = exc
else:
    _LGPIO_IMPORT_ERROR = None


class _RPiGPIOBackend:
    name = "RPi.GPIO"

    def __init__(self, dout_pin, pd_sck_pin):
        if GPIO is None:
            raise RuntimeError(f"RPi.GPIO-compatible module is required: {_GPIO_IMPORT_ERROR}")
        self.GPIO = GPIO
        self.dout_pin = int(dout_pin)
        self.pd_sck_pin = int(pd_sck_pin)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pd_sck_pin, GPIO.OUT, initial=GPIO.LOW)
        try:
            GPIO.setup(self.dout_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        except (AttributeError, TypeError):
            GPIO.setup(self.dout_pin, GPIO.IN)

    def output(self, pin, level):
        self.GPIO.output(int(pin), self.GPIO.HIGH if level else self.GPIO.LOW)

    def input(self, pin):
        return int(self.GPIO.input(int(pin)))

    def close(self):
        return


class _LGPIOBackend:
    name = "lgpio"

    def __init__(self, dout_pin, pd_sck_pin):
        if lgpio is None:
            raise RuntimeError(f"lgpio is required: {_LGPIO_IMPORT_ERROR}")
        self.dout_pin = int(dout_pin)
        self.pd_sck_pin = int(pd_sck_pin)
        chip_text = os.environ.get("PI_BUBBLE_GPIO_CHIP", "gpiochip0")
        chip_number = int(chip_text.replace("/dev/", "").replace("gpiochip", ""))
        self._handle = lgpio.gpiochip_open(chip_number)
        self._closed = False

        try:
            lgpio.gpio_claim_output(self._handle, self.pd_sck_pin, 0)
        except TypeError:
            lgpio.gpio_claim_output(self._handle, 0, self.pd_sck_pin, 0)
        try:
            lgpio.gpio_claim_input(self._handle, self.dout_pin)
        except TypeError:
            lgpio.gpio_claim_input(self._handle, 0, self.dout_pin)

    def output(self, pin, level):
        lgpio.gpio_write(self._handle, int(pin), 1 if level else 0)

    def input(self, pin):
        return int(lgpio.gpio_read(self._handle, int(pin)))

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            lgpio.gpiochip_close(self._handle)
        except Exception:
            pass


class HX711:
    """Small HX711 driver compatible with the legacy hx711v0_5_1 API."""

    _GAIN_PULSES = {
        ("A", 128): 1,
        ("B", 32): 2,
        ("A", 64): 3,
    }

    def __init__(self, dout_pin, pd_sck_pin, gain=128):
        self.dout_pin = int(dout_pin)
        self.pd_sck_pin = int(pd_sck_pin)
        self.gain = int(gain)
        self.byte_format = "MSB"
        self.bit_format = "MSB"
        self.offset = {"A": 0.0, "B": 0.0}
        self.reference_unit = {"A": 1.0, "B": 1.0}
        self.readyCallbackEnabled = False
        self._gpio = self._open_gpio_backend()
        self._configure_gain(self.gain)

    def _open_gpio_backend(self):
        errors = []
        for backend_cls in (_RPiGPIOBackend, _LGPIOBackend):
            try:
                backend = backend_cls(self.dout_pin, self.pd_sck_pin)
                self.backend_name = backend.name
                return backend
            except Exception as exc:
                errors.append(f"{backend_cls.name}: {exc}")
        raise RuntimeError("No usable GPIO backend for HX711. " + " | ".join(errors))

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
        self._gpio.output(self.pd_sck_pin, 0)
        time.sleep(0.001)
        self.powerDown()
        self.powerUp()

    def powerDown(self):
        self._gpio.output(self.pd_sck_pin, 0)
        self._gpio.output(self.pd_sck_pin, 1)
        time.sleep(0.00008)

    def powerUp(self):
        self._gpio.output(self.pd_sck_pin, 0)
        time.sleep(0.001)

    def isReady(self):
        return self._gpio.input(self.dout_pin) == 0

    def getLong(self, channel="A", timeout=1.5):
        channel = self._normalize_channel(channel)
        pulses = self._pulses_for_channel(channel)
        return self._read_raw(pulses, timeout=timeout)

    def getWeight(self, channel="A", timeout=1.5):
        channel = self._normalize_channel(channel)
        value = self.getLong(channel, timeout=timeout)
        return (float(value) - self.offset[channel]) / self.reference_unit[channel]

    def disableReadyCallback(self):
        self.readyCallbackEnabled = False

    def close(self):
        self._gpio.close()

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
        while self._gpio.input(self.dout_pin) != 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("HX711 is not ready.")
            time.sleep(0.001)

    def _read_raw(self, gain_pulses, timeout=1.5):
        self._wait_ready(timeout=timeout)
        value = 0

        for _ in range(24):
            self._gpio.output(self.pd_sck_pin, 1)
            value = (value << 1) | int(self._gpio.input(self.dout_pin))
            self._gpio.output(self.pd_sck_pin, 0)

        for _ in range(gain_pulses):
            self._gpio.output(self.pd_sck_pin, 1)
            self._gpio.output(self.pd_sck_pin, 0)

        if value & 0x800000:
            value -= 0x1000000
        return value
