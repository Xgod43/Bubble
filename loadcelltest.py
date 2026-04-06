import time
import sys
from pathlib import Path
import RPi.GPIO as GPIO

# Ensure local hx711py folder is importable when running from project root.
HX711_LIB_DIR = Path(__file__).resolve().parent / "library" / "hx711py"
if str(HX711_LIB_DIR) not in sys.path:
    sys.path.insert(0, str(HX711_LIB_DIR))

from hx711v0_5_1 import HX711

DT_PIN = 5
SCK_PIN = 6
READ_MODE_POLLING = "polling"
READ_MODE_INTERRUPT = "interrupt"


def read_average_raw(hx, samples=10, delay=0.05, channel="A"):
    readings = []
    for _ in range(samples):
        value = hx.getLong(channel)
        if value is None:
            continue
        readings.append(float(value))
        time.sleep(delay)

    if not readings:
        raise RuntimeError("No valid readings were captured from HX711.")

    return sum(readings) / len(readings)


def tare(hx, samples=20, channel="A"):
    print("Remove all weight from the scale...")
    time.sleep(2)
    offset = read_average_raw(hx, samples=samples, channel=channel)
    hx.setOffset(offset, channel)
    print(f"Tare offset: {offset:.2f}")
    return offset


def calibrate_scale(hx, offset, known_weight_grams, samples=20, channel="A"):
    print(f"Place {known_weight_grams:.2f} g on the scale...")
    time.sleep(3)
    loaded_raw = read_average_raw(hx, samples=samples, channel=channel)
    delta = loaded_raw - offset
    if abs(delta) < 1e-9:
        raise RuntimeError("Calibration failed: raw delta is too small.")
    counts_per_gram = delta / known_weight_grams
    hx.setReferenceUnit(counts_per_gram, channel)
    print(f"Calibration complete. counts_per_gram = {counts_per_gram:.6f}")
    return counts_per_gram


def parse_read_mode(argv):
    if "--interrupt" in argv:
        return READ_MODE_INTERRUPT
    return READ_MODE_POLLING


def print_live_weight_polling(hx, channel="A"):
    weight_grams = hx.getWeight(channel)
    if weight_grams is None:
        return
    weight_kilograms = weight_grams / 1000.0 * 2
    print(f"Weight: {weight_grams:.2f} g ({weight_kilograms:.4f} kg)")


def start_interrupt_output(hx, channel="A"):
    def callback(raw_bytes):
        if raw_bytes is None:
            return
        weight_grams = hx.rawBytesToWeight(raw_bytes, channel)
        weight_kilograms = weight_grams / 1000.0
        print(f"Weight: {weight_grams:.2f} g ({weight_kilograms:.4f} kg)")

    hx.enableReadyCallback(callback)


def main():
    hx = None
    try:
        read_mode = parse_read_mode(sys.argv[1:])

        hx = HX711(DT_PIN, SCK_PIN)
        hx.setReadingFormat("MSB", "MSB")
        hx.reset()

        print(f"Read mode: {read_mode}. Use --interrupt for interrupt-based output.")

        offset = tare(hx, channel="A")

        known_weight = input("Enter known calibration weight in grams (e.g. 500): ").strip()
        if not known_weight:
            raise RuntimeError("Calibration weight is required.")

        known_weight_grams = float(known_weight)
        if known_weight_grams <= 0:
            raise RuntimeError("Calibration weight must be > 0.")

        calibrate_scale(hx, offset, known_weight_grams, channel="A")

        print("Starting weight readings. Press Ctrl+C to exit.")

        if read_mode == READ_MODE_INTERRUPT:
            start_interrupt_output(hx, channel="A")

        while True:
            if read_mode == READ_MODE_POLLING:
                print_live_weight_polling(hx, channel="A")
                time.sleep(0.2)
            else:
                time.sleep(0.2)

    except (KeyboardInterrupt, SystemExit):
        print("\nExiting test.")
    finally:
        if hx is not None and getattr(hx, "readyCallbackEnabled", False):
            hx.disableReadyCallback()
        GPIO.cleanup()
        sys.exit()


if __name__ == "__main__":
    main()