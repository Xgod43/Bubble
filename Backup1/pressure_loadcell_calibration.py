import csv
import json
import os
import statistics
import sys
import time
from pathlib import Path

import RPi.GPIO as GPIO


def _import_pressure_libs():
    try:
        import board  # type: ignore
        import adafruit_mprls  # type: ignore
        return board, adafruit_mprls
    except Exception:
        pass

    # Common case: installed with --user under normal user, then script run with sudo.
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        candidate = Path(f"/home/{sudo_user}/.local/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages")
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))
            try:
                import board  # type: ignore
                import adafruit_mprls  # type: ignore
                return board, adafruit_mprls
            except Exception:
                pass

    raise RuntimeError(
        "Failed to import pressure sensor libraries (board, adafruit_mprls).\n"
        "If you installed with --user, do not run plain 'sudo python3'.\n"
        "Try one of these:\n"
        "1) python3 pressure_loadcell_calibration.py\n"
        "2) sudo -E /home/bubble/Desktop/AerkKungCNX/Pi/.venv/bin/python pressure_loadcell_calibration.py\n"
        "3) sudo pip3 install adafruit-circuitpython-mprls --break-system-packages"
    )


board, adafruit_mprls = _import_pressure_libs()

HX711_LIB_DIR = Path(__file__).resolve().parent / "library" / "hx711py"
if str(HX711_LIB_DIR) not in sys.path:
    sys.path.insert(0, str(HX711_LIB_DIR))

from hx711v0_5_1 import HX711

DT_PIN = 5
SCK_PIN = 6
G = 9.80665
CHANNEL = "A"


def read_average_raw(hx, samples=20, delay=0.05, channel=CHANNEL):
    values = []
    for _ in range(samples):
        value = hx.getLong(channel)
        if value is not None:
            values.append(float(value))
        time.sleep(delay)

    if not values:
        raise RuntimeError("No valid HX711 raw samples captured.")

    return sum(values) / len(values)


def read_average_weight_grams(hx, samples=12, delay=0.06, channel=CHANNEL):
    values = []
    for _ in range(samples):
        value = hx.getWeight(channel)
        if value is not None:
            values.append(float(value))
        time.sleep(delay)

    if not values:
        raise RuntimeError("No valid HX711 weight samples captured.")

    return sum(values) / len(values)


def read_average_pressure_hpa(sensor, samples=12, delay=0.06):
    values = []
    for _ in range(samples):
        values.append(float(sensor.pressure))
        time.sleep(delay)
    return sum(values) / len(values)


def tare_loadcell(hx, samples=25, channel=CHANNEL):
    print("\n[Step] Remove all load from load cell for tare...")
    time.sleep(2.0)
    offset = read_average_raw(hx, samples=samples, channel=channel)
    hx.setOffset(offset, channel)
    print(f"Tare offset: {offset:.2f}")
    return offset


def calibrate_loadcell(hx, tare_offset, known_weight_grams, samples=25, channel=CHANNEL):
    print(f"\n[Step] Place known calibration mass: {known_weight_grams:.2f} g")
    time.sleep(3.0)
    loaded_raw = read_average_raw(hx, samples=samples, channel=channel)
    delta = loaded_raw - tare_offset
    if abs(delta) < 1e-9:
        raise RuntimeError("Calibration failed: load cell delta too small.")
    counts_per_gram = delta / known_weight_grams
    hx.setReferenceUnit(counts_per_gram, channel)
    print(f"Load cell calibrated. counts_per_gram={counts_per_gram:.6f}")
    return counts_per_gram


def linear_regression(xs, ys):
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("Need at least 2 paired points for regression.")

    n = len(xs)
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n

    sxx = sum((x - x_mean) ** 2 for x in xs)
    sxy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))

    if abs(sxx) < 1e-12:
        raise RuntimeError("Regression failed: pressure values are too similar.")

    slope = sxy / sxx
    intercept = y_mean - slope * x_mean

    ss_tot = sum((y - y_mean) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 1e-12 else 1.0

    return slope, intercept, r2


def acquire_calibration_point(hx, sensor, capture_seconds=3.0, sample_delay=0.08):
    pressures = []
    weights_g = []
    forces_n = []

    end_time = time.time() + capture_seconds
    while time.time() < end_time:
        p_hpa = float(sensor.pressure)
        w_g = float(hx.getWeight(CHANNEL))
        f_n = (w_g / 1000.0) * G

        pressures.append(p_hpa)
        weights_g.append(w_g)
        forces_n.append(f_n)
        time.sleep(sample_delay)

    if not pressures:
        raise RuntimeError("No samples captured during calibration point acquisition.")

    return {
        "pressure_hpa": sum(pressures) / len(pressures),
        "weight_g": sum(weights_g) / len(weights_g),
        "force_n": sum(forces_n) / len(forces_n),
        "pressure_std": statistics.pstdev(pressures) if len(pressures) > 1 else 0.0,
        "weight_std": statistics.pstdev(weights_g) if len(weights_g) > 1 else 0.0,
        "samples": len(pressures),
    }


def save_outputs(points, model):
    out_dir = Path(__file__).resolve().parent / "logs" / "pressure_force_calibration"
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"calibration_points_{ts}.csv"
    json_path = out_dir / f"pressure_to_force_model_{ts}.json"

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "point",
            "pressure_hpa",
            "pressure_std_hpa",
            "weight_g",
            "weight_std_g",
            "force_n",
            "samples",
        ])
        for idx, p in enumerate(points, start=1):
            writer.writerow([
                idx,
                f"{p['pressure_hpa']:.6f}",
                f"{p['pressure_std']:.6f}",
                f"{p['weight_g']:.6f}",
                f"{p['weight_std']:.6f}",
                f"{p['force_n']:.6f}",
                p["samples"],
            ])

    json_path.write_text(json.dumps(model, indent=2), encoding="utf-8")

    print(f"\nSaved points CSV: {csv_path}")
    print(f"Saved model JSON: {json_path}")


def run_live_validation(sensor, model):
    slope = model["force_model"]["slope_n_per_hpa"]
    intercept = model["force_model"]["intercept_n"]
    unit = model["distance_unit"]

    print("\n[Live] pressure -> estimated force. Ctrl+C to stop.")
    while True:
        p_hpa = float(sensor.pressure)
        f_est = slope * p_hpa + intercept
        print(f"Pressure: {p_hpa:8.3f} hPa | Est Force: {f_est:8.4f} N ({f_est / G * 1000.0:8.2f} g)")
        time.sleep(0.25)


def main():
    hx = None
    try:
        print("=== Pressure to Force Calibration (MPRLS + HX711) ===")

        hx = HX711(DT_PIN, SCK_PIN)
        hx.setReadingFormat("MSB", "MSB")
        hx.reset()

        i2c = board.I2C()
        mprls = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)

        tare_offset = tare_loadcell(hx)

        known_weight_str = input("Enter known calibration mass for load cell (grams): ").strip()
        known_weight_grams = float(known_weight_str)
        if known_weight_grams <= 0:
            raise RuntimeError("Known calibration mass must be > 0.")

        calibrate_loadcell(hx, tare_offset, known_weight_grams)

        baseline_pressure = read_average_pressure_hpa(mprls, samples=20, delay=0.05)
        baseline_weight_g = read_average_weight_grams(hx, samples=20, delay=0.05)
        baseline_force_n = (baseline_weight_g / 1000.0) * G
        print(
            f"\nBaseline pressure={baseline_pressure:.4f} hPa, "
            f"baseline load={baseline_weight_g:.4f} g ({baseline_force_n:.6f} N)"
        )

        num_points = int(input("How many calibration points? (>=2): ").strip())
        if num_points < 2:
            raise RuntimeError("Need at least 2 points.")

        points = []
        for i in range(num_points):
            print(f"\nPoint {i + 1}/{num_points}")
            input("Apply target pressure/load and press Enter to capture 3 seconds of data...")
            point = acquire_calibration_point(hx, mprls, capture_seconds=3.0, sample_delay=0.08)
            points.append(point)
            print(
                f"Captured: P={point['pressure_hpa']:.4f} hPa, "
                f"F={point['force_n']:.6f} N, W={point['weight_g']:.3f} g"
            )

        x_pressure = [p["pressure_hpa"] for p in points]
        y_force = [p["force_n"] for p in points]

        slope, intercept, r2 = linear_regression(x_pressure, y_force)
        # F = slope * P_hPa + intercept, and F = (P_hPa * 100) * A + intercept
        # Therefore A ~= slope / 100
        effective_area_m2 = slope / 100.0

        print("\n=== Calibration Result ===")
        print(f"Force model: F(N) = {slope:.8f} * P(hPa) + {intercept:.8f}")
        print(f"R^2: {r2:.6f}")
        print(f"Estimated effective area: {effective_area_m2:.8e} m^2")

        model = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sensor": "MPRLS + HX711",
            "distance_unit": "N",
            "baseline": {
                "pressure_hpa": baseline_pressure,
                "weight_g": baseline_weight_g,
                "force_n": baseline_force_n,
            },
            "force_model": {
                "slope_n_per_hpa": slope,
                "intercept_n": intercept,
                "r2": r2,
                "effective_area_m2": effective_area_m2,
            },
            "notes": "Use this model to estimate force from pressure readings.",
        }

        save_outputs(points, model)

        live = input("\nRun live pressure->force estimate now? (y/N): ").strip().lower()
        if live == "y":
            run_live_validation(mprls, model)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if hx is not None and getattr(hx, "readyCallbackEnabled", False):
            try:
                hx.disableReadyCallback()
            except Exception:
                pass
        GPIO.cleanup()


if __name__ == "__main__":
    main()
