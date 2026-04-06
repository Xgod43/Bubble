import argparse
import logging
import math
import statistics
import sys
import threading
import time
from collections import deque
from pathlib import Path
import adafruit_mprls
import board
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
HX711_LIB_DIR = Path(__file__).resolve().parent / "library" / "hx711py"
if str(HX711_LIB_DIR) not in sys.path:
    sys.path.insert(0, str(HX711_LIB_DIR))
from hx711v0_5_1 import HX711
DT_PIN = 5
SCK_PIN = 6
DEFAULT_DURATION_SECONDS = 20
DEFAULT_SAMPLE_INTERVAL_SECONDS = 0.5
DEFAULT_PLOT_REFRESH_SECONDS = 0.1
def parse_args():
    parser = argparse.ArgumentParser(
        description="Combined Load Cell + Pressure graph (20s run with logging)."
    )
    parser.add_argument(
        "--interrupt",
        action="store_true",
        help="Required flag for stable HX711 readings.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SECONDS,
        help="Plot duration in seconds (default: 20).",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=DEFAULT_SAMPLE_INTERVAL_SECONDS,
        help="Seconds between plotted points (default: 0.5).",
    )
    parser.add_argument(
        "--plot-refresh",
        type=float,
        default=DEFAULT_PLOT_REFRESH_SECONDS,
        help="Seconds between plot redraws (default: 0.1).",
    )
    parser.add_argument(
        "--known-weight",
        type=float,
        default=None,
        help="Known calibration weight in grams. If omitted, you will be prompted.",
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Optional custom log file path.",
    )
    parser.add_argument(
        "--png-file",
        type=str,
        default=None,
        help="Optional output PNG path. Default uses log-file name with .png extension.",
    )
    return parser.parse_args()
def setup_logger(log_file_path):
    logger = logging.getLogger("combined_plot")
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    file_handler = logging.FileHandler(log_file_path)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(
        logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")
    )
    logger.addHandler(file_handler)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(console_handler)
    return logger
def read_average_raw(hx, logger, samples=20, delay=0.05, channel="A"):
    values = []
    for _ in range(samples):
        raw = hx.getLong(channel)
        if raw is not None:
            values.append(float(raw))
        time.sleep(delay)
    if not values:
        raise RuntimeError("No HX711 readings captured during calibration.")
    avg = sum(values) / len(values)
    logger.debug(
        "Raw average computed: %.6f from %d samples (requested=%d)",
        avg,
        len(values),
        samples,
    )
    return avg
def tare_and_calibrate(hx, logger, known_weight_grams=None, channel="A"):
    logger.info("Remove all weight from the scale...")
    time.sleep(2)
    offset = read_average_raw(hx, logger, channel=channel)
    hx.setOffset(offset, channel)
    logger.info("Tare offset: %.2f", offset)
    if known_weight_grams is None:
        known_weight_text = input(
            "Enter known calibration weight in grams (e.g. 500): "
        ).strip()
        if not known_weight_text:
            raise RuntimeError("Calibration weight is required.")
        known_weight_grams = float(known_weight_text)
    if known_weight_grams <= 0:
        raise RuntimeError("Calibration weight must be > 0.")
    logger.info("Place %.2f g on the scale...", known_weight_grams)
    time.sleep(3)
    loaded_raw = read_average_raw(hx, logger, channel=channel)
    delta = loaded_raw - offset
    if abs(delta) < 1e-9:
        raise RuntimeError("Calibration failed: raw delta is too small.")
    counts_per_gram = delta / known_weight_grams
    hx.setReferenceUnit(counts_per_gram, channel)
    logger.info("Calibration complete. counts_per_gram = %.6f", counts_per_gram)
    logger.debug(
        "Calibration details: loaded_raw=%.6f, offset=%.6f, delta=%.6f, known_weight=%.6f",
        loaded_raw,
        offset,
        delta,
        known_weight_grams,
    )
def set_axis_limits(ax, values, fallback_min=0.0, fallback_max=1.0):
    valid = [v for v in values if v is not None and math.isfinite(v)]
    if not valid:
        ax.set_ylim(fallback_min, fallback_max)
        return
    vmin = min(valid)
    vmax = max(valid)
    if abs(vmax - vmin) < 1e-9:
        pad = max(1.0, abs(vmin) * 0.1)
    else:
        pad = (vmax - vmin) * 0.1
    ax.set_ylim(vmin - pad, vmax + pad)
def summarize(values):
    valid = [v for v in values if v is not None and math.isfinite(v)]
    if not valid:
        return None
    return {
        "count": len(valid),
        "min": min(valid),
        "max": max(valid),
        "mean": statistics.fmean(valid),
    }
def main():
    args = parse_args()

    if not args.interrupt:
        raise RuntimeError(
            "This script requires --interrupt for stable loadcell readings."
        )
    if args.duration <= 0:
        raise RuntimeError("--duration must be greater than 0.")
    if args.sample_interval <= 0:
        raise RuntimeError("--sample-interval must be greater than 0.")
    if args.plot_refresh <= 0:
        raise RuntimeError("--plot-refresh must be greater than 0.")
    logs_dir = Path(__file__).resolve().parent / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    if args.log_file:
        log_file_path = Path(args.log_file).expanduser().resolve()
        log_file_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_file_path = logs_dir / f"combined_plot_{timestamp}.log"
    if args.png_file:
        png_file_path = Path(args.png_file).expanduser().resolve()
        png_file_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        png_file_path = log_file_path.with_suffix(".png")
    logger = setup_logger(log_file_path)
    logger.info("Starting combined plot script")
    logger.info("Log file: %s", log_file_path)
    logger.info("PNG file: %s", png_file_path)
    logger.info(
        "Run config: interrupt=%s duration=%.2f sample_interval=%.3f plot_refresh=%.3f",
        args.interrupt,
        args.duration,
        args.sample_interval,
        args.plot_refresh,
    )
    hx = None
    fig = None
    try:
        i2c = board.I2C()
        mprls = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
        logger.info("Pressure sensor initialized")
        hx = HX711(DT_PIN, SCK_PIN)
        hx.setReadingFormat("MSB", "MSB")
        hx.reset()
        logger.info("HX711 initialized and reset")
        tare_and_calibrate(hx, logger, known_weight_grams=args.known_weight, channel="A")
        lock = threading.Lock()
        latest_weight_grams = {"value": None}
        callback_count = {"value": 0}
        def on_hx_ready(raw_bytes):
            if raw_bytes is None:
                return
            weight_grams = hx.rawBytesToWeight(raw_bytes, "A")
            with lock:
                latest_weight_grams["value"] = float(weight_grams)
                callback_count["value"] += 1
        hx.enableReadyCallback(on_hx_ready)
        logger.info("HX711 interrupt callback enabled")
        max_points = int(args.duration / args.sample_interval) + 20
        x_data = deque(maxlen=max_points)
        weight_data = deque(maxlen=max_points)
        pressure_data = deque(maxlen=max_points)
        start_time = time.time()
        last_logged_second = {"value": -1}
        sample_timestamps = []
        fig, ax_weight = plt.subplots(figsize=(11, 6))
        ax_pressure = ax_weight.twinx()
        line_weight, = ax_weight.plot(
            [], [], color="tab:blue", linewidth=2, label="Weight (g)"
        )
        line_pressure, = ax_pressure.plot(
            [], [], color="tab:red", linewidth=2, label="Pressure (hPa)"
        )
        ax_weight.set_xlabel("Time (s)")
        ax_weight.set_ylabel("Weight (g)", color="tab:blue")
        ax_pressure.set_ylabel("Pressure (hPa)", color="tab:red")
        ax_weight.set_title("Load Cell and Pressure Live Graph")
        ax_weight.grid(True, alpha=0.3)
        ax_weight.set_xlim(0, args.duration)
        lines = [line_weight, line_pressure]
        labels = [line.get_label() for line in lines]
        ax_weight.legend(lines, labels, loc="upper left")
        logger.info("Displaying live plot window")
        plt.ion()
        plt.tight_layout()
        plt.show(block=False)
        next_sample_time = time.perf_counter()
        while True:
            elapsed = time.time() - start_time
            now_perf = time.perf_counter()
            if now_perf >= next_sample_time:
                pressure_hpa = float(mprls.pressure)
                with lock:
                    weight_grams = latest_weight_grams["value"]
                x_data.append(elapsed)
                pressure_data.append(pressure_hpa)
                if weight_grams is None:
                    weight_data.append(float("nan"))
                else:
                    weight_data.append(weight_grams)
                sample_timestamps.append(now_perf)
                next_sample_time += args.sample_interval
                if now_perf > next_sample_time + args.sample_interval:
                    next_sample_time = now_perf + args.sample_interval
                second_mark = int(elapsed)
                if second_mark != last_logged_second["value"]:
                    last_logged_second["value"] = second_mark
                    logger.info(
                        "t=%02ds | weight_g=%s | pressure_hpa=%.3f",
                        second_mark,
                        "None" if weight_grams is None else f"{weight_grams:.3f}",
                        pressure_hpa,
                    )
            line_weight.set_data(x_data, weight_data)
            line_pressure.set_data(x_data, pressure_data)
            ax_weight.set_xlim(0, args.duration)
            set_axis_limits(ax_weight, weight_data, fallback_min=-10.0, fallback_max=10.0)
            set_axis_limits(ax_pressure, pressure_data, fallback_min=0.0, fallback_max=1100.0)
            fig.canvas.draw_idle()
            plt.pause(0.001)
            if elapsed >= args.duration:
                logger.info("Duration reached (%.2fs). Closing plot.", elapsed)
                break
            sleep_time = max(0.001, min(args.plot_refresh, next_sample_time - time.perf_counter()))
            time.sleep(sleep_time)
        plt.ioff()
        fig.savefig(png_file_path, dpi=150, bbox_inches="tight")
        logger.info("Saved plot PNG: %s", png_file_path)
        plt.close(fig)
        weight_summary = summarize(weight_data)
        pressure_summary = summarize(pressure_data)
        logger.info("Run finished")
        logger.info("HX711 callback count: %d", callback_count["value"])
        if len(sample_timestamps) >= 2:
            sample_intervals = [
                b - a for a, b in zip(sample_timestamps[:-1], sample_timestamps[1:])
            ]
            logger.info(
                "Actual sample interval | n=%d min=%.3fs max=%.3fs mean=%.3fs",
                len(sample_intervals),
                min(sample_intervals),
                max(sample_intervals),
                statistics.fmean(sample_intervals),
            )
        else:
            logger.info("Actual sample interval | insufficient samples")
        if weight_summary:
            logger.info(
                "Weight summary | n=%d min=%.3f max=%.3f mean=%.3f",
                weight_summary["count"],
                weight_summary["min"],
                weight_summary["max"],
                weight_summary["mean"],
            )
        else:
            logger.info("Weight summary | no valid data")
        if pressure_summary:
            logger.info(
                "Pressure summary | n=%d min=%.3f max=%.3f mean=%.3f",
                pressure_summary["count"],
                pressure_summary["min"],
                pressure_summary["max"],
                pressure_summary["mean"],
            )
        else:
            logger.info("Pressure summary | no valid data")
    except (KeyboardInterrupt, SystemExit):
        logger.info("Interrupted by user")
    finally:
        if hx is not None and getattr(hx, "readyCallbackEnabled", False):
            try:
                hx.disableReadyCallback()
                logger.info("HX711 callback disabled")
            except RuntimeError as exc:
                logger.warning("HX711 callback disable warning: %s", exc)
        if fig is not None:
            try:
                plt.close(fig)
            except Exception:
                pass
        GPIO.cleanup()
        logger.info("GPIO cleanup complete")
if __name__ == "__main__":
    main()
