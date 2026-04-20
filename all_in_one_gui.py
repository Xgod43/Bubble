import queue
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None


LIMIT_1_PIN = 17
LIMIT_2_PIN = 27

STEPPER_PUL_PIN = 24
STEPPER_DIR_PIN = 23
STEPPER_ENA_PIN = 26
STEPPER_FREQUENCY_HZ = 800
STEPPER_PULSE_DELAY = 1.0 / (2.0 * STEPPER_FREQUENCY_HZ)

LOADCELL_DT_PIN = 5
LOADCELL_SCK_PIN = 6

if GPIO is None:
    GPIO_HIGH = 1
    GPIO_LOW = 0
else:
    GPIO_HIGH = GPIO.HIGH
    GPIO_LOW = GPIO.LOW

DIR_UP_STATE = GPIO_HIGH
DIR_DOWN_STATE = GPIO_LOW
STEPPER_ENA_ACTIVE_STATE = GPIO_HIGH
STEPPER_ENA_INACTIVE_STATE = GPIO_LOW


class AllInOneTesterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("All-in-One Hardware Test GUI")
        self.root.geometry("980x760")

        self.log_queue = queue.Queue()

        self.camera = None
        self.picam2 = None
        self.camera_running = False

        self.limit_thread = None
        self.limit_stop_event = threading.Event()

        self.stepper_thread = None
        self.stepper_stop_event = threading.Event()

        self.pressure_thread = None
        self.pressure_stop_event = threading.Event()

        self.loadcell_thread = None
        self.loadcell_stop_event = threading.Event()
        self.loadcell_hx = None

        self._build_ui()
        self._setup_gpio_once()
        self.root.after(100, self._drain_log_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.log("GUI initialized.")

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        title = ttk.Label(
            main,
            text="All-in-One Test Panel",
            font=("Segoe UI", 18, "bold"),
        )
        title.pack(anchor="w")

        subtitle = ttk.Label(
            main,
            text="Camera, limit switches, stepper, pressure, and load cell in one screen.",
        )
        subtitle.pack(anchor="w", pady=(0, 10))

        notebook = ttk.Notebook(main)
        notebook.pack(fill="both", expand=True)

        self._build_camera_tab(notebook)
        self._build_limit_tab(notebook)
        self._build_stepper_tab(notebook)
        self._build_pressure_tab(notebook)
        self._build_loadcell_tab(notebook)

        log_frame = ttk.LabelFrame(main, text="Log")
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.log_text = ScrolledText(log_frame, height=14, state="disabled", wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def _build_camera_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="1) Camera Test")

        self.camera_status_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text="Open camera preview. Use Stop to close the preview.",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Button(tab, text="Open Camera", command=self.start_camera).grid(
            row=1, column=0, sticky="w"
        )
        ttk.Button(tab, text="Stop Camera", command=self.stop_camera).grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="Status:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.camera_status_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )

    def _build_limit_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="2) Limit Switch Test")

        self.limit1_status_var = tk.StringVar(value="not triggered")
        self.limit2_status_var = tk.StringVar(value="not triggered")
        self.limit_monitor_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text=(
                f"Monitoring GPIO{LIMIT_1_PIN} and GPIO{LIMIT_2_PIN}. "
                "When pressed, status changes to TRIGGERED."
            ),
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Button(tab, text="Start Monitor", command=self.start_limit_monitor).grid(
            row=1, column=0, sticky="w"
        )
        ttk.Button(tab, text="Stop Monitor", command=self.stop_limit_monitor).grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="Monitor:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.limit_monitor_state_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Limit 1:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.limit1_status_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Limit 2:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Label(tab, textvariable=self.limit2_status_var).grid(
            row=4, column=1, sticky="w", pady=(6, 0)
        )

    def _build_stepper_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="3) Stepper Test")

        self.stepper_direction_var = tk.StringVar(value="up")
        self.stepper_seconds_var = tk.StringVar(value="1.0")
        self.stepper_state_var = tk.StringVar(value="Idle")

        ttk.Label(
            tab,
            text=(
                "Command-style move: select up or down, enter seconds, then click Move. "
                "(Pins: PUL=GPIO24, DIR=GPIO23, ENA=GPIO26)"
            ),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(tab, text="Direction:").grid(row=1, column=0, sticky="w")
        direction_combo = ttk.Combobox(
            tab,
            textvariable=self.stepper_direction_var,
            values=("up", "down"),
            state="readonly",
            width=10,
        )
        direction_combo.grid(row=1, column=1, sticky="w")

        ttk.Label(tab, text="Seconds:").grid(row=1, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(tab, textvariable=self.stepper_seconds_var, width=10).grid(
            row=1, column=3, sticky="w"
        )

        ttk.Button(tab, text="Move", command=self.start_stepper_move).grid(
            row=2, column=0, sticky="w", pady=(10, 0)
        )
        ttk.Button(tab, text="Stop", command=self.stop_stepper_move).grid(
            row=2, column=1, sticky="w", pady=(10, 0), padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.stepper_state_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

    def _build_pressure_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="4) Pressure Test")

        self.pressure_value_var = tk.StringVar(value="-")
        self.pressure_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text="Reads pressure from MPRLS and displays value in hPa.",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Button(tab, text="Start Reading", command=self.start_pressure_read).grid(
            row=1, column=0, sticky="w"
        )
        ttk.Button(tab, text="Stop Reading", command=self.stop_pressure_read).grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.pressure_state_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Pressure:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.pressure_value_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

    def _build_loadcell_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="5) Load Cell Test")

        self.loadcell_known_weight_var = tk.StringVar(value="500")
        self.loadcell_weight_var = tk.StringVar(value="-")
        self.loadcell_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text=(
                "Calibrates with a known weight (grams), then shows measured load in kg. "
                "(Pins: DT=GPIO5, SCK=GPIO6)"
            ),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(tab, text="Known weight (g):").grid(row=1, column=0, sticky="w")
        ttk.Entry(tab, textvariable=self.loadcell_known_weight_var, width=12).grid(
            row=1, column=1, sticky="w"
        )

        ttk.Button(tab, text="Start", command=self.start_loadcell_read).grid(
            row=2, column=0, sticky="w", pady=(10, 0)
        )
        ttk.Button(tab, text="Stop", command=self.stop_loadcell_read).grid(
            row=2, column=1, sticky="w", pady=(10, 0), padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_state_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Weight:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_weight_var).grid(
            row=4, column=1, sticky="w", pady=(10, 0)
        )

    def _setup_gpio_once(self):
        if GPIO is None:
            self.log("RPi.GPIO is not available. GPIO-based tests cannot run on this machine.")
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

    def _require_gpio(self):
        if GPIO is not None:
            return True
        self.log("GPIO test requested, but RPi.GPIO is unavailable.")
        messagebox.showerror(
            "GPIO Unavailable",
            "RPi.GPIO is not available in this environment.",
        )
        return False

    def _set_var(self, variable, value):
        if self.root.winfo_exists():
            self.root.after(0, variable.set, value)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")

    def _drain_log_queue(self):
        if not self.root.winfo_exists():
            return

        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break

            self.log_text.configure(state="normal")
            self.log_text.insert("end", line + "\n")
            self.log_text.see("end")
            self.log_text.configure(state="disabled")

        self.root.after(100, self._drain_log_queue)

    @staticmethod
    def _wait_with_cancel(seconds, stop_event, slice_seconds=0.05):
        end_time = time.perf_counter() + seconds
        while time.perf_counter() < end_time:
            if stop_event.is_set():
                return False
            remaining = end_time - time.perf_counter()
            time.sleep(min(slice_seconds, max(0.0, remaining)))
        return True

    def start_camera(self):
        if self.camera_running:
            self.log("Camera preview is already running.")
            return

        try:
            from picamzero import Camera

            self.camera = Camera()
            self.camera.start_preview()
            self.camera_running = True
            self.camera_status_var.set("Preview running (picamzero)")
            self.log("Camera preview started (picamzero).")
            return
        except Exception as picamzero_exc:
            self.log(f"picamzero camera start failed: {picamzero_exc}")

        try:
            from picamera2 import Picamera2

            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration())
            self.picam2.start()
            self.camera_running = True
            self.camera_status_var.set("Camera running (picamera2)")
            self.log("Camera started (picamera2).")
        except Exception as picamera2_exc:
            self.camera_running = False
            self.camera_status_var.set("Failed to open")
            self.log(f"picamera2 start failed: {picamera2_exc}")
            messagebox.showerror(
                "Camera Error",
                "Could not open camera. Check picamzero/picamera2 installation and camera connection.",
            )

    def stop_camera(self):
        stopped = False

        if self.camera is not None:
            try:
                self.camera.stop_preview()
                stopped = True
            except Exception:
                pass
            finally:
                self.camera = None

        if self.picam2 is not None:
            try:
                self.picam2.stop()
                stopped = True
            except Exception:
                pass
            finally:
                self.picam2 = None

        self.camera_running = False
        self.camera_status_var.set("Stopped")
        if stopped:
            self.log("Camera stopped.")

    def start_limit_monitor(self):
        if not self._require_gpio():
            return
        if self.limit_thread is not None and self.limit_thread.is_alive():
            self.log("Limit monitor is already running.")
            return

        GPIO.setup(LIMIT_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LIMIT_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.limit_stop_event.clear()
        self.limit_thread = threading.Thread(target=self._limit_worker, daemon=True)
        self.limit_thread.start()
        self.limit_monitor_state_var.set("Running")
        self.log(
            f"Limit monitor started on GPIO{LIMIT_1_PIN} and GPIO{LIMIT_2_PIN}."
        )

    def _limit_worker(self):
        last_state = (None, None)
        try:
            while not self.limit_stop_event.is_set():
                l1_triggered = GPIO.input(LIMIT_1_PIN) == GPIO_LOW
                l2_triggered = GPIO.input(LIMIT_2_PIN) == GPIO_LOW
                current_state = (l1_triggered, l2_triggered)

                if current_state != last_state:
                    l1_text = "TRIGGERED" if l1_triggered else "not triggered"
                    l2_text = "TRIGGERED" if l2_triggered else "not triggered"
                    self._set_var(self.limit1_status_var, l1_text)
                    self._set_var(self.limit2_status_var, l2_text)

                    if l1_triggered:
                        self.log("Limit 1 triggered.")
                    if l2_triggered:
                        self.log("Limit 2 triggered.")
                    if not l1_triggered and not l2_triggered:
                        self.log("Both limits are not triggered.")

                    last_state = current_state

                time.sleep(0.05)
        except Exception as exc:
            self.log(f"Limit monitor error: {exc}")
        finally:
            self._set_var(self.limit_monitor_state_var, "Stopped")

    def stop_limit_monitor(self):
        self.limit_stop_event.set()
        if self.limit_thread is not None and self.limit_thread.is_alive():
            self.limit_thread.join(timeout=1.0)
        self.limit_thread = None
        self.limit_monitor_state_var.set("Stopped")
        self.log("Limit monitor stopped.")

    def start_stepper_move(self):
        if not self._require_gpio():
            return
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.log("Stepper is already moving.")
            return

        direction = self.stepper_direction_var.get().strip().lower()
        if direction not in {"up", "down"}:
            messagebox.showerror("Invalid Direction", "Direction must be up or down.")
            return

        try:
            seconds = float(self.stepper_seconds_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Seconds", "Seconds must be a number.")
            return

        if seconds <= 0:
            messagebox.showerror("Invalid Seconds", "Seconds must be greater than 0.")
            return

        GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO_LOW)
        GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
        GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)

        # Keep command naming consistent with existing stepper script wiring.
        direction_state = DIR_DOWN_STATE if direction == "up" else DIR_UP_STATE

        self.stepper_stop_event.clear()
        self.stepper_thread = threading.Thread(
            target=self._stepper_worker,
            args=(direction, direction_state, seconds),
            daemon=True,
        )
        self.stepper_thread.start()
        self.stepper_state_var.set(f"Running ({direction} for {seconds:.2f}s)")
        self.log(f"Stepper command: {direction} for {seconds:.2f} seconds.")

    def _stepper_worker(self, direction_name, direction_state, seconds):
        try:
            GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_ACTIVE_STATE)
            GPIO.output(STEPPER_DIR_PIN, direction_state)
            time.sleep(0.002)

            end_time = time.perf_counter() + seconds
            while time.perf_counter() < end_time and not self.stepper_stop_event.is_set():
                GPIO.output(STEPPER_PUL_PIN, GPIO_HIGH)
                time.sleep(STEPPER_PULSE_DELAY)
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                time.sleep(STEPPER_PULSE_DELAY)

            if self.stepper_stop_event.is_set():
                self.log("Stepper movement stopped by user.")
            else:
                self.log(f"Stepper movement complete: {direction_name}.")
        except Exception as exc:
            self.log(f"Stepper error: {exc}")
        finally:
            try:
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
            except Exception:
                pass
            self._set_var(self.stepper_state_var, "Idle")

    def stop_stepper_move(self):
        self.stepper_stop_event.set()
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.stepper_thread.join(timeout=1.0)
        self.stepper_thread = None
        self.stepper_state_var.set("Idle")

        if GPIO is not None:
            try:
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
            except Exception:
                pass

        self.log("Stepper stop requested.")

    def start_pressure_read(self):
        if self.pressure_thread is not None and self.pressure_thread.is_alive():
            self.log("Pressure read is already running.")
            return

        self.pressure_stop_event.clear()
        self.pressure_thread = threading.Thread(target=self._pressure_worker, daemon=True)
        self.pressure_thread.start()
        self.pressure_state_var.set("Running")
        self.log("Pressure read started.")

    def _pressure_worker(self):
        try:
            import board
            import adafruit_mprls

            i2c = board.I2C()
            sensor = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
            self.log("MPRLS pressure sensor initialized.")

            while not self.pressure_stop_event.is_set():
                pressure_hpa = float(sensor.pressure)
                self._set_var(self.pressure_value_var, f"{pressure_hpa:.2f} hPa")
                time.sleep(0.5)
        except Exception as exc:
            self.log(f"Pressure sensor error: {exc}")
            self._set_var(self.pressure_value_var, "Error")
        finally:
            self._set_var(self.pressure_state_var, "Stopped")

    def stop_pressure_read(self):
        self.pressure_stop_event.set()
        if self.pressure_thread is not None and self.pressure_thread.is_alive():
            self.pressure_thread.join(timeout=1.0)
        self.pressure_thread = None
        self.pressure_state_var.set("Stopped")
        self.log("Pressure read stopped.")

    def _load_hx711_class(self):
        hx_path = Path(__file__).resolve().parent / "library" / "hx711py"
        if str(hx_path) not in sys.path:
            sys.path.insert(0, str(hx_path))
        from hx711v0_5_1 import HX711

        return HX711

    @staticmethod
    def _read_average_raw(hx, samples=20, delay=0.05, channel="A"):
        readings = []
        for _ in range(samples):
            value = hx.getLong(channel)
            if value is not None:
                readings.append(float(value))
            time.sleep(delay)

        if not readings:
            raise RuntimeError("No valid load cell readings were captured.")

        return sum(readings) / len(readings)

    def start_loadcell_read(self):
        if not self._require_gpio():
            return
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.log("Load cell read is already running.")
            return

        try:
            known_weight_grams = float(self.loadcell_known_weight_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Weight", "Known weight must be a number.")
            return

        if known_weight_grams <= 0:
            messagebox.showerror("Invalid Weight", "Known weight must be > 0 grams.")
            return

        self.loadcell_stop_event.clear()
        self.loadcell_thread = threading.Thread(
            target=self._loadcell_worker,
            args=(known_weight_grams,),
            daemon=True,
        )
        self.loadcell_thread.start()
        self.loadcell_state_var.set("Running")
        self.log("Load cell read started.")

    def _loadcell_worker(self, known_weight_grams):
        hx = None
        try:
            HX711 = self._load_hx711_class()
            hx = HX711(LOADCELL_DT_PIN, LOADCELL_SCK_PIN)
            self.loadcell_hx = hx

            hx.setReadingFormat("MSB", "MSB")
            hx.reset()

            self.log("Load cell: remove all weight for tare.")
            if not self._wait_with_cancel(2.0, self.loadcell_stop_event):
                return

            offset = self._read_average_raw(hx, channel="A")
            hx.setOffset(offset, "A")
            self.log(f"Load cell tare offset: {offset:.2f}")

            self.log(f"Load cell: place {known_weight_grams:.2f} g for calibration.")
            if not self._wait_with_cancel(3.0, self.loadcell_stop_event):
                return

            loaded_raw = self._read_average_raw(hx, channel="A")
            delta = loaded_raw - offset
            if abs(delta) < 1e-9:
                raise RuntimeError("Calibration failed: raw delta is too small.")

            counts_per_gram = delta / known_weight_grams
            hx.setReferenceUnit(counts_per_gram, "A")
            self.log(f"Load cell calibrated. counts_per_gram={counts_per_gram:.6f}")

            while not self.loadcell_stop_event.is_set():
                weight_grams = hx.getWeight("A")
                if weight_grams is not None:
                    weight_kg = float(weight_grams) / 1000.0
                    self._set_var(self.loadcell_weight_var, f"{weight_kg:.4f} kg")
                time.sleep(0.2)
        except Exception as exc:
            self.log(f"Load cell error: {exc}")
            self._set_var(self.loadcell_weight_var, "Error")
        finally:
            if hx is not None and getattr(hx, "readyCallbackEnabled", False):
                try:
                    hx.disableReadyCallback()
                except RuntimeError:
                    pass

            self.loadcell_hx = None
            self._set_var(self.loadcell_state_var, "Stopped")

    def stop_loadcell_read(self):
        self.loadcell_stop_event.set()
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.loadcell_thread.join(timeout=1.0)
        self.loadcell_thread = None
        self.loadcell_state_var.set("Stopped")
        self.log("Load cell stop requested.")

    def on_close(self):
        self.stop_camera()
        self.stop_limit_monitor()
        self.stop_stepper_move()
        self.stop_pressure_read()
        self.stop_loadcell_read()

        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass

        self.root.destroy()


def main():
    root = tk.Tk()
    app = AllInOneTesterGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()