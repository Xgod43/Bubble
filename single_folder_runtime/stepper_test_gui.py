from __future__ import annotations

import os
import subprocess
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


STEPPER_PUL_PIN = 24
STEPPER_DIR_PIN = 23
STEPPER_ENA_PIN = 26
STEPPER_PULSES_PER_REV = 1600
STEPPER_FREQUENCY_HZ = float(os.environ.get("BUBBLE_STEPPER_FREQUENCY_HZ", STEPPER_PULSES_PER_REV))
STEPPER_PULSE_DELAY = 1.0 / (2.0 * STEPPER_FREQUENCY_HZ)
STEPPER_MM_PER_REV = float(os.environ.get("BUBBLE_STEPPER_MM_PER_REV", "8.0"))
STEPPER_MM_PER_PULSE = STEPPER_MM_PER_REV / STEPPER_PULSES_PER_REV

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

COLOR_BG = "#071018"
COLOR_SURFACE = "#0c1722"
COLOR_PANEL = "#122232"
COLOR_BORDER = "#22384d"
COLOR_TEXT = "#eef5fb"
COLOR_MUTED = "#93a9bc"
COLOR_OK = "#5bd69a"
COLOR_WARN = "#e4a84f"


class StepperTestGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Stepper Test")
        self.root.geometry("620x520")
        self.root.minsize(540, 440)
        self.root.configure(bg=COLOR_BG)

        self.stepper_thread = None
        self.stop_event = threading.Event()
        self.position_mm = 0.0
        self.native_runner_build_message = ""
        self.native_runner = self._ensure_native_stepper_runner()

        self.direction_var = tk.StringVar(value="down")
        self.seconds_var = tk.StringVar(value="1.0")
        self.frequency_var = tk.StringVar(value=f"{STEPPER_FREQUENCY_HZ:.0f}")
        self.state_var = tk.StringVar(value="Ready")
        self.position_var = tk.StringVar(value="0.000 mm")
        self.pulse_var = tk.StringVar(value="0 pulses")
        self.mode_var = tk.StringVar(value=self._mode_text())
        self.simulation_var = tk.BooleanVar(value=GPIO is None and self.native_runner is None)

        self._configure_styles()
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.log("Stepper test ready.")
        if self.native_runner_build_message:
            self.log(self.native_runner_build_message)

    def _configure_styles(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=COLOR_BG, foreground=COLOR_TEXT)
        style.configure("TFrame", background=COLOR_BG)
        style.configure("Surface.TFrame", background=COLOR_SURFACE)
        style.configure("Panel.TFrame", background=COLOR_PANEL)
        style.configure("TLabel", background=COLOR_BG, foreground=COLOR_TEXT)
        style.configure("Muted.TLabel", background=COLOR_PANEL, foreground=COLOR_MUTED, font=("Segoe UI", 9))
        style.configure("Value.TLabel", background=COLOR_PANEL, foreground=COLOR_TEXT, font=("Segoe UI", 12, "bold"))
        style.configure("TButton", padding=(10, 7), font=("Segoe UI", 10, "bold"))
        style.configure("Accent.TButton", background="#0b6e69", foreground="#ffffff")
        style.map("Accent.TButton", background=[("active", "#0a5f5a")])
        style.configure("Danger.TButton", background="#a34b2a", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#8d3f23")])
        style.configure("TLabelframe", background=COLOR_SURFACE, bordercolor=COLOR_BORDER, relief="solid")
        style.configure("TLabelframe.Label", background=COLOR_SURFACE, foreground=COLOR_TEXT, font=("Segoe UI", 10, "bold"))

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        header = ttk.Frame(self.root, padding=12, style="Surface.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text="Stepper Test",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT,
            font=("Segoe UI", 18, "bold"),
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            text=f"PUL GPIO{STEPPER_PUL_PIN} | DIR GPIO{STEPPER_DIR_PIN} | ENA GPIO{STEPPER_ENA_PIN}",
            background=COLOR_SURFACE,
            foreground=COLOR_MUTED,
            font=("Segoe UI", 9),
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))

        controls = ttk.LabelFrame(self.root, text="Move Command", padding=10)
        controls.grid(row=1, column=0, sticky="ew", padx=12, pady=12)
        for col in range(4):
            controls.columnconfigure(col, weight=1)

        ttk.Label(controls, text="Direction").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            controls,
            textvariable=self.direction_var,
            values=("up", "down"),
            state="readonly",
            width=10,
        ).grid(row=1, column=0, sticky="ew", padx=(0, 8))

        ttk.Label(controls, text="Seconds").grid(row=0, column=1, sticky="w")
        ttk.Entry(controls, textvariable=self.seconds_var, width=10).grid(row=1, column=1, sticky="ew", padx=(0, 8))

        ttk.Label(controls, text="Frequency Hz").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.frequency_var, width=10).grid(row=1, column=2, sticky="ew", padx=(0, 8))

        ttk.Checkbutton(
            controls,
            text="Simulation",
            variable=self.simulation_var,
        ).grid(row=1, column=3, sticky="w")

        self.move_btn = ttk.Button(controls, text="Move", style="Accent.TButton", command=self.start_move)
        self.move_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0), padx=(0, 8))
        self.stop_btn = ttk.Button(controls, text="Stop", style="Danger.TButton", command=self.stop_move)
        self.stop_btn.grid(row=2, column=2, columnspan=2, sticky="ew", pady=(10, 0))

        readout = ttk.LabelFrame(self.root, text="Readout", padding=10)
        readout.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 12))
        readout.columnconfigure(1, weight=1)
        readout.rowconfigure(5, weight=1)
        self._readout_row(readout, 0, "State", self.state_var)
        self._readout_row(readout, 1, "Position", self.position_var)
        self._readout_row(readout, 2, "Last pulse count", self.pulse_var)
        self._readout_row(readout, 3, "Mode", self.mode_var)

        reset_btn = ttk.Button(readout, text="Reset Position", command=self.reset_position)
        reset_btn.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(10, 8))

        self.log_text = ScrolledText(readout, height=8, state="disabled", wrap="word")
        self.log_text.configure(
            bg="#0a131b",
            fg=COLOR_TEXT,
            insertbackground=COLOR_TEXT,
            relief="flat",
            font=("Consolas", 10),
        )
        self.log_text.grid(row=5, column=0, columnspan=2, sticky="nsew")
        self._set_running(False)

    def _readout_row(self, parent, row, label, variable):
        ttk.Label(parent, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=(0 if row == 0 else 6, 0))
        ttk.Label(parent, textvariable=variable, style="Value.TLabel").grid(row=row, column=1, sticky="w", padx=(10, 0), pady=(0 if row == 0 else 6, 0))

    def _mode_text(self):
        if self.native_runner is not None:
            return f"native runner: {self.native_runner}"
        if GPIO is not None:
            return "RPi.GPIO"
        return "simulation only"

    def _real_motion_available(self):
        return self.native_runner is not None or GPIO is not None

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_running(self, running):
        self.move_btn.configure(state="disabled" if running else "normal")
        self.stop_btn.configure(state="normal" if running else "disabled")

    def _set_state(self, text):
        self.root.after(0, lambda: self.state_var.set(text))

    def _set_readouts(self, pulses=None):
        if pulses is not None:
            self.pulse_var.set(f"{int(pulses)} pulses")
        self.position_var.set(f"{self.position_mm:.3f} mm")
        if self.simulation_var.get():
            self.mode_var.set("simulation only")
        else:
            self.mode_var.set(self._mode_text())

    def reset_position(self):
        self.position_mm = 0.0
        self._set_readouts(pulses=0)
        self.log("Position estimate reset.")

    def _resolve_native_stepper_runner(self):
        env_override = os.environ.get("PI_BUBBLE_STEPPER_RUNNER")
        if env_override:
            candidate = Path(env_override)
            if candidate.exists():
                return str(candidate)

        runner_name = "stepper_runner.exe" if os.name == "nt" else "stepper_runner"
        candidate = Path(__file__).resolve().parent / "native" / "build" / runner_name
        if candidate.exists():
            return str(candidate)
        return None

    def _ensure_native_stepper_runner(self):
        runner = self._resolve_native_stepper_runner()
        if runner is not None:
            return runner

        script = Path(__file__).resolve().parent / "native" / "build_pi.sh"
        if not script.exists():
            self.native_runner_build_message = "Native stepper runner not found; build script is missing."
            return None

        try:
            result = subprocess.run(
                ["bash", str(script)],
                cwd=str(script.parent),
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except Exception as exc:
            self.native_runner_build_message = f"Native runner auto-build failed: {exc}"
            return None

        runner = self._resolve_native_stepper_runner()
        if runner is not None:
            self.native_runner_build_message = "Native stepper runner built automatically."
            return runner

        output = (result.stderr or result.stdout or "").strip()
        if result.returncode == 0:
            self.native_runner_build_message = "Native runner was not produced; using RPi.GPIO or simulation."
        else:
            self.native_runner_build_message = f"Native runner build failed: {output[:240]}"
        return None

    def _validate_command(self):
        direction = self.direction_var.get().strip().lower()
        if direction not in {"up", "down"}:
            messagebox.showerror("Invalid Direction", "Direction must be up or down.")
            return None
        try:
            seconds = float(self.seconds_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Seconds", "Seconds must be a number.")
            return None
        try:
            frequency = float(self.frequency_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Frequency", "Frequency must be a number.")
            return None
        if seconds <= 0.0:
            messagebox.showerror("Invalid Seconds", "Seconds must be greater than 0.")
            return None
        if frequency <= 0.0:
            messagebox.showerror("Invalid Frequency", "Frequency must be greater than 0.")
            return None
        return direction, seconds, frequency

    def start_move(self):
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.log("Stepper is already moving.")
            return
        command = self._validate_command()
        if command is None:
            return

        direction, seconds, frequency = command
        if not self.simulation_var.get() and not self._real_motion_available():
            messagebox.showerror(
                "GPIO Unavailable",
                "No native runner or RPi.GPIO is available. Enable Simulation or build native/stepper_runner.",
            )
            return

        self.stop_event.clear()
        self._set_running(True)
        self.state_var.set(f"Running ({direction}, {seconds:.2f}s)")
        self.log(f"Move command: {direction}, {seconds:.2f}s, {frequency:.0f} Hz.")
        self.stepper_thread = threading.Thread(
            target=self._move_worker,
            args=(direction, seconds, frequency, self.simulation_var.get()),
            daemon=True,
        )
        self.stepper_thread.start()

    def stop_move(self):
        self.stop_event.set()
        worker = self.stepper_thread
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        self.stepper_thread = None
        self._set_outputs_safe()
        self.state_var.set("Ready")
        self._set_running(False)
        self.log("Stop requested.")

    def _direction_state(self, direction):
        return DIR_DOWN_STATE if direction == "up" else DIR_UP_STATE

    def _apply_motion_estimate(self, direction, pulses=None, elapsed_seconds=None, frequency=None):
        if pulses is None:
            pulses = max(0, int(round(float(elapsed_seconds or 0.0) * float(frequency or STEPPER_FREQUENCY_HZ))))
        delta_mm = float(pulses) * STEPPER_MM_PER_PULSE
        if direction == "up":
            delta_mm *= -1.0
        self.position_mm += delta_mm
        self.root.after(0, lambda: self._set_readouts(pulses=pulses))

    def _run_native_stepper(self, direction_state, seconds, frequency):
        started_at = time.perf_counter()
        chip_name = os.environ.get("PI_BUBBLE_GPIO_CHIP", "gpiochip0")
        command = [
            self.native_runner,
            "--chip",
            chip_name,
            "--pul",
            str(STEPPER_PUL_PIN),
            "--dir",
            str(STEPPER_DIR_PIN),
            "--ena",
            str(STEPPER_ENA_PIN),
            "--dir-state",
            str(int(direction_state)),
            "--ena-active",
            str(int(STEPPER_ENA_ACTIVE_STATE)),
            "--ena-inactive",
            str(int(STEPPER_ENA_INACTIVE_STATE)),
            "--frequency",
            str(float(frequency)),
            "--seconds",
            f"{float(seconds):.6f}",
        ]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        while process.poll() is None:
            if self.stop_event.is_set():
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
                return True, time.perf_counter() - started_at
            time.sleep(0.02)

        _stdout, stderr = process.communicate()
        if process.returncode != 0:
            message = stderr.strip() or f"stepper_runner exited with code {process.returncode}"
            raise RuntimeError(message)
        return False, time.perf_counter() - started_at

    def _run_gpio_stepper(self, direction_state, seconds, frequency):
        GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO_LOW)
        GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
        GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)
        GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_ACTIVE_STATE)
        GPIO.output(STEPPER_DIR_PIN, direction_state)
        time.sleep(0.002)

        delay = 1.0 / (2.0 * frequency)
        end_time = time.perf_counter() + seconds
        pulses = 0
        while time.perf_counter() < end_time and not self.stop_event.is_set():
            GPIO.output(STEPPER_PUL_PIN, GPIO_HIGH)
            time.sleep(delay)
            GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
            time.sleep(delay)
            pulses += 1
        return pulses

    def _move_worker(self, direction, seconds, frequency, simulation):
        direction_state = self._direction_state(direction)
        try:
            if simulation:
                started_at = time.perf_counter()
                while time.perf_counter() - started_at < seconds and not self.stop_event.is_set():
                    time.sleep(0.02)
                elapsed = time.perf_counter() - started_at
                self._apply_motion_estimate(direction, elapsed_seconds=elapsed, frequency=frequency)
                self.root.after(0, lambda: self.log("Simulation move complete." if not self.stop_event.is_set() else "Simulation stopped."))
                return

            if self.native_runner is not None:
                stopped, elapsed = self._run_native_stepper(direction_state, seconds, frequency)
                self._apply_motion_estimate(direction, elapsed_seconds=elapsed, frequency=frequency)
                self.root.after(0, lambda: self.log("Native move stopped." if stopped else "Native move complete."))
                return

            try:
                pulses = self._run_gpio_stepper(direction_state, seconds, frequency)
            except RuntimeError as exc:
                error_text = str(exc)
                if "Cannot determine SOC peripheral base address" in error_text:
                    self.simulation_var.set(True)
                    self.root.after(
                        0,
                        lambda: self.log(
                            "RPi.GPIO is not compatible with this board/OS. "
                            "Use native stepper_runner or Simulation."
                        ),
                    )
                raise
            self._apply_motion_estimate(direction, pulses=pulses)
            self.root.after(0, lambda: self.log("GPIO move stopped." if self.stop_event.is_set() else "GPIO move complete."))
        except Exception as exc:
            error_message = str(exc)
            self.root.after(0, lambda message=error_message: self.log(f"Stepper error: {message}"))
            self.root.after(0, lambda message=error_message: messagebox.showerror("Stepper Error", message))
        finally:
            self._set_outputs_safe()
            self.root.after(0, lambda: self.state_var.set("Ready"))
            self.root.after(0, lambda: self._set_running(False))

    def _set_outputs_safe(self):
        if GPIO is None:
            return
        try:
            GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
            GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
        except Exception:
            pass

    def on_close(self):
        self.stop_event.set()
        worker = self.stepper_thread
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        self._set_outputs_safe()
        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    StepperTestGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
