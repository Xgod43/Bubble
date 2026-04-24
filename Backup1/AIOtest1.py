import select
import sys
import termios
import time
import tty
from pathlib import Path
import RPi.GPIO as GPIO
STEPPER_PUL_PIN = 24
STEPPER_DIR_PIN = 23
STEPPER_ENA_PIN = 26
LIMIT_1_PIN = 17
LIMIT_2_PIN = 27
LOADCELL_DT_PIN = 5
LOADCELL_SCK_PIN = 6
STEPPER_ENA_ACTIVE_STATE = GPIO.HIGH
STEPPER_ENA_INACTIVE_STATE = GPIO.LOW
"""
Load Cell
Red E+
Black E-
White A-
Green A+ 
Module
VCC
SCK
DT
GND
"""
TESTS = {
	"1": "Stepper Test",
	"2": "Limit Switch Test",
	"3": "Pressure Sensor Test",
	"4": "Pi Camera Test",
	"5": "Load Cell Test",
}
class KeyPoller:
	def __init__(self):
		self.fd = None
		self.old_settings = None
	def __enter__(self):
		self.fd = sys.stdin.fileno()
		self.old_settings = termios.tcgetattr(self.fd)
		tty.setcbreak(self.fd)
		return self
	def __exit__(self, exc_type, exc, tb):
		if self.fd is not None and self.old_settings is not None:
			termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
	def get_key(self, timeout=0.0):
		ready, _, _ = select.select([sys.stdin], [], [], timeout)
		if ready:
			return sys.stdin.read(1)
		return None
def setup_stepper_gpio():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)
def stepper_move_for_seconds(direction_high, seconds=1.0, frequency_hz=800):
	pulse_delay = 1.0 / (2.0 * frequency_hz)
	GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_ACTIVE_STATE)
	GPIO.output(STEPPER_DIR_PIN, GPIO.HIGH if direction_high else GPIO.LOW)
	time.sleep(0.002)
	end_time = time.perf_counter() + seconds
	while time.perf_counter() < end_time:
		GPIO.output(STEPPER_PUL_PIN, GPIO.HIGH)
		time.sleep(pulse_delay)
		GPIO.output(STEPPER_PUL_PIN, GPIO.LOW)
		time.sleep(pulse_delay)
	GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
def run_stepper_test():
	print("\n=== Stepper Test (1) ===")
	print("Pins: PUL=GPIO24, DIR=GPIO23, ENA=GPIO26")
	print("Commands: up, down, quit")
	print("Each movement runs for 5 seconds.")
	setup_stepper_gpio()
	try:
		while True:
			cmd = input("> ").strip().lower()
			if cmd == "down":
				stepper_move_for_seconds(direction_high=True, seconds=5.0)
				print("Moved down")
			elif cmd == "up":
				stepper_move_for_seconds(direction_high=False, seconds=5.0)
				print("Moved up")
			elif cmd in ("q", "quit", "exit"):
				break
			else:
				print("Unknown command. Use up, down, quit")
	finally:
		GPIO.output(STEPPER_PUL_PIN, GPIO.LOW)
		GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
		GPIO.cleanup()
def is_limit_triggered(pin):
	return GPIO.input(pin) == GPIO.LOW
def run_limit_test():
	print("\n=== Limit Switch Test (2) ===")
	print("Pins: Limit1=GPIO17, Limit2=GPIO22")
	print("Press q to exit test.")
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(LIMIT_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	last_state = (None, None)
	try:
		with KeyPoller() as poller:
			while True:
				l1 = is_limit_triggered(LIMIT_1_PIN)
				l2 = is_limit_triggered(LIMIT_2_PIN)
				current = (l1, l2)
				if current != last_state:
					print(
						"Limit1: "
						+ ("TRIGGERED" if l1 else "not triggered")
						+ " | Limit2: "
						+ ("TRIGGERED" if l2 else "not triggered")
					)
					last_state = current
				key = poller.get_key(timeout=0.05)
				if key and key.lower() == "q":
					break
				time.sleep(0.05)
	finally:
		GPIO.cleanup()
def run_pressure_test():
	print("\n=== Pressure Sensor Test (3) ===")
	print("Using default Raspberry Pi I2C pins (SDA/SCL).")
	print("Press q to exit test.")
	try:
		import board
		import adafruit_mprls
	except Exception as exc:
		print(f"Pressure test unavailable: {exc}")
		return
	i2c = board.I2C()
	sensor = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
	with KeyPoller() as poller:
		while True:
			pressure_hpa = float(sensor.pressure)
			print(f"Pressure: {pressure_hpa:.2f} hPa")
			for _ in range(10):
				key = poller.get_key(timeout=0.05)
				if key and key.lower() == "q":
					return
				time.sleep(0.05)
def run_camera_test():
	print("\n=== Pi Camera Test (4) ===")
	print("Opening camera preview. Press q to exit test.")
	try:
		from picamzero import Camera
	except Exception as exc:
		print(f"Camera test unavailable: {exc}")
		return
	cam = Camera()
	try:
		cam.start_preview()
		with KeyPoller() as poller:
			while True:
				key = poller.get_key(timeout=0.1)
				if key and key.lower() == "q":
					break
	finally:
			try:
				cam.stop_preview()
			except Exception:
				pass
def _load_hx711_class():
	hx_path = Path(__file__).resolve().parent / "library" / "hx711py"
	if str(hx_path) not in sys.path:
		sys.path.insert(0, str(hx_path))
	from hx711v0_5_1 import HX711
	return HX711
def _read_average_raw(hx, samples=20, delay=0.05, channel="A"):
	values = []
	for _ in range(samples):
		value = hx.getLong(channel)
		if value is not None:
			values.append(float(value))
		time.sleep(delay)
	if not values:
		raise RuntimeError("No load cell readings captured.")
	return sum(values) / len(values)
def run_loadcell_test():
	print("\n=== Load Cell Test (5) ===")
	print("Pins: DT=GPIO5, SCK=GPIO6")
	print("Read mode: interrupt (stable)")
	print("Press q to exit test.")

	try:
		HX711 = _load_hx711_class()
	except Exception as exc:
		print(f"Load cell test unavailable: {exc}")
		return

	hx = None
	latest_grams = {"value": None}

	try:
		hx = HX711(LOADCELL_DT_PIN, LOADCELL_SCK_PIN)
		hx.setReadingFormat("MSB", "MSB")
		hx.reset()

		print("Remove all weight from load cell...")
		time.sleep(2)
		offset = _read_average_raw(hx, channel="A")
		hx.setOffset(offset, "A")
		print(f"Tare offset: {offset:.2f}")

		known_text = input("Enter known weight in grams (e.g. 500): ").strip()
		if not known_text:
			print("No calibration value entered. Skipping load cell test.")
			return
		known_weight = float(known_text)
		if known_weight <= 0:
			print("Calibration weight must be > 0. Skipping load cell test.")
			return

		print(f"Place {known_weight:.2f} g on the sensor...")
		time.sleep(3)
		loaded_raw = _read_average_raw(hx, channel="A")
		delta = loaded_raw - offset
		if abs(delta) < 1e-9:
			print("Calibration failed: delta too small.")
			return

		counts_per_gram = delta / known_weight
		hx.setReferenceUnit(counts_per_gram, "A")
		print(f"Calibration complete. counts_per_gram={counts_per_gram:.6f}")

		def callback(raw_bytes):
			if raw_bytes is None:
				return
			latest_grams["value"] = float(hx.rawBytesToWeight(raw_bytes, "A"))

		hx.enableReadyCallback(callback)

		with KeyPoller() as poller:
			while True:
				grams = latest_grams["value"]
				if grams is not None:
					kg = grams / 2000.0
					print(f"Weight: {grams:.2f} g ({kg:.4f} kg)")

				for _ in range(5):
					key = poller.get_key(timeout=0.05)
					if key and key.lower() == "q":
						return
					time.sleep(0.05)
	finally:
		if hx is not None and getattr(hx, "readyCallbackEnabled", False):
			try:
				hx.disableReadyCallback()
			except RuntimeError:
				pass
		GPIO.cleanup()


def parse_selection(raw):
	text = raw.strip().lower()
	if text == "all":
		return ["1", "2", "3", "4", "5"]
