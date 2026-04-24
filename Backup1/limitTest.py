import time
import RPi.GPIO as GPIO

# Raspberry Pi GPIO assignment (BCM numbering)
# Limit switch 1 (NO): one side -> GPIO17, other side -> GND
# Limit switch 2 (NO): one side -> GPIO27, other side -> GND
LIMIT_1_PIN = 17
LIMIT_2_PIN = 27

# Polling interval for console updates.
POLL_INTERVAL_SECONDS = 0.1


def is_triggered(pin):
	# NO switch with pull-up:
	# - Not pressed: HIGH
	# - Pressed (triggered): LOW
	return GPIO.input(pin) == GPIO.LOW


def format_state(name, triggered):
	return f"{name}: {'TRIGGERED' if triggered else 'not triggered'}"


def main():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LIMIT_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(LIMIT_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

	print("Limit switch test started (NO wiring)")
	print("GPIO mapping:")
	print(f"  Limit 1 -> GPIO{LIMIT_1_PIN}")
	print(f"  Limit 2 -> GPIO{LIMIT_2_PIN}")
	print("Press Ctrl+C to exit")

	last_state = (None, None)

	try:
		while True:
			l1 = is_triggered(LIMIT_1_PIN)
			l2 = is_triggered(LIMIT_2_PIN)
			current_state = (l1, l2)

			if current_state != last_state:
				print(
					f"{format_state('Limit1', l1)} | "
					f"{format_state('Limit2', l2)}"
				)
				last_state = current_state

			time.sleep(POLL_INTERVAL_SECONDS)

	except KeyboardInterrupt:
		print("\nExiting limit switch test")
	finally:
		GPIO.cleanup()


if __name__ == "__main__":
	main()
