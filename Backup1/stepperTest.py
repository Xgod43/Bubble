import time
import RPi.GPIO as GPIO

PUL_PIN = 24
DIR_PIN = 23
ENA_PIN = 26

MOVE_SECONDS = 4
PULSES_PER_REV = 3200   # 200 steps/rev × 16 microsteps
STEP_FREQUENCY_HZ = PULSES_PER_REV  # 1 rev/sec
PULSE_DELAY = 1.0 / (2.0 * STEP_FREQUENCY_HZ)

DIR_UP_STATE = GPIO.HIGH
DIR_DOWN_STATE = GPIO.LOW
ENA_ACTIVE_STATE = GPIO.LOW      # TB6600 active LOW
ENA_INACTIVE_STATE = GPIO.HIGH

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
    GPIO.setup(ENA_PIN, GPIO.OUT, initial=ENA_INACTIVE_STATE)

def move_for_seconds(direction_state, seconds=MOVE_SECONDS):
    GPIO.output(ENA_PIN, ENA_ACTIVE_STATE)
    GPIO.output(DIR_PIN, direction_state)
    time.sleep(0.002)
    end_time = time.perf_counter() + seconds
    while time.perf_counter() < end_time:
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(PULSE_DELAY)
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(PULSE_DELAY)
    GPIO.output(ENA_PIN, ENA_INACTIVE_STATE)

def main():
    setup_gpio()
    print("Stepper test ready.")
    print("Type: up, down, or quit")
    print(f"Each command moves motor for {MOVE_SECONDS:g} second(s).")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "up":
                move_for_seconds(DIR_UP_STATE, MOVE_SECONDS)
                print(f"Moved up for {MOVE_SECONDS:g} second(s)")
            elif cmd == "down":
                move_for_seconds(DIR_DOWN_STATE, MOVE_SECONDS)
                print(f"Moved down for {MOVE_SECONDS:g} second(s)")
            elif cmd in ("q", "quit", "exit"):
                print("Exiting")
                break
            else:
                print("Unknown command. Use: up, down, quit")
    finally:
        GPIO.output(PUL_PIN, GPIO.LOW)
        GPIO.output(ENA_PIN, ENA_INACTIVE_STATE)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
