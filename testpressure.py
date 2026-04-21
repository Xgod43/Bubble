import time

import adafruit_mprls
import board


if not hasattr(board, "I2C"):
    raise RuntimeError(
        "Detected a non-Adafruit 'board' module.\n"
        "Use the project virtualenv interpreter:\n"
        "/home/bubble/Desktop/AerkKungCNX/Pi/.venv/bin/python testpressure.py"
    )


def main() -> None:
    i2c = board.I2C()
    mprls = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)

    while True:
        try:
            pressure = mprls.pressure
            print(f"Pressure: {pressure:.2f} hPa")
            time.sleep(0.5)
        except OSError as exc:
            # Errno 121 is typical when wiring/address/bus is unstable.
            if getattr(exc, "errno", None) == 121:
                print("I2C Remote I/O error (121). Check SDA/SCL wiring, power, and I2C bus state.")
                time.sleep(1.0)
                continue
            raise


if __name__ == "__main__":
    main()