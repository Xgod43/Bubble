#CircuitPython na ja
import time
import board
import adafruit_mprls
i2c = board.I2C()
mprls = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
while True:
    pressure = mprls.pressure
    print(f"Pressure: {pressure:.2f} hPa")
    time.sleep(0.5)