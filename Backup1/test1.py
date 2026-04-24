import time
import board
import adafruit_mprls
i2c = board.I2C()
mprls = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
import matplotlib.pyplot as plt
times = []
pressures = []
print("Ready? only 5 seconds to prepare to start")
time.sleep(1)
print("5")
time.sleep(1)
print("4")
time.sleep(1)
print("3")
time.sleep(1)
print("2")
time.sleep(1)
print("1")
time.sleep(1)
print("0")
print("10 seconds for waiting for output and then i will start to plot graph")
start_time = time.time()
while time.time() - start_time <= 10.5:
    current_time = time.time() - start_time
    pressure = mprls.pressure
    times.append(current_time)
    pressures.append(pressure)
    print(f"Time: {current_time:.2f} s | Pressure: {pressure:.3f} hPa")
    time.sleep(0.5)
print("Finish la jaa | Gam lung ja plot graph jaa")
plt.figure(figsize=(10, 5))
plt.plot(times, pressures, marker='o', color='b', linestyle='-')
plt.title('Pressure and Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Pressure (hPa)')
plt.grid(True)
filename = 'aerkjaza.png'
plt.savefig(filename)
print(f"Save plot graph picture la jaa: {filename}")