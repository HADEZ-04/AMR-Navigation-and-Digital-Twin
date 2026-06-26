import serial
import matplotlib.pyplot as plt
import time

PORT = "COM11"        
BAUD = 9600
DURATION = 2 * 60   

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2) 

timestamps = []
thetas = []

start_time = time.time()

print("Collecting data for 15 minutes...")

while (time.time() - start_time) < DURATION:
    try:
        line = ser.readline().decode("utf-8").strip()
        if line:
            theta = float(line)
            elapsed = time.time() - start_time
            timestamps.append(elapsed / 60.0)  # store time in minutes
            thetas.append(theta)
            print(f"{elapsed:.1f} sec | Theta = {theta:.2f}°")
    except:
        pass

ser.close()

print("Data collection complete!")

# Plot Drift
plt.figure(figsize=(10, 6))
plt.plot(timestamps, thetas, label="Pitch (Theta)")
plt.xlabel("Time (minutes)")
plt.ylabel("Pitch angle (degrees)")
plt.title("MPU6050 ACCEL Drift (1 minute)")
plt.xlim(0, 2)          
plt.ylim(-10, 10)       
plt.legend()
plt.show()
