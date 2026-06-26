import numpy as np
STOP = 307.377
CW_MAX = 204.91   # Forward max
CCW_MAX = 409.83  # Reverse max
CH_WHEEL = [0, 1, 2, 3]  # PCA channels for motors
cw_range = [STOP, CW_MAX]   # Forward
ccw_range = [STOP, CCW_MAX]   # Reverse



# ---------- Get input speed once ----------
speed = float(input("Enter forward speed : "))




speed_f1 = 0.00001497 * (speed**3) - 0.002692 * (speed**2) + 0.9166 * speed - 0.3084
duty11 = int(np.interp(speed_f1, [0, 100], cw_range))

speed_f4 = 0.00001453 * (speed**3) - 0.003171 * (speed**2) + 0.9904 * speed + 3.381
duty41 = int(np.interp(speed_f4, [0, 100], cw_range))
           
speed_f2 = 0.000001777 * (speed**3) + 0.0001007 * (speed**2) + 0.7185 * speed + 1.926
duty21 = int(np.interp(speed_f2, [0, 100], cw_range))

speed_f3 = 0.000001777 * (speed**3) + 0.0001007 * (speed**2) + 0.7185 * speed + 1.926
duty31 = int(np.interp(speed_f3, [0, 100], cw_range))


speed_b1= 0.00003198 * (speed**3) - 0.006403 * (speed**2) + 1.166 * speed - 6.434
duty10 = int(np.interp(speed_b1, [0, 100], ccw_range))

speed_b2= 0.00002272 * (speed**3) - 0.004907 * (speed**2) + 1.072 * speed - 6.4
duty20 = int(np.interp(speed_b2, [0, 100], ccw_range))

speed_b3= 0.000009995 * (speed**3) - 0.002492 * (speed**2) + 0.9476 * speed - 4.095
duty30 = int(np.interp(speed_b3, [0, 100], ccw_range))

speed_b4= 0.00001485 * (speed**3) - 0.002752 * (speed**2) + 0.9342 * speed - 2.871
duty40 = int(np.interp(speed_b4, [0, 100], ccw_range))

# In[ ]:




