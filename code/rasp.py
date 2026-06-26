import time
# import Adafruit_PCA9685
import numpy as np
import paho.mqtt.client as mqtt
import math

MQTT_BROKER_ADDRESS = "10.9.51.57"
MQTT_TOPIC = "tracker/pose"

GLOBAL_Xr = 0.0
GLOBAL_Yr = 0.0
GLOBAL_Thetar = 0.0
d=0

# ---------- PCA9685 Setup ----------
# pwm = Adafruit_PCA9685.PCA9685(busnum=1)
# pwm.set_pwm_freq(50)

# ---------- Constants ----------
STOP = 307
CW_MAX = 204
CCW_MAX = 409
cw_range = [STOP, CW_MAX]
ccw_range = [STOP, CCW_MAX]
CH_WHEEL = [0, 1, 2, 3]


# Ask for speed
speed = int(input("Enter forward speed : "))


duty = int(np.interp(speed, [0, 100], cw_range))
duty_b = int(np.interp(speed, [0, 100], ccw_range))



# ---------- MQTT Callbacks ----------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Connected")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"MQTT Error {rc}")
        
# def stop_motors():
#     for ch in CH_WHEEL:
#         pwm.set_pwm(ch, 0, int(STOP))
#     print("Motors Stopped")

# def cw():
#         pwm.set_pwm(0, 0, duty)
#         pwm.set_pwm(1, 0, duty_b)
#         pwm.set_pwm(2, 0, duty_b)
#         pwm.set_pwm(3, 0, duty)
#         print("Clock_wise")
        
# def ccw():
#         pwm.set_pwm(0, 0, duty_b)
#         pwm.set_pwm(1, 0, duty)
#         pwm.set_pwm(2, 0, duty)
#         pwm.set_pwm(3, 0, duty_b)
#         print("Anti_Clock_wise")


# def straight_line():
    if GLOBAL_Thetar < heading - 7:
        pwm.set_pwm(0, 0, duty)
        pwm.set_pwm(1, 0, STOP)
        pwm.set_pwm(2, 0, STOP)
        pwm.set_pwm(3, 0, duty)

    elif GLOBAL_Thetar > heading + 7:
        pwm.set_pwm(0, 0, STOP)
        pwm.set_pwm(1, 0, duty)
        pwm.set_pwm(2, 0, duty)
        pwm.set_pwm(3, 0, STOP)

    else:
        pwm.set_pwm(0, 0, duty)
        pwm.set_pwm(1, 0, duty)
        pwm.set_pwm(2, 0, duty)
        pwm.set_pwm(3, 0, duty)

def on_message(client, userdata, msg):
    global GLOBAL_Xr, GLOBAL_Yr, GLOBAL_Thetar, duty,duty_b
    global a, heading, disp  # you modify these inside

    try:
        data = msg.payload.decode()
        x_str, y_str, yaw_str = data.split(',')
        GLOBAL_Xr = float(x_str)
        GLOBAL_Yr = float(y_str)
        GLOBAL_Thetar = float(yaw_str)
        # if d>0 and GLOBAL_Thetar<:
        #     cw()
        # elif GLOBAL_Thetar>(heading+14):
        #     ccw()
        # else:
        #     straight_line()

    except Exception as e:
        print(f"Error parsing MQTT: {e}. Payload: {msg.payload}")


# ---------- MQTT Client ----------
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(MQTT_BROKER_ADDRESS, 1883, 60)

    client.loop_start()
    print(GLOBAL_Thetar)

except KeyboardInterrupt:
    # stop_motors()
    print("\nProgram Ended Safely.")

