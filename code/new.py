import time
import Adafruit_PCA9685
import numpy as np
import paho.mqtt.client as mqtt
import math

MQTT_BROKER_ADDRESS = "192.168.137.57"
MQTT_TOPIC = "tracker/pose"

IsAligned=True
GLOBAL_Xr = 0.0
GLOBAL_Yr = 0.0
GLOBAL_Thetar = 0.0
X_d=0
Y_d=0
heading=0.0
d=None

# ---------- PCA9685 Setup ----------
pwm = Adafruit_PCA9685.PCA9685(busnum=1)
pwm.set_pwm_freq(50)

# ---------- Constants ----------
STOP = 307
CW_MAX = 204
CCW_MAX = 409
cw_range = [STOP, CW_MAX]
ccw_range = [STOP, CCW_MAX]
CH_WHEEL = [0, 1, 2, 3]


# ~ # Ask for speed
speed = int(input("Enter forward speed : "))

fspeed=speed-5

speed_f1 = 0.00001497 * (fspeed**3) - 0.002692 * (fspeed**2) + 0.9166 * fspeed - 0.3084
duty11 = int(np.interp(speed_f1, [0, 100], cw_range))

speed_f4 = 0.00001453 * (fspeed**3) - 0.003171 * (fspeed**2) + 0.9904 * speed + 3.381
duty41 = int(np.interp(speed_f4, [0, 100], cw_range))
           
speed_f2 = 0.000001777 * (fspeed**3) + 0.0001007 * (fspeed**2) + 0.7185 * fspeed + 1.926
duty21 = int(np.interp(speed_f2, [0, 100], cw_range))

speed_f3 = 0.000001777 * (fspeed**3) + 0.0001007 * (fspeed**2) + 0.7185 * fspeed + 1.926
duty31 = int(np.interp(speed_f3, [0, 100], cw_range))


speed_b1= 0.00003198 * ((speed-5)**3) - 0.006403 * ((speed-5)**2) + 1.166 * (speed-5) - 6.434
duty10 = int(np.interp(speed_b1, [0, 100], ccw_range))

speed_b2= 0.00002272 * (speed**3) - 0.004907 * (speed**2) + 1.072 * speed - 6.4
duty20 = int(np.interp(speed_b2, [0, 100], ccw_range))

speed_b3= 0.000009995 * (speed**3) - 0.002492 * (speed**2) + 0.9476 * speed - 4.095
duty30 = int(np.interp(speed_b3, [0, 100], ccw_range))

speed_b4= 0.00001485 * ((speed-5)**3) - 0.002752 * ((speed-5)**2) + 0.9342 * (speed-5) - 2.871
duty40 = int(np.interp(speed_b4, [0, 100], ccw_range))

def stop_motors():
    for ch in CH_WHEEL:
        pwm.set_pwm(ch, 0, int(STOP))
    # ~ print("Motors Stopped")

def generate_waypoints(x_start, y_start, x_goal, y_goal, ds):

    dx = x_goal - x_start
    dy = y_goal - y_start
    total_dist = math.hypot(dx, dy)

    if total_dist < ds:
        return [(x_goal, y_goal)]

    steps = int(total_dist / ds)
    waypoints = []

    for i in range(1, steps + 1):
        t = i / steps
        xi = x_start + t * dx
        yi = y_start + t * dy
        waypoints.append((xi, yi))

    return waypoints
def distance(x1,y1,x2,y2):
    return math.hypot(x2-x1,y2-y1)


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker")
        client.subscribe(MQTT_TOPIC)
    else:
        print("Connection failed, reason code:", reason_code)

        


def cw():
    pwm.set_pwm(0, 0, duty11)
    pwm.set_pwm(1, 0, duty20)
    pwm.set_pwm(2, 0, duty30)
    pwm.set_pwm(3, 0, duty41)
    # ~ print("Clock_wise")
        
def ccw():
    pwm.set_pwm(0, 0, duty10)
    pwm.set_pwm(1, 0, duty21)
    pwm.set_pwm(2, 0, duty31)
    pwm.set_pwm(3, 0, duty40)
    # ~ print("Anti_Clock_wise")


def moveTo(x,y):
    slope=math.degrees(math.atan2(x-GLOBAL_Xr,y-GLOBAL_Yr))
    while distance(GLOBAL_Xr,GLOBAL_Yr,x,y)>0.3:
        if GLOBAL_Thetar < slope - 3:
            pwm.set_pwm(0, 0, duty11)
            pwm.set_pwm(1, 0, STOP)
            pwm.set_pwm(2, 0, STOP)
            pwm.set_pwm(3, 0, duty41)

        elif GLOBAL_Thetar > slope + 3:
            pwm.set_pwm(0, 0, STOP)
            pwm.set_pwm(1, 0, duty21)
            pwm.set_pwm(2, 0, duty31)
            pwm.set_pwm(3, 0, STOP)

        else:
            pwm.set_pwm(0, 0, duty11)
            pwm.set_pwm(1, 0, duty21)
            pwm.set_pwm(2, 0, duty31)
            pwm.set_pwm(3, 0, duty41)
    stop_motors()
def on_message(client, userdata, msg):
    global GLOBAL_Xr, GLOBAL_Yr, GLOBAL_Thetar,d,heading,X_d,Y_d,GR
    try:
        data = msg.payload.decode()
        msg = data.split(',')
        try:
            if len(msg) == 3:
                GLOBAL_Xr, GLOBAL_Yr,GLOBAL_Thetar = map(float, msg)
            elif len(msg) == 5:
                X_d, Y_d,GLOBAL_Thetar,d,heading = map(float, msg)
                IsAligned=False
            else:
                print("Unknown payload format:")
            
            GR=distance(GLOBAL_Xr,GLOBAL_Yr,X_d,Y_d)

        except ValueError:
            print("Parse error:")

    except Exception as e:
        print(f"Error parsing MQTT: {e}. Payload: {msg.payload}")

def align_heading():
    global GLOBAL_Thetar,d,heading
    if d>0:
        while GLOBAL_Thetar<heading-2 or GLOBAL_Thetar>heading+2:
            cw()
        IsAligned=True
        stop_motors()
    elif d<0: 
        while GLOBAL_Thetar<heading-2 or GLOBAL_Thetar>heading+2:
            ccw()
        IsAligned=True
        stop_motors()
    else:
     IsAligned=True
     print("Motor Stop")
     stop_motors()
     
     
# ---------- MQTT Client ----------
client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2
)
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(MQTT_BROKER_ADDRESS, 1883, 60)
    client.loop_start()
    while True:
        while IsAligned==False:
            align_heading()
        moveTo(X_d,Y_d)
        stop_motors()
        break
        time.sleep(0.1)
except KeyboardInterrupt:
    stop_motors()
    print("\nProgram Ended Safely.")

