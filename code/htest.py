import time
import Adafruit_PCA9685
import numpy as np
import paho.mqtt.client as mqtt
import math

MQTT_BROKER_ADDRESS = "192.168.137.57"

IsAligned=True
GLOBAL_Xr = None
GLOBAL_Yr = None
GLOBAL_Thetar =None
X_d=None
Y_d=None
GR=0.2
WR=0.3
ds=0.7
theta_tol=3
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


speed = int(input("Enter forward speed : "))
dutyf = int(np.interp(speed, [0, 100], cw_range))
dutyb = int(np.interp(speed, [0, 100], ccw_range))

def distance(x1,y1,x2,y2):
    return math.hypot(x2-x1,y2-y1)
def shortest_rotation(current, target):

    d = target - current
    d = (d + 180) % 360 - 180   # wrap into (-180, 180]
    return d
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
def stop_motors():
    for ch in CH_WHEEL:
        pwm.set_pwm(ch, 0, int(STOP))
        # ~ print("Motors Stopped")
def cw():
    pwm.set_pwm(0, 0, dutyf)
    pwm.set_pwm(1, 0, dutyb)
    pwm.set_pwm(2, 0, dutyb)
    pwm.set_pwm(3, 0, dutyf)
    # ~ print("Clock_wise")        
def ccw():
    pwm.set_pwm(0, 0, dutyb)
    pwm.set_pwm(1, 0, dutyf)
    pwm.set_pwm(2, 0, dutyf)
    pwm.set_pwm(3, 0, dutyb)
    # ~ print("Anti_Clock_wise")
# def moveTo(x,y):
#     time.sleep(0.1)
#     slope=math.degrees(math.atan2(x-GLOBAL_Xr,y-GLOBAL_Yr))
#     print(x,y,slope)
#     while distance(GLOBAL_Xr,GLOBAL_Yr,x,y)>WR:
#         if GLOBAL_Thetar < slope - 3:
#             pwm.set_pwm(0, 0, dutyf)
#             pwm.set_pwm(1, 0, STOP)
#             pwm.set_pwm(2, 0, STOP)
#             pwm.set_pwm(3, 0, dutyf)

#         elif GLOBAL_Thetar > slope + 3:
#             pwm.set_pwm(0, 0, STOP)
#             pwm.set_pwm(1, 0, dutyf)
#             pwm.set_pwm(2, 0, dutyf)
#             pwm.set_pwm(3, 0, STOP)

#         else:
#             pwm.set_pwm(0, 0, dutyf)
#             pwm.set_pwm(1, 0, dutyf)
#             pwm.set_pwm(2, 0, dutyf)
#             pwm.set_pwm(3, 0, dutyf)
#     print("Reached")

# def align_heading():
#     time.sleep(0.1)
#     print("ALIGNING")
#     slope=math.degrees(math.atan2(X_d-GLOBAL_Xr,Y_d-GLOBAL_Yr))
#     d=shortest_rotation(GLOBAL_Thetar,slope)
#     if d>0:
#         while GLOBAL_Thetar<slope-2 or GLOBAL_Thetar>slope+2:
#             cw()
#     elif d<0: 
#         while GLOBAL_Thetar<slope-2 or GLOBAL_Thetar>slope+2:
#             ccw()
#         IsAligned=True
#         stop_motors()     
#     IsAligned=True
#     print("DONE!")
#     stop_motors()

def align_heading():
    global IsAligned
    time.sleep(0.1)
    print("ALIGNING")
    slope=math.degrees(math.atan2(Y_d-GLOBAL_Yr,X_d-GLOBAL_Xr))
    d=shortest_rotation(GLOBAL_Thetar,slope)
    while(abs(d) <= theta_tol):
        d=shortest_rotation(GLOBAL_Thetar,slope)
        if d>0:
            cw()
        elif d<0:
            ccw()
    IsAligned=True
    print("DONE!")
    stop_motors()
    
def moveTo(x,y):
    time.sleep(0.1)
    slope=math.degrees(math.atan2(Y_d-GLOBAL_Yr,X_d-GLOBAL_Xr))
    while distance(GLOBAL_Xr,GLOBAL_Yr,x,y)>WR:
        d=shortest_rotation(GLOBAL_Thetar,slope)
        if d>theta_tol:
            pwm.set_pwm(0, 0, dutyf)
            pwm.set_pwm(1, 0, STOP)
            pwm.set_pwm(2, 0, STOP)
            pwm.set_pwm(3, 0, dutyf)
        elif d<-theta_tol:
            pwm.set_pwm(0, 0, STOP)
            pwm.set_pwm(1, 0, dutyf)
            pwm.set_pwm(2, 0, dutyf)
            pwm.set_pwm(3, 0, STOP)
        else:
            pwm.set_pwm(0, 0, dutyf)
            pwm.set_pwm(1, 0, dutyf)
            pwm.set_pwm(2, 0, dutyf)
            pwm.set_pwm(3, 0, dutyf)        

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT broker")
        client.subscribe("agv/#")
    else:
        print("Connection failed, reason code:", reason_code)    
def on_message(client, userdata, msg):
    global GLOBAL_Xr, GLOBAL_Yr, GLOBAL_Thetar,X_d,Y_d,IsAligned
    try:
        data = msg.payload.decode()
        if msg.topic=="agv/tar_coord":
            msg = data.split(',')
            X_d, Y_d= map(float, msg)
            IsAligned=False
        if msg.topic=="agv/Live_coord":
            msg = data.split(',')
            GLOBAL_Xr, GLOBAL_Yr,GLOBAL_Thetar = map(float, msg)            
        else:
            print("Unknown payload format:")  
    except ValueError:
        print("Parse error:")

    except Exception as e:
        print(f"Error parsing MQTT: {e}. Payload: {msg.payload}")
client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(MQTT_BROKER_ADDRESS, 1883, 60)
    client.loop_start()
    while True:
        if IsAligned==False:
            align_heading()
            waypoints = generate_waypoints(GLOBAL_Xr, GLOBAL_Yr,X_d,Y_d,ds)
            wp_index = 0
            while wp_index!=len(waypoints):
                x_wp, y_wp = waypoints[wp_index]
                moveTo(x_wp, y_wp)
                wp_index += 1
                stop_motors()
                time.sleep(1)
            stop_motors()
            break
        time.sleep(0.1)
except KeyboardInterrupt:
    stop_motors()
    print("\nProgram Ended Safely.")

