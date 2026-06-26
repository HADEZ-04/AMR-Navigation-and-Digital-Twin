import openvr
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

def steamvr_pose_to_matrix(m):
    M = np.eye(4)
    for r in range(3):
        for c in range(4):
            M[r, c] = m[r][c]
    return M

def yaw_from_matrix(M):
    # yaw = atan2(r21, r11)
    yaw = math.atan2(M[1, 0], -M[0, 0])
    return math.degrees(yaw)
def shortest_rotation(current, target):

    d = target - current
    d = (d + 180) % 360 - 180   # wrap into (-180, 180]
    return d

def get_pose():
        poses = system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )
        pose = poses[TRACKER_ID]

        if pose.bDeviceIsConnected and pose.bPoseIsValid:
            M_steamvr = steamvr_pose_to_matrix(pose.mDeviceToAbsoluteTracking)
            M_world = T_world_from_steamvr @ M_steamvr
            pos_world = M_world[:3, 3]
            yaw_deg = yaw_from_matrix(M_world)
            X_cm = pos_world[0] 
            Y_cm = pos_world[1] 
            return X_cm,Y_cm,yaw_deg
        else:
            print("Tracker not valid or not connected")
        time.sleep(0.3)


calib_pos = np.array([1.3885144,-1.63984489,-4.17149496])  #implication of translation in rotation matrix

# Your rotation matrix (from the earlier solution)
R_calib = np.array([[[9.29396093e-01,3.69040519e-01,-5.66030247e-03],
 [-5.09585952e-03,-2.50406191e-03,-9.99983788e-01],
 [-3.69048744e-01,9.29409802e-01,-4.46679100e-04]]] )

T_calib = np.eye(4)
T_calib[:3, :3] = R_calib
T_calib[:3, 3] = calib_pos

T_world_from_steamvr = np.linalg.inv(T_calib)

openvr.init(openvr.VRApplication_Other)
system = openvr.VRSystem()
TRACKER_ID = 1   

#---------------------------------------------------------------------------------------------------------------------------------------------------------------
MQTT_BROKER = "192.168.137.57" 
MQTT_TOPIC  = "agv/Live_coord"
a=0
d=0
client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

try:
    while True:
            # if a==0:
                # x2=float(input("Enter x2 :"))
                # y2=float(input("Enter y2 :"))
                # x,y,yaw=get_pose()
                # slope=math.degrees(math.atan2(y2-y,x2-x))
                # d=shortest_rotation(yaw,slope)

                # payload = f"{x2:.1f},{y2:.1f},{yaw:.1f},{d:.1f},{slope:1f}"
                # client.publish(MQTT_TOPIC, payload,)
                # a=1
                
            x,y,yaw=get_pose()    
            payload = f"{x:.3f},{y:.3f},{yaw:.3f}"
            client.publish(MQTT_TOPIC, payload)
            print(payload)
            # print(f"x={x:1f} y={y:1f} yaw={yaw:.1f} Heading={slope:.1f} D={d:.1f}")

except KeyboardInterrupt:
    print("Stopping...")
openvr.shutdown()
