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
def distance(x1,y1,x2,y2):
    return math.hypot(x2-x1,y2-y1)

def moveTo(xd, yd):
    payload = f"{xd:.3f},{yd:.3f}"
    client.publish("agv/tar_coord", payload,)
    x,y,yaw=get_pose()
    print(yaw)
    print(math.degrees(math.atan2(yd-y,-(xd-x))))
    while distance(x,y,xd,yd)>0.19:
        x,y,yaw=get_pose()    
        payload = f"{x:.3f},{y:.3f},{yaw:.3f}"
        client.publish("agv/Live_coord", payload,)
        # print(f"x={x:1f} y={y:1f} yaw={yaw:.1f}")



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
MQTT_BROKER = "192.168.137.57" 
client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()
# x1,y1=-1.5,1
# x2,y2=-3.5,1
# print(math.degrees(math.atan2(y2-y1,-(x2-x1))))
# try:
# while True:
#     x,y,yaw=get_pose()
#     print(f"x={x:1f} y={y:1f} yaw={yaw:.1f}")
moveTo(0,0)
# time.sleep(1)
# moveTo(-3.5,1)
# time.sleep(1)
# moveTo(-1.5,1)
# time.sleep(1)
# moveTo(-1.5,3)


    
        # xd=float(input("Enter x2 :"))
        # yd=float(input("Enter y2 :"))
        # payload = f"{xd:.3f},{yd:.3f}"
        # client.publish("agv/tar_coord", payload,)
        # x,y,yaw=get_pose()
        # while distance(x,y,xd,yd)>0.2:
        #     x,y,yaw=get_pose()    
        #     payload = f"{x:.3f},{y:.3f},{yaw:.3f}"
        #     client.publish("agv/Live_coord", payload,)
        #     print(f"x={x:1f} y={y:1f} yaw={yaw:.1f}")
# except KeyboardInterrupt:
#     print("Stopping...")
# openvr.shutdown()



  
