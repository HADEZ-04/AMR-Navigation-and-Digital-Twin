import openvr
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

heading=0
def steamvr_pose_to_matrix(m):
    M = np.eye(4)
    for r in range(3):
        for c in range(4):
            M[r, c] = m[r][c]
    return M

def yaw_from_matrix(M):
    yaw = math.atan2(M[1, 0], M[0, 0])
    return math.degrees(yaw)

# STEP 1 — Your saved calibration pose
calib_pos = np.array([1.3885144,-1.63984489,-4.17149496])  #implication of translation in rotation matrix

# Your rotation matrix (from the earlier solution)
R_calib = np.array([[[9.29396093e-01,3.69040519e-01,-5.66030247e-03],
 [-5.09585952e-03,-2.50406191e-03,-9.99983788e-01],
 [-3.69048744e-01,9.29409802e-01,-4.46679100e-04]]] )

# Build 4x4 matrix
T_calib = np.eye(4)
T_calib[:3, :3] = R_calib
T_calib[:3, 3] = calib_pos

# -------------------------------------------
# STEP 2 — World transform = inverse of calibration pose
# -------------------------------------------
T_world_from_steamvr = np.linalg.inv(T_calib)

openvr.init(openvr.VRApplication_Other)
system = openvr.VRSystem()
TRACKER_ID = 1 

plt.ion()
fig, ax = plt.subplots()
scat = ax.scatter(0, 0, c='r', marker='o')
ax.set_xlim(-5, 5) 
ax.set_ylim(-5, 5)  
ax.set_xlabel('X (cm)')
ax.set_ylabel('Y (cm)')
ax.set_title('Vive Tracker Position (2D X-Y, cm)')

try:
    while True:
        poses = system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

        pose = poses[TRACKER_ID]

        if pose.bDeviceIsConnected and pose.bPoseIsValid:

            # SteamVR → Tracker now
            M_steamvr = steamvr_pose_to_matrix(pose.mDeviceToAbsoluteTracking)

            # Convert SteamVR → YourWorld
            M_world = T_world_from_steamvr @ M_steamvr

            pos_world = M_world[:3, 3]

            yaw_deg = yaw_from_matrix(M_world)
            print(yaw_deg)

            X_cm = pos_world[0] 
            Y_cm = pos_world[1] 

            scat.set_offsets((X_cm,Y_cm))
            plt.draw()
            plt.pause(0.05)
            # print(M_world)

            # print(f"World Position [mm]: X={X_cm}, Y={Y_cm:}, Z={yaw_deg}")
            #print(M_world[1, 0],M_world[0, 0])


        else:
            print("Tracker not valid or not connected")
        time.sleep(0.3)

except KeyboardInterrupt:
    print("Stopping...")

openvr.shutdown()
