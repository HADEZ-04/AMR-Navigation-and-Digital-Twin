import openvr
import time
import math
import numpy as np

def extract_pose(mat):
    # Convert 3x4 OpenVR matrix into R, t
    R = np.array([
        [mat[0][0], mat[0][1], mat[0][2]],
        [mat[1][0], mat[1][1], mat[1][2]],
        [mat[2][0], mat[2][1], mat[2][2]],
    ])
    t = np.array([mat[0][3], mat[1][3], mat[2][3]])

    # Extract Yaw-Pitch-Roll (Z-Y-X) Euler angles
    yaw = math.atan2(R[1,0], R[0,0])
    pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
    roll = math.atan2(R[2,1], R[2,2])

    return t,R


openvr.init(openvr.VRApplication_Other)
system = openvr.VRSystem()

tracker_device_index = 1  # update based on your setup

try:
    while True:
        poses = system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)

        pose = poses[tracker_device_index]

        if pose.bDeviceIsConnected and pose.bPoseIsValid:
            mat = pose.mDeviceToAbsoluteTracking
            t,R = extract_pose(mat)

            print(f"Pos (m): {t}")
            print(f"Rot :{R}")
            # print(mat)

        else:
            print("Tracker not valid.")

        time.sleep(5)

except KeyboardInterrupt:
    pass

openvr.shutdown()
