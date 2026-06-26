from rplidar import RPLidar
import numpy as np
import matplotlib.pyplot as plt
import time
import paho.mqtt.client as mqtt 

# --- CONFIGURATION ---
PORT_NAME = '/dev/ttyUSB0'
MQTT_BROKER_ADDRESS = " " # !! REPLACE ME !! e.g., "192.168.1.100"
MQTT_TOPIC = "robot/pose"

# Lidar's fixed position and orientation relative to the robot's center (meters, radians)
xlr = 0.10  # m (10 cm)
ylr = 0.10  # m (10 cm)
thetalr = np.deg2rad(10)  # 10 degrees converted to radians

# Map properties
map_size = 20  # m
map_resolution = 0.3  # m
map_sizecells = int(map_size / map_resolution)
map_centrecells = map_sizecells // 2

# Static Map Update Values
PROB_OCCUPIED = 0.95      # High certainty for occupied space
PROB_FREE = 0.05          # High certainty for free space
NEW_MEASUREMENT_WEIGHT = 0.05 # Low weight for slow, stable map integration
STATIC_UPDATE_INTERVAL_SECONDS = 5 * 60  # Update the static map every 5 minutes

# Global variables to store the robot's pose received via MQTT from the Vive Tracker
GLOBAL_Xr = 0.0      # Global X position (meters)
GLOBAL_Yr = 0.0      # Global Y position (meters)
GLOBAL_Thetar = 0.0  # Global Yaw/Orientation (radians)


def on_connect(client, userdata, flags, rc):
    """Called when the client connects to the MQTT broker."""
    if rc == 0:
        print("MQTT Client Connected successfully.")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"MQTT Connection failed with code {rc}")

def on_message(client, userdata, msg):
    """Callback function for MQTT message reception (Vive Tracker Pose)."""
    global GLOBAL_Xr, GLOBAL_Yr, GLOBAL_Thetar
    
    try:
        # Assuming the tracker data is sent as a comma-separated string: "X,Y,Yaw"
        # X and Y must be in meters. Yaw must be in radians.
        data = msg.payload.decode()
        x_str, y_str, theta_str = data.split(',')
        
        GLOBAL_Xr = float(x_str)
        GLOBAL_Yr = float(y_str)
        GLOBAL_Thetar = float(theta_str)
        
    except Exception as e:
        print(f"Error parsing MQTT data: {e}. Payload: {msg.payload}")



def polar_to_cartesian(angle, distance):
    """Converts Lidar polar coordinates (angle, distance) to Lidar frame (x, y)."""
    rad = np.deg2rad(angle)
    x = distance * np.cos(rad)
    y = distance * np.sin(rad)
    return x, y

def transform_to_robot(x, y, xlr, ylr, thetalr):
    """Transforms a point from Lidar frame to Robot frame."""
    point = np.array([x, y])
    R_theta = np.array([[np.cos(thetalr), -np.sin(thetalr)],
                        [np.sin(thetalr), np.cos(thetalr)]])
    b = np.array([xlr, ylr])
    # pr = R * pl + t_lr
    xr, yr = R_theta @ point + b
    return xr, yr

def transform_to_global(xr, yr, Xr, Yr, thetar):
    """Transforms a point from Robot frame to Global frame (uses Vive pose)."""
    c = np.array([xr, yr])
    R_thetar = np.array([[np.cos(thetar), -np.sin(thetar)],
                         [np.sin(thetar), np.cos(thetar)]])
    d = np.array([Xr, Yr])
    # pg = R * pr + t_r
    xg, yg = R_thetar @ c + d
    return xg, yg

def world_to_map(xg, yg):
    """Converts global (m) coordinates to map cell indices (pixels)."""
    mx = int(xg / map_resolution) + map_centrecells
    my = int(yg / map_resolution) + map_centrecells
    return mx, my

def get_robot_pose():
    """Retrieves the latest robot pose from the Vive Tracker via global MQTT variables."""
    return GLOBAL_Xr, GLOBAL_Yr, GLOBAL_Thetar

def update_static_map(probability_grid, mx_lidar, my_lidar, mx_occ, my_occ):
    """
    Updates the probability grid using ray-casting and weighted averaging. 
    This writes scans to a temporary grid.
    """
    if not (0 <= mx_occ < map_sizecells and 0 <= my_occ < map_sizecells):
        return

    # 1. Line generation (Simplified Bresenham)
    dx = mx_occ - mx_lidar
    dy = my_occ - my_lidar
    num_steps = max(abs(dx), abs(dy))
    if num_steps < 1: return
    
    xs = np.linspace(mx_lidar, mx_occ, num_steps + 1, dtype=int)
    ys = np.linspace(my_lidar, my_occ, num_steps + 1, dtype=int)

    # 2. Update Free cells along the ray (all but the last point)
    for i in range(len(xs) - 1):
        mx_free, my_free = xs[i], ys[i]
        if 0 <= mx_free < map_sizecells and 0 <= my_free < map_sizecells:
            old_prob = probability_grid[my_free, mx_free]
            # Weighted average update for FREE space (PROB_FREE=0.05)
            new_prob = old_prob * (1 - NEW_MEASUREMENT_WEIGHT) + PROB_FREE * NEW_MEASUREMENT_WEIGHT
            probability_grid[my_free, mx_free] = new_prob

    # 3. Update Occupied cell (the hit point)
    old_prob = probability_grid[my_occ, mx_occ]
    # Weighted average update for OCCUPIED space (PROB_OCCUPIED=0.95)
    new_prob = old_prob * (1 - NEW_MEASUREMENT_WEIGHT) + PROB_OCCUPIED * NEW_MEASUREMENT_WEIGHT
    probability_grid[my_occ, mx_occ] = new_prob


# -----------------------------------------------------------------------------
# --- MAIN EXECUTION ---
# -----------------------------------------------------------------------------

def run():
    global xlr, ylr, thetalr 
    
    # --- MQTT Setup ---
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message 
    
    try:
        print(f"Connecting to MQTT broker at {MQTT_BROKER_ADDRESS}...")
        client.connect(MQTT_BROKER_ADDRESS, 1883, 60)
        client.loop_start() # Start the non-blocking thread to handle network traffic
    except Exception as e:
        print(f"FATAL: Could not connect to MQTT broker. Check IP address and connection: {e}")
        return

    # --- Lidar Setup ---
    try:
        lidar = RPLidar(PORT_NAME)
    except Exception as e:
        print(f"FATAL: Could not connect to RPLidar. Check port and connection: {e}")
        client.loop_stop()
        return
        
    # --- Map Grids ---
    static_map_grid = np.full((map_sizecells, map_sizecells), 0.5, dtype=float)
    temp_scan_grid = np.full((map_sizecells, map_sizecells), 0.5, dtype=float)

    # --- Time Tracking ---
    last_update_time = time.time()

    # --- Visualization Setup ---
    plt.ion()
    fig, ax = plt.subplots()
    # cmap='gray_r' displays 1.0 (occupied) as black, 0.0 (free) as white.
    map_img = ax.imshow(static_map_grid, cmap='gray_r', vmin=0, vmax=1)
    fig.canvas.draw()
    
    print('Starting Lidar Scan for Static Mapping...')

    try:        
        for scan in lidar.iter_scans():
            current_time = time.time()
            Xr, Yr, thetar = get_robot_pose() # **Vive Tracker Pose Data Used Here**

            # 1. Calculate Lidar's global position (Ray Start Point)
            R_thetar = np.array([[np.cos(thetar), -np.sin(thetar)],
                                 [np.sin(thetar), np.cos(thetar)]])
            t_lr = np.array([xlr, ylr])
            t_r = np.array([Xr, Yr])
            P_L_g = R_thetar @ t_lr + t_r
            xl_g, yl_g = P_L_g[0], P_L_g[1]
            mx_lidar, my_lidar = world_to_map(xl_g, yl_g)

            # 2. Update the TEMPORARY Scan Grid with current scan data
            for (quality, angle, distance) in scan:
                if quality < 15 or distance == 0:
                    continue
                
                distance_meter = distance / 1000.0 # distance in meters
                
                # Transformation pipeline: Lidar -> Robot -> Global -> Map Index
                x, y = polar_to_cartesian(angle, distance_meter)
                xr, yr = transform_to_robot(x, y, xlr, ylr, thetalr)
                xg, yg = transform_to_global(xr, yr, Xr, Yr, thetar)
                mx_occ, my_occ = world_to_map(xg, yg)
                
                update_static_map(temp_scan_grid, mx_lidar, my_lidar, mx_occ, my_occ)

            # 3. Check for Static Map Update Time (Every 5 minutes)
            if current_time - last_update_time >= STATIC_UPDATE_INTERVAL_SECONDS:
                print(f"Updating Static Map after {STATIC_UPDATE_INTERVAL_SECONDS/60} minutes ---")
                
                # --- Map Merge Logic (Blend Temp Data into Static Map) ---
                # A low NEW_MEASUREMENT_WEIGHT is used here (e.g., 0.05)
                # to ensure the merge is slow and stable.
                
                # Blend Occupied: If temp map is certain (e.g., > 0.6), reinforce static map occupancy.
                occupied_mask = temp_scan_grid > 0.6
                current_static_prob = static_map_grid[occupied_mask]
                static_map_grid[occupied_mask] = current_static_prob * (1 - NEW_MEASUREMENT_WEIGHT) + PROB_OCCUPIED * NEW_MEASUREMENT_WEIGHT
                
                # Blend Free: If temp map is certain (e.g., < 0.4), reinforce static map freeness.
                free_mask = temp_scan_grid < 0.4
                current_static_prob = static_map_grid[free_mask]
                static_map_grid[free_mask] = current_static_prob * (1 - NEW_MEASUREMENT_WEIGHT) + PROB_FREE * NEW_MEASUREMENT_WEIGHT
                
                # --- Reset for next interval ---
                temp_scan_grid.fill(0.5) 
                last_update_time = current_time 
                print(f"Static Map Updated.")

            # 4. Update visualization
            map_img.set_data(static_map_grid)
            elapsed_time = int(current_time - last_update_time)
            title = f'Static Map - Next Update in: {STATIC_UPDATE_INTERVAL_SECONDS - elapsed_time}s'
            ax.set_title(title)
            fig.canvas.draw()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nStopping Scan Loop.")
    finally:
        # Cleanup
        RPLidar.stop(lidar)
        RPLidar.disconnect(lidar)
        client.loop_stop() # Stop the MQTT thread
        client.disconnect()
        print("Lidar and MQTT disconnected.")

    # Final Save
    plt.imsave('final_static_map.png', static_map_grid, cmap='gray_r', vmin=0, vmax=1)
    print("Static map saved as final_static_map.png")
    plt.show()

if __name__ == '__main__':
    run()