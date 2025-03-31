# sim_config.py

# === 🗺️ Mapping Parameters ===
params_map = {
    "MAP_RESOLUTION": 0.02,
    "OCCUPANCY_UP": 1,
    "OCCUPANCY_DOWN": 0.3,
    "MAP_CENTER": (-52.5, -59.0),
    "MAP_SIZE": (24, 24),
    "MAP_FILENAME": "test.png",
    "MAPVIS_RESIZE_SCALE": 1.0,
}

# === 🧭 LiDAR Sensor Parameters ===
params_lidar = {
    "Range": 90,  # min & max range of lidar azimuths
    "CHANNEL": 1,  # vertical channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": 1206,
    "X": 0.0,  # position (meters)
    "Y": 0.0,
    "Z": 0.10,
    "YAW": 0.0,  # orientation (degrees)
    "PITCH": 0.0,
    "ROLL": 0.0,
}

# === 📷 Camera Sensor Parameters ===
params_cam = {
    "WIDTH": 320,
    "HEIGHT": 240,
    "FOV": 60,
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": 65000,
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.19,
    "YAW": 0.0,
    "PITCH": 0.0,
    "ROLL": 0.0,
}

# === 🔄 Monte Carlo Localization (MCL) Parameters ===
params_mcl = {
    "NUM_PARTICLE": 500,
    "MIN_ODOM_DISTANCE": 0.1,
    "MIN_ODOM_ANGLE": 1,
    "REPROPAGETE_COUNT": 1,
    "ODOM_TRANSLATION_COVARIANCE": 0.025,
    "ODOM_HEADING_COVARIANCE": 0.001,  # previously 0.0005
}
