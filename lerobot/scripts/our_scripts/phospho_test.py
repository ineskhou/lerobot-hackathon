import requests
import time

# Configuration
BASE_URL = "http://127.0.0.1:80"
INIT_ENDPOINT = f"{BASE_URL}/move/init"
RELATIVE_ENDPOINT = f"{BASE_URL}/move/relative"

# Give the server a moment to spin up
time.sleep(1)

# Initialize the robot
try:
    init_resp = requests.post(INIT_ENDPOINT, json={}, timeout=5)
    init_resp.raise_for_status()
    print("Init response:", init_resp.json())
except requests.RequestException as e:
    print("Failed to initialize:", e)
    raise

# Suppose x and y are lists of positions in centimeters
# e.g.
x = [0, 10, 20, 20]
y = [0, 5,  5,  15]

for i in range(1, len(x)):
    # Compute delta in meters
    dx = (x[i] - x[i - 1]) 
    dy = (y[i] - y[i - 1]) 

    payload = {
        "x": dx,      # meters
        "y": dy,      # meters
        "z": 0,
        "rx": 0,
        "ry": 0,
        "rz": 0,
        "open": 0     # change to 1 if you want the gripper to open
    }

    try:
        resp = requests.post(RELATIVE_ENDPOINT, json=payload, timeout=1)
        resp.raise_for_status()
        print(f"Moved by dx={dx:.3f} m, dy={dy:.3f} m → response: {resp.json()}")
    except requests.RequestException as e:
        print(f"Movement request failed at step {i}:", e)

    time.sleep(0.1)