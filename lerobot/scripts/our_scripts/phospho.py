# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "phosphobot",
# ]
# ///
from phosphobot.api.client import PhosphoApi
from pybotics.robot import Robot
import requests

from draw import LiveDrawer
import numpy as np

import time
import numpy as np
from draw_to_coordinates import MousePathRecorder


robot_parameters = np.array([
    [0,     0,  0,    0],     # Joint 1: [theta, d, a, alpha]
    [0,  11.5,  0,  np.pi/2], # Joint 2
    [0,     0,  12,     0],    # Joint 3
  #  [0,     0,  11,     0],    # Joint 4
    [0,     7,  0,  np.pi/2]  # Joint 5
      
    ])

human_parameters = np.array([
        [0,     0,  0,    0],     # Joint 1: [theta, d, a, alpha]
        [0,  11.5,  0,  np.pi/2], # Joint 2
        [0,     0,  12,     0],    # Joint 3
#        [0,     0,  11,     0],    # Joint 4
        [0,     7,  0,  np.pi/2]  # Joint 5
    ])


def draw_from_follow():
    # Connect to the phosphobot server
    client = PhosphoApi(base_url="http://localhost:80")


    # Need to wait for the cameras to initialize
    time.sleep(1)

    print("got to the loopx!!")
    robot = Robot.from_parameters(human_parameters)


    i = 0
    xv = []
    yv = []
    while i < 1000:
        i+=1
        print(i)
        # Get the robot state
        state = client.control.read_joints()

        
        inputs = {"state": np.array(state.angles_rad)}
        #print(inputs)
        print([state.angles_rad[0], state.angles_rad[1], state.angles_rad[2], state.angles_rad[4]])
        pose = robot.fk([state.angles_rad[0], state.angles_rad[1], state.angles_rad[2], state.angles_rad[4]])
        x = pose[0,3] 
        y = pose[1,3]
        z = pose[2,3]

        xv.append(x*30)
        yv.append(y*30)
        # print("ACCORDING TO RAW SCORE")
        # print("the x value is ", x , "mm")
        # print("the y value is ", y , "mm")
        # print("the z value is ", z , "mm")


    print("xv = ", xv)
    print("yv = ", yv)
    drawer = LiveDrawer(xv, yv, delay=1)
    drawer.run()


def draw_from_points(x, y):
        
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


    for i in range(1, len(x)):
        # Compute delta in meters
        dx = (x[i] - x[i - 1]/50) 
        dy = (y[i] - y[i - 1]/50) 

        payload = {
            "x": 0,      # meters
            "y": dy,      # meters
            "z": 0,
            "rx": 0,
            "ry": 0,
            "rz": dx,
            "open": 0     # change to 1 if you want the gripper to open
        }

        try:
            resp = requests.post(RELATIVE_ENDPOINT, json=payload, timeout=1)
            resp.raise_for_status()
            print(f"Moved by dx={dx:.3f} m, dy={dy:.3f} m → response: {resp.json()}")
        except requests.RequestException as e:
            print(f"Movement request failed at step {i}:", e)

        time.sleep(0.1)



if __name__ == "__main__":
    recorder = MousePathRecorder()
    x_positions, y_positions = recorder.run()
    draw_from_points(x_positions, y_positions)

    # client = PhosphoApi(base_url="http://localhost:80")
    # client.control.move_relative(
    #         x=1,
    #         y=1,
    #         z=10,
    #         rx=0,
    #         ry=0,
    #         rz=0,
    #         open=0  # change to 1 if you want the gripper to open
    #     )
    # time.sleep(2)


    #draw_from_follow()



    



