# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "phosphobot",
# ]
# ///
from phosphobot.api.client import PhosphoApi
from pybotics.robot import Robot

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
    client = PhosphoApi(base_url="http://localhost:80")
    time.sleep(1)
    response = client.control.move_init()
    print(response)

    # Convert absolute x, y (in cm) to relative deltas (in m)
    for i in range(1, len(x)):
        dx = (x[i]/7000)  # cm to meters
        dy = (y[i]/7000 ) # cm to meters
      
        client.control.move_to_absolute_position(
            x=0,
            y=dy,
            z=dx,
            rx=0,
            ry=0,
            rz=0,
            open=0  # change to 1 if you want the gripper to open
        )
        print(f"Moved by dx={dx:.3f} m, dy={dy:.3f} m")
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



    



