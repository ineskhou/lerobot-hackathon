# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "phosphobot",
# ]
# ///
from phosphobot.camera import AllCameras
from phosphobot.api.client import PhosphoApi
from phosphobot.am import ACT
from pybotics.robot import Robot
from pybotics.predefined_models import ur10

from draw import LiveDrawer
import numpy as np

import time
import numpy as np

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

    xv.append(x*15)
    yv.append(y*15)
    # print("ACCORDING TO RAW SCORE")
    print("the x value is ", x , "mm")
    print("the y value is ", y , "mm")
    print("the z value is ", z , "mm")



drawer = LiveDrawer(xv, yv, delay=1)
drawer.run()





    



