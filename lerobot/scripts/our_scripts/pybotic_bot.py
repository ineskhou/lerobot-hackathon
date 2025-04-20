from pybotics.robot import Robot
from pybotics.predefined_models import ur10
import numpy as np
from draw import LiveDrawer
from draw_to_coordinates import MousePathRecorder


robot_parameters = np.array([
    [0,     0,  0,    0],     # Joint 1: [theta, d, a, alpha]
    [0,  11.5,  0,  np.pi/2], # Joint 2
    [0,     0,  12,     0],    # Joint 3
    [0,     0,  11,     0],    # Joint 4
    [0,     7,  0,  np.pi/2]  # Joint 5
      
    ])

human_parameters = np.array([
        [0,     0,  0,    0],     # Joint 1: [theta, d, a, alpha]
        [0,  11.5,  0,  np.pi/2], # Joint 2
        [0,     0,  12,     0],    # Joint 3
        [0,     0,  11,     0],    # Joint 4
        [0,     7,  0,  np.pi/2]  # Joint 5
    ])


def forward_kinematics(pos):
    robot = Robot.from_parameters(human_parameters)

    #where am I with the given joint angles 
    joint_angles = np.deg2rad([5,5,5,5])
    pose = robot.fk(joint_angles)
    x = pose[0,3]
    y = pose[1,3]
    z = pose[2,3]
    print(pose)
    print("the x value is ", x , "mm")
    print("the y value is " , "mm")
    print("the z value is ", z , "mm")

    return x, y, z

def inverse_kinematics_follower():
    robot = Robot.from_parameters(robot_parameters)


    