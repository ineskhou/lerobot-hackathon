from pybotics.robot import Robot
from pybotics.predefined_models import ur10
import numpy as np

robot_parameters = np.array([
    [0,     0,  0,    0],     # Joint 1: [theta, d, a, alpha]
    [0,  11.5,  0,  np.pi/2], # Joint 2
    [0,     0,  12,     0],    # Joint 3
    [0,     0,  11,     0],    # Joint 4
    [0,     7,  0,  np.pi/2]  # Joint 5
])

human_parameters = np.array([
    [0,   100,  0,    0],     # Joint 1: [theta, d, a, alpha]
    [0,     0, 50,  np.pi/2], # Joint 2
    [0,     0, 50,      0],   # Joint 3
    [0,     0,  0, -np.pi/2]  # Joint 4
    [0,     0,  0,  0]        # Joint 5
])

robot = Robot.from_parameters(robot_parameters)

#where am I with the given joint angles 
joint_angles = np.deg2rad([5,5,5,5])
pose = robot.fk(joint_angles)
x = pose[0,3]
y = pose[1,3]
z = pose[2,3]
print(pose)
print(x , "mm")
print(y , "mm")
print(z , "mm")
#from the pose we want the x y z

print("done forward")
#inverse
solved_joints = robot.ik(pose)
print("go to this joint", np.rad2deg(solved_joints))