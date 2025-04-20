import math
import time
import numpy as np

def angle2rad(degrees):
    return degrees * np.pi / 180

class robot:
    def __init__(self, theta_def, lengths):

        self.seconds = time.time()
        self.L = lengths#link length 
        self.theta_def = theta_def

    def get_angles(self):
        #bottom motor first!!!!
        return [0, 0, 0, 0]

    def forward_kinematics_4dof(self, theta1, theta2, theta3, theta4, L1, L2, L3, L4):
        # Convert angles to radians
        L1, L2, L3, L4, = self.L[0], self.L[1], self.L[2], self.L[3]

        t1 = angle2rad(theta1)
        t2 = angle2rad(theta2)
        t3 = angle2rad(theta3)
        t4 = angle2rad(theta4)

        # Denavit-Hartenberg approach (simple, typical 4-DOF arm)
        # This assumes the first joint rotates about Z (base), the rest about Y
        # Adjust for your robot's actual configuration if needed
        # first joint should be the bottom joint since its the only one that rotates around Z 
        
        #code taken by stack overflow 
        # Position of end effector
        x = (L2 * np.cos(t2) + L3 * np.cos(t2 + t3) + L4 * np.cos(t2 + t3 + t4)) * np.cos(t1)
        y = (L2 * np.cos(t2) + L3 * np.cos(t2 + t3) + L4 * np.cos(t2 + t3 + t4)) * np.sin(t1)
        z = L1 + L2 * np.sin(t2) + L3 * np.sin(t2 + t3) + L4 * np.sin(t2 + t3 + t4)
        return x, y, z
 
    def record_arm(self):
        print("you have 5 seconds to place the arm at origin point")
        time.sleep(5)
        print("Starting to record motion")

        seconds = time.time()

        x_coords = []
        y_coords = []

        while(True):
            angles = self.get_angles()
            x, y, z = self.forward_kinematics_4dof(angles[0], angles[1], angles[2], angles[3])

            if (10 < z != 10):
                print("not on plane")
            else:
                x_coords.append(x)
                y_coords.append(y)
                print(f"End effector position: x = {x:.2f}, y = {y:.2f}")

    if __name__ == "__main__":
        print("starting")
        theta_def = [0,0,0,0]
        lengths = [0,0,0,0]
        robot = robot(theta_def, lengths)

        robot.record_arm(5)
        print("done")
