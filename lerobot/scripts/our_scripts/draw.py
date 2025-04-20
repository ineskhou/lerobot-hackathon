import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt

class LiveDrawer:
    def __init__(self, x_coords, y_coords, width=500, height=500, delay=500):
        self.x_coords = x_coords
        self.y_coords = y_coords
        self.index = 0
        self.delay = delay  # in milliseconds
        self.prev_x = None
        self.prev_y = None

        self.root = tk.Tk()
        self.root.title("Live Coordinate Drawer with Lines")

        self.canvas = tk.Canvas(self.root, width=width, height=height, bg="white")
        self.canvas.pack()

        self.update_point()  # start the update loop

    def update_point(self):
        if self.index < len(self.x_coords):
            x = self.x_coords[self.index]
            y = self.y_coords[self.index]

            # Draw a red dot
            self.canvas.create_oval(x-5, y-5, x+5, y+5, fill='red')

            # If there is a previous point, draw a line to the current point
            if self.prev_x is not None and self.prev_y is not None:
                self.canvas.create_line(self.prev_x, self.prev_y, x, y, fill='blue', width=2)

            # Update previous point
            self.prev_x = x
            self.prev_y = y

            self.index += 1
            self.root.after(self.delay, self.update_point)

    def add_point(self, x, y):
        self.canvas.create_oval(x-5, y-5, x+5, y+5, fill='red')

        if self.prev_x is not None and self.prev_y is not None:
            self.canvas.create_line(self.prev_x, self.prev_y, x, y, fill='blue', width=2)

        self.prev_x = x
        self.prev_y = y

        # Do not schedule another update loop here
        self.index += 1

    def run(self):
        self.root.mainloop()


def generate_circle_coords(radius=1.0, num_points=100, center=(50,50)):
    # Theta goes from 0 to 2π, but negative direction for clockwise
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    
    # Circle center is at (0, radius) so that (0, 0) is topmost point
    x = radius * np.sin(theta) + center[0]
    y = radius * (1 - np.cos(theta)) + center[1] # shifted down so (0, 0) is at top

    return x, y


if __name__ == "__main__":
    #x_coords = [50, 100, 150, 200, 250]
    #y_coords = [60, 110, 160, 210, 260]
    num_points = 100
    total_time = 5 * 1000
    delay = total_time / num_points
    x, y = generate_circle_coords(radius=50, num_points=num_points, center=(250,20))
    drawer = LiveDrawer(x, y, delay=int(delay))
    drawer.run()
