import tkinter as tk

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

if __name__ == "__main__":
    x_coords = [50, 100, 150, 200, 250]
    y_coords = [60, 110, 160, 210, 260]
    drawer = LiveDrawer(x_coords, y_coords, delay=500)
    drawer.run()
