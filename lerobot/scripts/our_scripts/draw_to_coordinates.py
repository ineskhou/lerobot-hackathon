import tkinter as tk

class MousePathRecorder:
    def __init__(self, width=500, height=500):
        self.x_coords = []
        self.y_coords = []

        self.drawing = False
        self.finished = False

        self.root = tk.Tk()
        self.root.title("Mouse Path Recorder (Press Space to Start, Enter to Stop)")

        self.canvas = tk.Canvas(self.root, width=width, height=width, bg="white")
        self.canvas.pack()

        self.instructions = tk.Label(self.root, text="Press [Space] to start drawing, [Enter] to finish.")
        self.instructions.pack()

        # Bind events
        self.canvas.bind("<Motion>", self.track_mouse)
        self.root.bind("<space>", self.start_drawing)
        self.root.bind("<Return>", self.finish)

    def start_drawing(self, event):
        self.drawing = True
        self.instructions.config(text="Drawing... Press [Enter] to stop.")

    def track_mouse(self, event):
        if self.drawing:
            x, y = event.x, event.y
            self.x_coords.append(x)
            self.y_coords.append(y)
            self.canvas.create_oval(x-1, y-1, x+1, y+1, fill="black", outline="")

    def finish(self, event):
        self.drawing = False
        self.finished = True
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        
        return self.x_coords, self.y_coords


def main():
    recorder = MousePathRecorder()
    x_positions, y_positions = recorder.run()
    print("X Coordinates:", x_positions)
    print("Y Coordinates:", y_positions)

if __name__ == "__main__":
    main()
