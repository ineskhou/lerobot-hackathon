import torch
import os
from huggingface_hub import HfApi
from pathlib import Path
from diffusers.utils import load_image
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import cv2
#import pytesseract
from controlnet_aux import LineartDetector

from diffusers import (
    StableDiffusionPipeline, 
    ControlNetModel,
    StableDiffusionControlNetPipeline,
    UniPCMultistepScheduler,
)

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



# 1. Generate image from prompt
pipe = StableDiffusionPipeline.from_pretrained("runwayml/stable-diffusion-v1-5", torch_dtype=torch.float16)
#pipe.enable_model_cpu_offload()

lineart_detector = LineartDetector.from_pretrained("lllyasviel/Annotators")

print("Loaded everything")

def generate_image(prompt):
    prompt += "make very very simple, with no background (blank white space). Primitive shapes, like circles, squares, triangles, etc."
    generator = torch.manual_seed(0)
    generated = pipe(prompt, num_inference_steps=30, generator=generator).images[0]
    lineart_image = lineart_detector(generated)
    return lineart_image



def trace_lineart(img, min_contour_length=100):
    """
    Given a lineart image, return a list of (X, Y) traces approximating the contours.
    Each trace is a list of connected points, simulating pencil strokes.
    """
    # Load and convert to grayscale
    if(isinstance(img, str)):
        img = cv2.imread(img, cv2.IMREAD_GRAYSCALE).astype(np.uint8)
    elif(isinstance(img, Image.Image)):
        img = np.array(img).astype(np.uint8)
    elif(isinstance(img, np.ndarray)):
        img = img.astype(np.uint8)
    else:
        raise Exception("Invalid image type")

    # Invert the image: lineart should be white on black for contour detection
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    if(len(binary.shape) > 2):
        binary = binary[:, :, 0]
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Process and simplify contours
    traces = []
    for cnt in contours:
        if len(cnt) >= min_contour_length:
            traces.append(cnt[:,0,:])

    return traces, binary, contours


def extract_coordinates(traces, binary, contours):
    """
    Given a list of traces, return a list of (X, Y) coordinates.
    """
    rows, cols = np.where(binary > 0)
    return cols, rows




if __name__ == "__main__":
    prompt = input("Enter what you want to draw: ")
    img = generate_image(prompt)
    traces, binary, contours = trace_lineart(img)
    x, y = extract_coordinates(traces, binary, contours)
    num_points = len(x)
    total_time = 5 * 1000
    delay = total_time / num_points
    drawer = LiveDrawer(x, y, delay=int(delay))
    drawer.run()