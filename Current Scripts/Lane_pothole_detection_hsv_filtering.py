# CURRENT LANE DETECTION SCRIPT 

# dependencies - opencv-python and numpy
# will probably need to replace cv2_imshow with cv2.imshow

import cv2
from PIL import Image as im
from google.colab.patches import cv2_imshow
import numpy as np
import matplotlib.pyplot as plt

# FINAL WORKING attempt at just seeing road
# image = cv2.imread(f'/content/1492626166147797438_0/1.jpg')
image_path = '/content/pothole_track_drawing.png'
image = cv2.imread(image_path)
cv2_imshow(image)
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
blurred_image = cv2.GaussianBlur(hsv_image, (5, 5), 10)  # You can adjust the kernel size (e.g., (5, 5)) and the sigma value (e.g., 0) as needed
lower_road = np.array([0, 0, 50])
upper_road = np.array([179, 50, 200])

road_mask = cv2.inRange(blurred_image, lower_road, upper_road)
filtered_image = cv2.bitwise_and(blurred_image, image, mask=road_mask)
# cv2_imshow(filtered_image)

# invert image
inverse_mask = cv2.bitwise_not(road_mask)
cv2_imshow(inverse_mask)
plt.imshow(inverse_mask)

from PIL import Image as im


def convert_to_bw(image_path, threshold=128):
    # Open the image
    img = im.fromarray(inverse_mask)

    # Get the size of the image
    width, height = img.size

    # Initialize an array to hold the black and white pixels
    white_pixels = []

    # Iterate through each pixel
    for y in range(height):
        for x in range(width):
            # Get the pixel value
            pixel = img.getpixel((x, y))

            # Convert pixel to black or white based on threshold
            if pixel > threshold:
                white_pixels.append((x, y))
    return white_pixels

# Example usage


white_pixels = convert_to_bw(image_path)

# Print the first 10 black and white pixels
with open("white_pixels.txt", 'w') as f:
  f.write(str(white_pixels))

from PIL import Image as im


def convert_to_bw(image_path, threshold=128):
    # Open the image
    img = im.fromarray(inverse_mask)

    # Get the size of the image
    width, height = img.size

    # Initialize an array to hold the black and white pixels
    white_pixels = []

    # Iterate through each pixel
    for y in range(height):
        for x in range(width):
            # Get the pixel value
            pixel = img.getpixel((x, y))

            # Convert pixel to black or white based on threshold
            if pixel > threshold:
                white_pixels.append((x, y))
    return white_pixels

# Example usage


white_pixels = convert_to_bw(image_path)

# Print the first 10 black and white pixels
with open("white_pixels.txt", 'w') as f:
  f.write(str(white_pixels))
