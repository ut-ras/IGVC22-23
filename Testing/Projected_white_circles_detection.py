import cv2
import numpy as np

# Load the image
img = cv2.imread("image.webp")

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian Blur to reduce noise
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Threshold the image to create a binary image
_, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

# Use the Hough Circle Transform to detect circles in the binary image
circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

# Check if any circles were detected
if circles is not None:
    # Convert the (x, y) coordinates and radius of the circles to integers
    circles = np.round(circles[0, :]).astype("int")

    # Loop over the circles
    for (x, y, r) in circles:
        # Draw the circle in the output image
        cv2.circle(img, (x, y), r, (0, 255, 0), 4)

# Show the output image
cv2.imshow("Detected Circles", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
