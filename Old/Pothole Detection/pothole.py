import cv2
import numpy as np

# Load the image
# img = cv2.imread("road_image.jpg")
img = cv2.imread("white dots.jpg")

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur
blur = cv2.GaussianBlur(gray, (5, 5), 0)

# Apply adaptive threshold
thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

# Find contours
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

# Fit ellipses to contours and draw ellipses on original image
for i in range(len(contours)):
    # Ignore small contours and contours with no parent
    if cv2.contourArea(contours[i]) < 2500 or hierarchy[0][i][3] != -1:
        continue

    # Fit ellipse to contour
    ellipse = cv2.fitEllipseAMS(contours[i])

    # Check if center of ellipse is within a certain range of the image width
    center_x = ellipse[0][0]
    img_width = img.shape[1]
    x_range = img_width // 3  # Change this value to adjust the range
    if center_x < x_range or center_x > img_width - x_range:
        continue

    # Check if the width of the ellipse is greater than one-third of the image width
    if ellipse[1][0] > img_width // 3:
        continue

    if abs(ellipse[2]) < 80:
        continue

    # Draw ellipse on original image
    cv2.ellipse(img, ellipse, (0, 255, 0), 2)

# Scale down the output image
scale_percent = 50  # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
output_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# Display the output image
cv2.imshow("output", output_img)
cv2.waitKey(0)
