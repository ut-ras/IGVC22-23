# import required libraries
import cv2
import numpy as np
# load the input image
img = cv2.imread('test_case_2.jpg')

# convert the input image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# apply thresholding to convert grayscale to binary image
ret,thresh = cv2.threshold(gray,210,255,0)

lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 150,maxLineGap=50)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 5)

# Display the Binary Image
cv2.imshow("Binary Image", thresh)
cv2.imshow("Line Detection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()