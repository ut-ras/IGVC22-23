import cv2 as cv

img = cv.imread("./road_image.jpg")
# img = cv.resize(img, (0,0), fx=0.5, fy=0.5)

# ellipses = cv.ximgproc.findEllipses(img, scoreThreshold = 0.7, reliabilityThreshold = 0.5, centerDistanceThreshold = 0.05) 	
ellipses = cv.ximgproc.findEllipses(img, scoreThreshold = 0.4, reliabilityThreshold = 0.5, centerDistanceThreshold = 0.05)
print(ellipses[0][0][0])
# Draw ellipse on original image
for ellipse in ellipses:
    cv.ellipse(img, center=(int(ellipse[0][0]), int(ellipse[0][1])), axes=(int(ellipse[0][2]), int(ellipse[0][3])), angle=ellipse[0][4], startAngle=0, endAngle=360, color=(0, 255, 0), thickness=2)
    print(f"The ellipse is {ellipse}")
# Scale down the output image
scale_percent = 30  # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
output_img = cv.resize(img, dim, interpolation=cv.INTER_AREA)

# Display the output image
cv.imshow("output", output_img)
cv.waitKey(0)