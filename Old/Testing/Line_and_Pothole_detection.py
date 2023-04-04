import cv2
import pyrealsense2 as rs
import numpy as np

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the depth image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image to get binary image
        _, binary_image = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

        # Detect white lines and circles using Hough lines and Hough circles
        lines = cv2.HoughLinesP(binary_image, 1, np.pi/180, 50, maxLineGap=50)
        circles = cv2.HoughCircles(binary_image, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(color_image, (x, y), r, (0, 255, 0), 2)

        # Show the image and binary image in two separate windows
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Binary Image", binary_image)

        # Get the point cloud data and print it
        points = rs.points(depth_frame)
        print("Point Cloud Data:")
        print(points.get_vertices())

        # Break if escape key is pressed
        key = cv2.waitKey(1)
        if key == 27:
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows
