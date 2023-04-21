import pyrealsense2 as rs
import numpy as np
import cv2
import serial


arduinoData = serial.Serial('com6', 9600)
# Configure depth and color streams
pipeline = rs.pipeline()
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

        # Apply threshold to get a binary image
        ret, binary_image = cv2.threshold(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY), 210, 255, cv2.THRESH_BINARY)

        left_lines = []
        right_lines = []

    

        # Detect lines using HoughLinesP
        lines = cv2.HoughLinesP(binary_image, 1, np.pi/180, 50, maxLineGap=50)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)
                if slope == 0 or np.isnan(slope) or np.isinf(slope):
                 continue
                if slope < 0:
                    left_lines.append(line)
                else:
                   right_lines.append(line)
                
                if right_lines and left_lines:
                 line = left_lines.pop(0)
                 slope_left = (y2 - y1) / (x2 - x1)

                 if slope_left == 0 or np.isnan(slope_left):
                  continue
                 intercept_left= y1-slope_left*x1

                # Get the y-coordinate of the bottom of the image
                 bottom_y = color_image.shape[0] - 1

                # Calculate the x-coordinate of the bottom left point of the line
                 if slope_left == 0 or np.isnan(slope_left) or np.isinf(intercept_left) or np.isinf(slope_left):
                  continue
                 else:
                  bottom_x_left = int((bottom_y - intercept_left) / slope_left)

                 line = right_lines.pop(0)
                 slope_right = (y2 - y1) / (x2 - x1)


                 if slope_right == 0 or np.isnan(slope_right):
                  continue
                 intercept_right = y1-slope_right*x1

                # Calculate the x-coordinate of the bottom right point of the line
                 if slope_right == 0 or np.isnan(slope_right) or np.isnan(intercept_right) or np.isinf(intercept_right) or np.isinf(slope_right):
                   continue
                 else:
                   bottom_x_right = int((bottom_y - intercept_right) / slope_right)
                 
                 if bottom_x_left is not None and bottom_x_right is not None:
                  true_middle = (bottom_x_left + bottom_x_right)/2

                  vehicle_offset = color_image.shape[1]/2 - true_middle

                  if vehicle_offset < 0:
                     print("Turn right")
                     cmd="Turn right"+'\r'
                  if vehicle_offset > 0:
                     print("Turn left")
                     cmd="Turn left"+'\r'

                 arduinoData.write(cmd.encode())



                cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
                
                # Get point cloud data along the line
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                line_point_cloud = []
                for x, y in zip(range(x1, x2), range(y1, y2)):
                    point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_frame.get_distance(x, y))
                    line_point_cloud.append(point)
                
                # Print and store the point cloud data
           #     print("Point cloud data along the line:", line_point_cloud)
           # Find the closest point along the detected line
#                 min_distance = float('inf')
#                 closest_point = None
#                 for x, y in zip(range(x1, x2), range(y1, y2)):
#                     distance = depth_frame.get_distance(x, y)
#                     if distance < min_distance:
#                         min_distance = distance
#                         closest_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], distance)

# # Print the depth value of the closest point
#                 print("Depth of closest point:", min_distance)

        # Show images
        cv2.imshow("Binary Image", binary_image)
        cv2.imshow("Line Detection", color_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
