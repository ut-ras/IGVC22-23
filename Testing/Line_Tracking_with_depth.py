import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd

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


        linedata = {}
        line_num = 1

        # Detect lines using HoughLinesP
        lines = cv2.HoughLinesP(binary_image, 1, np.pi/180, 50, maxLineGap=50)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
                
                # Get point cloud data along the line
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                line_point_cloud = []
                for x, y in zip(range(x1, x2), range(y1, y2)):
                    point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_frame.get_distance(x, y))
                    line_point_cloud.append(point)
                
                # Print and store the point cloud data
                print("Point cloud data along the line:", line_point_cloud)
                df = pd.DataFrame(line_point_cloud, columns=['x', 'y', 'z', 'depth'])
                linedata[line_num] = df
                line_num+=1
        
        for key in linedata:
            df.to_csv("line{key}.csv", index=False)

        # Show images
        cv2.imshow("Binary Image", binary_image)
        cv2.imshow("Line Detection", color_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
