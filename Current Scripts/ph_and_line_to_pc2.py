import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


rospy.init_node("point_cloud_publisher")

pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

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

        img = color_image

        # Apply threshold to get a binary image
        ret, binary_image = cv2.threshold(cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY)

        # Detect lines using HoughLinesP
        lines = cv2.HoughLinesP(binary_image, 1, np.pi/180, 50, maxLineGap=50)
        line_point_cloud = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
                
                # Get point cloud data along the line
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                for x, y in zip(range(x1, x2), range(y1, y2)):
                    point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_frame.get_distance(x, y))
                    line_point_cloud.append(point)

                # Print and store the point cloud data
                print("Point cloud data along the line:", line_point_cloud)
        
        
        ## Detect potholes
        ## Comment out if facing problems
        ellipses = cv2.ximgproc.findEllipses(img, scoreThreshold = 0.4, reliabilityThreshold = 0.5, centerDistanceThreshold = 0.05)
        if ellipses is not None:
            for ellipse in ellipses:
                x1, y1, a, b, radius, score = ellipse[0]
                x = np.linspace(x - radius, x + radius, num=1000)
                y = np.linspace(y - radius, y + radius, num=1000)
                xx, yy = np.meshgrid(x, y)

                ellipse_mask = ((xx - x) / a)**2 + ((yy - y) / b)**2 <= 1
                x_inside_ellipse = xx[ellipse_mask]
                y_inside_ellipse = yy[ellipse_mask]

                for x, y in zip(range(x_inside_ellipse, y_inside_ellipse)):
                    point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_frame.get_distance(x, y))
                    line_point_cloud.append(point)            


        points = []

        try:    
            for i in range(len(line_point_cloud)):
                x = line_point_cloud[i][0]
                y = line_point_cloud[i][1]
                z = line_point_cloud[i][2]
                pt = [x, y, z, 0]
                rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
                pt[3] = rgb
                points.append(pt)
        except:
            points.append([1, 2, 3, 4])

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 16, PointField.UINT32, 1),
            ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points)

        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        # rospy.sleep(1.0)

        # Show images
        # cv2.imshow("Binary Image", binary_image)
        # cv2.imshow("Line Detection", color_image)
        # key = cv2.waitKey(1)
        # if key & 0xFF == ord('q') or key == 27:
        #     break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
