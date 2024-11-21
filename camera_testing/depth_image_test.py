import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import sensor_msgs.point_cloud2 as pc2
from concurrent.futures import ThreadPoolExecutor
import os


bridge = CvBridge()
color_img = None
color_img_filtered = None
filtered_depth_img = None
depth_img = None
filtered_depth_img_view = None
max_depth = 2500

# Camera intrinsics (replace with your actual values)
fx = 607.7837524414062  # Focal length in pixels
fy = 606.473876953125  # Focal length in pixels
cx = 323.4130554199219  # Principal point (center of image)
cy = 245.60577392578125  # Principal point (center of image)

num_threads = os.cpu_count()  # Number of threads for parallel processing


def process_chunk(chunk):
    """Process a chunk of the image to compute point cloud."""
    start_row, end_row, depth_chunk, mask_chunk = chunk
    v, u = np.meshgrid(
        np.arange(depth_chunk.shape[1]), np.arange(start_row, end_row)
    )  # Compute pixel indices

    z = depth_chunk / 1000.0  # Convert mm to meters
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    valid_points = mask_chunk  # Apply the mask to filter points
    points = np.column_stack((x[valid_points], y[valid_points], z[valid_points]))
    return points

def parallel_depth_to_pointcloud(depth_img, mask):
    """Convert filtered depth image to a PointCloud2 message in parallel."""
    height, width = depth_img.shape

    # Create chunks for parallel processing
    chunk_size = height // num_threads
    chunks = [
        (i * chunk_size, (i + 1) * chunk_size if i < num_threads - 1 else height,
         depth_img[i * chunk_size:(i + 1) * chunk_size],
         mask[i * chunk_size:(i + 1) * chunk_size])
        for i in range(num_threads)
    ]

    # Process chunks in parallel
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        results = executor.map(process_chunk, chunks)

    # Combine results from all threads
    points = np.vstack(results)

    # Create the PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera_link"  # Replace with your frame ID

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    point_cloud = pc2.create_cloud(header, fields, points)
    return point_cloud

def filter_img(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred_image = cv2.GaussianBlur(hsv_image, (15, 15), 20)  # Adjust kernel size and sigma value as needed
    lower_road = np.array([0, 0, 50])
    upper_road = np.array([179, 50, 200])

    road_mask = cv2.inRange(blurred_image, lower_road, upper_road)

    # Invert image
    inverse_mask = cv2.bitwise_not(road_mask)
    return inverse_mask

def callback(color_image, depth_image):
    global color_img, color_img_filtered, depth_img, depth_img_view, filtered_depth_img, filtered_depth_img_view, max_depth
    try:
        # filter colored image, same as viewing
        color_img = bridge.imgmsg_to_cv2(color_image, "bgr8")
        color_img_filtered = filter_img(color_img)

        # process depth image
        depth_img = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        
        # masking, we can throw out invalid points when converting to PointCloud
        valid_mask = (depth_img > 0) & np.isfinite(depth_img)
        
        # for viewing
        depth_img_view = depth_img.copy()
        depth_img_view[~valid_mask] = max_depth
        depth_img_view = np.clip(depth_img_view, 0, max_depth)
        depth_img_view = cv2.normalize(depth_img_view, None, 0, 255, cv2.NORM_MINMAX)
        depth_img_view = depth_img_view.astype(np.uint8)

        filtered_depth_img = np.where(color_img_filtered != 0, depth_img, float('NaN'))
        filtered_depth_img_view = np.where(color_img_filtered != 0, depth_img_view, 255).astype(np.uint8) 
        
        # Publish the filtered depth point cloud
        point_cloud_msg = parallel_depth_to_pointcloud(filtered_depth_img, valid_mask)
        pointcloud_pub.publish(point_cloud_msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == "__main__":
    rospy.init_node("image_subscriber")

    color_subscriber = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_subscriber = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    # Publisher for the filtered point cloud
    pointcloud_pub = rospy.Publisher("/filtered_depth/pointcloud", PointCloud2, queue_size=10)


    ts = message_filters.TimeSynchronizer([color_subscriber, depth_subscriber], 10)
    ts.registerCallback(callback)
    while not rospy.is_shutdown():
        if color_img is not None:
            cv2.imshow("Color Image", color_img)
        if depth_img is not None:
            cv2.imshow("Depth Image", depth_img_view)
        if color_img_filtered is not None:
            cv2.imshow("Color Image Filtered", color_img_filtered)
        if filtered_depth_img is not None:
            cv2.imshow("Filtered Depth Image", filtered_depth_img_view)
        if depth_img is not None:
            cv2.imshow("OG Depth Image", depth_img)
        if filtered_depth_img is not None:
            cv2.imshow("OG Filtered Depth Image", filtered_depth_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Quit OpenCV window")
    cv2.destroyAllWindows()