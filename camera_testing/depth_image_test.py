import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


bridge = CvBridge()
color_img = None
color_img_filtered = None
filtered_depth_img = None
depth_img = None
filtered_depth_img_view = None
max_depth = 2500

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
        og_msg = depth_image
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
        

        # Convert back to ROS Image message
        filtered_msg = bridge.cv2_to_imgmsg(filtered_depth_img, encoding="passthrough")
        filtered_msg.header = og_msg.header  # Retain original message header
        # Publish the filtered depth point cloud
        filtered_image_pub.publish(filtered_msg)
        rospy.loginfo("Published filtered depth image")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == "__main__":
    rospy.init_node("image_subscriber")

    color_subscriber = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_subscriber = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    filtered_image_pub = rospy.Publisher(
        "/filtered_depth/image_raw", Image, queue_size=10
    )


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