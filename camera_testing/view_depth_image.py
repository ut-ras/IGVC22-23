import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import threading

bridge = CvBridge()
max_depth = 2500

# Global variables to store images
depth_image = None
color_image = None
lock = threading.Lock()

def callback_function(data):
    global depth_image, max_depth
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv_image = cv_image.copy()
        valid_mask = (cv_image > 0) & np.isfinite(cv_image)
        cv_image[~valid_mask] = max_depth
        cv_image = np.clip(cv_image, 0, max_depth)
        depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_8u = depth_normalized.astype(np.uint8)
        with lock:
            depth_image = depth_8u
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def image_callback(msg):
    global color_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        with lock:
            color_image = cv_image
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

if __name__ == "__main__":
    rospy.init_node("image_subscriber")
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_function)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    while not rospy.is_shutdown():
        with lock:
            if depth_image is not None:
                cv2.imshow("Depth Image", depth_image)
            if color_image is not None:
                cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Quit OpenCV window")
    cv2.destroyAllWindows()
