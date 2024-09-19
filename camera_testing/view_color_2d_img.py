#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    # Initialize the CvBridge class
    bridge = CvBridge()

    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        print(cv_image.shape)
        
        # Display the image
        cv2.imshow("Camera Image", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Quit OpenCV window")
            cv2.destroyAllWindows()
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('camera_image_subscriber', anonymous=True)
    
    # Subscribe to the /camera/color/image_raw topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # Keep the program running until manually interrupted
    rospy.spin()

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
