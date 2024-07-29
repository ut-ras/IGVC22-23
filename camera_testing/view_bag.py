import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def callback_function(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    cv2.imshow("Image Window", cv_image)
    key = cv2.waitKey(3)
    if key == ord('q'):
        rospy.signal_shutdown("pressed q")

if __name__ == "__main__":
    rospy.init_node("image_subscriber")

    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback_function)

    rospy.spin()
    cv2.destroyAllWindows()