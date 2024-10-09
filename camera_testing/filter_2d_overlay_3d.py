#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf.transformations
import struct
import cv2
import sys
from matplotlib import pyplot as plt
import argparse

def filter_img(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred_image = cv2.GaussianBlur(hsv_image, (15, 15), 20)  # Adjust kernel size and sigma value as needed
    lower_road = np.array([0, 0, 50])
    upper_road = np.array([179, 50, 200])

    road_mask = cv2.inRange(blurred_image, lower_road, upper_road)

    # Invert image
    inverse_mask = cv2.bitwise_not(road_mask)
    return inverse_mask

def image_callback(msg):
    # Initialize the CvBridge class
    rospy.loginfo("Message received by subscriber")
    bridge = CvBridge()

    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        filtered_image = filter_img(cv_image)
        
        # Display the image
        cv2.imshow("Camera Image", filtered_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Quit OpenCV window")
            cv2.destroyAllWindows()
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

# Add the static transform broadcaster function
def broadcast_static_transform(camera_height, camera_angle):
    # Initialize the broadcaster
    br = tf2_ros.StaticTransformBroadcaster()

    # Create the TransformStamped message
    transform = geometry_msgs.msg.TransformStamped()

    # Set the time
    transform.header.stamp = rospy.Time.now()

    # Set the parent and child frame IDs
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_link"

    # Set translation (camera_height above the ground)
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = camera_height

    # Set rotation (camera_angle tilt downwards)
    quat = tf.transformations.quaternion_from_euler(-camera_angle, 0, 0)
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    # Send the transform
    br.sendTransform(transform)


def main():
    # Initialize the ROS node
    rospy.init_node('camera_image_subscriber', anonymous=True)
    rospy.loginfo("Color Image subscriber launched")

    # Get the camera parameters
    camera_height = rospy.get_param("camera_height", 1.0)  # Default to 1 meter
    camera_angle = rospy.get_param("camera_angle", -0.7854)  # Default to -0.7854 radians (45 degrees downwards)

    # Broadcast the static transform
    broadcast_static_transform(camera_height, camera_angle)
    
    # Subscribe to the /camera/color/image_raw topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # Keep the program running until manually interrupted
    rospy.spin()

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
