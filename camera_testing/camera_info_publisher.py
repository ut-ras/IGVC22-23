#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node("camera_info_publisher", anonymous=True)
    info_pub = rospy.Publisher("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Define camera intrinsics (example values; replace with actual calibration data)
    camera_info = CameraInfo()
    camera_info.width = 640
    camera_info.height = 480
    camera_info.K = [607.783, 0.0, 323.413, 0.0, 606.473, 245.606, 0.0, 0.0, 1.0]
    camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.P = [607.783, 0.0, 323.413, 0.0, 0.0, 606.473, 245.606, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info.distortion_model = "plumb_bob"

    while not rospy.is_shutdown():
        try:
            info_pub.publish(camera_info)
            rospy.loginfo("Published camera info")
        except rospy.ROSInterruptException:
            break

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
