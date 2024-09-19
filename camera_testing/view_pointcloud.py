import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct
import cv2
import sys

x_min = None

def unpack_rgb(rgb_value):
    packed = struct.pack('f', rgb_value)
    integer = struct.unpack('I', packed)[0]
    r = (integer >> 16) & 0x0000ff
    g = (integer >> 8) & 0x0000ff
    b = (integer) & 0x0000ff
    return r, g, b

def point_cloud_to_image(points, image_width=500, image_height=1000):
    global x_min, x_max, y_min, y_max

    x_vals = points[:, 0]
    y_vals = points[:, 1]
    z_vals = points[:, 2]


    # rgb_vals = points[:, 3] # rgb not needed anymore

    # x_min, x_max = -0.5, 0.5
    # y_min, y_max = -1, 1
    x_min, x_max = np.min(x_vals), np.max(x_vals)
    y_min, y_max = np.min(y_vals), np.max(y_vals)
    x_scale = (image_width - 1) / (x_max - x_min)
    y_scale = (image_height - 1) / (y_max - y_min)
    
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    
    for x, y, z in zip(x_vals, y_vals, z_vals):
        
        x_img = int((x - x_min) * x_scale)
        y_img = int((y - y_min) * y_scale)
        
        if 0 <= x_img < image_width and 0 <= y_img < image_height:
            # Map z (depth) to a grayscale value (255 is close, 0 is far)
            depth_value = 255 - int(np.clip(z, 0, 5) * 255 / 5)  # Adjust the scale as necessary
            image[y_img, x_img] = depth_value
    
    return image

def print_data(data):
    point_generator = pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = np.array(list(point_generator))
    image = point_cloud_to_image(points)
    return image

def callback_function(msg):
    rospy.loginfo("Message received by subscriber")
    image = print_data(msg)
    
    cv2.imshow('Point Cloud Image', image)
    key = cv2.waitKey(3)
    if key == ord('q'):
        rospy.signal_shutdown("pressed q")

if __name__ == "__main__":
    rospy.init_node("depth_image_subscriber")
    rospy.loginfo("PointCloud subscriber launched")

    sub = rospy.Subscriber("/converted_point_cloud", PointCloud2, callback_function)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    rospy.loginfo("Exiting program")
    rospy.sleep(1)
    sys.exit()
