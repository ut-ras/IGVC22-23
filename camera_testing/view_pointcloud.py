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

def point_cloud_to_image(points, image_width=640, image_height=480):
    global x_min, x_max, y_min, y_max

    x_vals = points[:, 0]
    y_vals = points[:, 1]
    z_vals = points[:, 2]


    # rgb_vals = points[:, 3] # rgb not needed anymore

    x_min, x_max = -4.744822833574398, 4.744822833574398
    y_min, y_max = -2.771545257263845, 2.771545257263845

    # x_min, x_max = np.min(x_vals), np.max(x_vals)
    # y_min, y_max = np.min(y_vals), np.max(y_vals)
    x_scale = (image_width - 1) / (x_max - x_min)
    y_scale = (image_height - 1) / (y_max - y_min)
    
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    
    # Vectorized computation of image coordinates
    x_img = np.clip(((x_vals - x_min) * x_scale).astype(int), 0, image_width - 1)
    y_img = np.clip(((y_vals - y_min) * y_scale).astype(int), 0, image_height - 1)


        
    # Map z values to depth (grayscale) 255 is close, 0 is far
    depth_values = 255 - np.clip(z_vals, 0, 5) * 255 / 5
    depth_values = depth_values.astype(np.uint8)
    

    # Filter valid indices within image boundaries
    valid_mask = (x_img >= 0) & (x_img < image_width) & (y_img >= 0) & (y_img < image_height)


    # Assign depth values to the corresponding pixels
    image[y_img[valid_mask], x_img[valid_mask]] = depth_values[valid_mask, None]
    return image

def print_data(data):
    point_generator = pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = np.array(list(point_generator))
    image = point_cloud_to_image(points)
    return image

def callback_function(msg):
    rospy.loginfo("Message received by subscriber")
    image = print_data(msg)
    blurred_image = cv2.GaussianBlur(image, (5, 5), 20)  # Adjust kernel size and sigma value as needed
    
    cv2.imshow('Point Cloud Image', blurred_image)
    key = cv2.waitKey(3)
    if key == ord('q'):
        rospy.signal_shutdown("pressed q")

if __name__ == "__main__":
    rospy.init_node("depth_image_subscriber")
    rospy.loginfo("PointCloud subscriber launched")

    sub = rospy.Subscriber("/filtered_depth/pointcloud", PointCloud2, callback_function)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)
    
    rospy.loginfo("Exiting program")
    rospy.sleep(1)
    sys.exit()