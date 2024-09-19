import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct
import cv2
import sys
from matplotlib import pyplot as plt

x_min = None

def filter_img(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blurred_image = cv2.GaussianBlur(hsv_image, (15, 15), 20)  # Adjust kernel size and sigma value as needed
    lower_road = np.array([0, 0, 50])
    upper_road = np.array([179, 50, 200])

    road_mask = cv2.inRange(blurred_image, lower_road, upper_road)

    # Invert image
    inverse_mask = cv2.bitwise_not(road_mask)
    return inverse_mask

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
    rgb_vals = points[:, 3]

    x_min, x_max = -0.5, 0.5
    y_min, y_max = -1, 1
    x_scale = (image_width - 1) / (x_max - x_min)
    y_scale = (image_height - 1) / (y_max - y_min)
    
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    image_3d = np.zeros((image_height, image_width, 4), dtype=np.float32)
    
    # Vectorized operation for converting point coordinates to image coordinates
    x_img = np.clip(((x_vals - x_min) * x_scale).astype(int), 0, image_width - 1)
    y_img = np.clip(((y_vals - y_min) * y_scale).astype(int), 0, image_height - 1)
    
    # Convert float RGB values to uint32 first
    rgb_vals_uint32 = rgb_vals.astype(np.float32).view(np.uint32)
    r = (rgb_vals_uint32 >> 16) & 0x0000ff
    g = (rgb_vals_uint32 >> 8) & 0x0000ff
    b = rgb_vals_uint32 & 0x0000ff
    
    # Assign pixel values
    image[y_img, x_img, :] = np.stack([b, g, r], axis=-1)
    image_3d[y_img, x_img, :3] = np.stack([b, g, r], axis=-1)
    image_3d[y_img, x_img, 3] = z_vals
    
    return image, image_3d

def print_data(data):
    point_generator = pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = np.array(list(point_generator))
    image, image_3d = point_cloud_to_image(points)
    
    filtered_image = filter_img(image)
    
    white_pixels = np.where(filtered_image == 255)
    z_values = image_3d[white_pixels[0], white_pixels[1], 3]
    
    # z_visual = np.zeros_like(filtered_image, dtype=np.float32)
    z_visual = filtered_image // 3
    z_visual[white_pixels] = z_values
    
    return image, filtered_image

def callback_function(msg):
    rospy.loginfo("Message received by subscriber")
    image, filtered_image = print_data(msg)
    
    cv2.imshow('Point Cloud Image', image)
    cv2.imshow('Filtered Image', filtered_image)
    key = cv2.waitKey(3)
    if key == ord('q'):
        rospy.signal_shutdown("pressed q")

if __name__ == "__main__":
    rospy.init_node("python_subscriber")
    rospy.loginfo("PointCloud subscriber launched")

    sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_function)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)
    
    rospy.loginfo("Exiting program")
    rospy.sleep(1)
    sys.exit()
