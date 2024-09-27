import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import rospy
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

def reverse_image_to_point_cloud(depth_image, x_min, x_max, y_min, y_max, image_width, image_height):
    """
    Reverse the transformations to convert the depth image back to a point cloud.
    """
    # Create grid of pixel coordinates
    y_img, x_img = np.mgrid[0:image_height, 0:image_width]

    # Reverse the scaling of x and y image coordinates back to meters
    x_vals = x_img / ((image_width - 1) / (x_max - x_min)) + x_min
    y_vals = y_img / ((image_height - 1) / (y_max - y_min)) + y_min

    # Z values are directly from the depth image
    z_vals = depth_image  # Depth values remain unchanged

    # Combine x, y, z into a point cloud
    point_cloud = np.stack([x_vals, y_vals, z_vals], axis=-1)

    return point_cloud

def create_point_cloud2(points, frame_id="camera_link"):
    """ Create a PointCloud2 message from a list of 3D points. """
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Create a PointCloud2 message
    cloud_msg = pc2.create_cloud(header, fields, points.reshape(-1, 3))  # Flatten the point cloud for publishing
    return cloud_msg


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

    # Retrieve the x, y, z values from the point cloud
    x_vals = points[:, 0]
    y_vals = points[:, 1]
    z_vals = points[:, 2]  # This is the raw z-coordinate (distance from the camera)
    rgb_vals = points[:, 3]


    # Calculate the real z-value (horizontal distance in front of the robot)
    camera_angle_rad = np.radians(camera_angle)  # Convert to radians
    z_real = z_vals * np.cos(camera_angle_rad)   # Project z onto the robot's forward plane

    # Now z_real represents how far the object is in front of the robot

    x_min, x_max = -0.5, 0.5
    y_min, y_max = -1, 1
    x_scale = (image_width - 1) / (x_max - x_min)
    y_scale = (image_height - 1) / (y_max - y_min)
    
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    image_3d = np.zeros((image_height, image_width, 4), dtype=np.float32)
    
    # Vectorized operation for converting point coordinates to image coordinates
    x_img = np.clip(((x_vals - x_min) * x_scale).astype(int), 0, image_width - 1)
    y_img = np.clip(((y_vals - y_min) * y_scale).astype(int), 0, image_height - 1)
    
    # Use z_real instead of z_vals for further calculations
    image_3d[y_img, x_img, 3] = z_real

    # Convert float RGB values to uint32 first
    rgb_vals_uint32 = rgb_vals.astype(np.float32).view(np.uint32)
    r = (rgb_vals_uint32 >> 16) & 0x0000ff
    g = (rgb_vals_uint32 >> 8) & 0x0000ff
    b = rgb_vals_uint32 & 0x0000ff
    
    # Assign pixel values
    image[y_img, x_img, :] = np.stack([b, g, r], axis=-1)
    image_3d[y_img, x_img, :3] = np.stack([b, g, r], axis=-1)
    # Use z_real instead of z_vals for further calculations
    image_3d[y_img, x_img, 3] = z_real
    
    return image, image_3d

def print_data(data):
    point_generator = pc2.read_points(data, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = np.array(list(point_generator))
    image, image_3d = point_cloud_to_image(points)
    
    filtered_image = filter_img(image)
    
    white_pixels = np.where(filtered_image == 255)
    # select the z-values (depth values) corresponding to those white pixels
    # 3 refers to the fourth channel in image_3d, which contains the depth information (z_real).
    z_values = image_3d[white_pixels[0], white_pixels[1], 3] 

    valid_mask = z_values != 0
    z_values = z_values[valid_mask]
    white_pixels = (white_pixels[0][valid_mask], white_pixels[1][valid_mask])
    
    # z_visual = np.full_like(filtered_image, 255, dtype=np.float32)
    # z_visual[white_pixels] = z_values # Advanced Indexing in NumPy - look this up if unclear
    
    # Initialize z_visual as an RGB image (3 channels), filled with white (255)
    depth_image = np.full_like(filtered_image, 255, dtype=np.float32)
    z_visual_rgb = np.full((filtered_image.shape[0], filtered_image.shape[1], 3), 0, dtype=np.float32)
    
    z_min, z_max = np.min(z_values), np.max(z_values)
    z_normalized = 255 * (z_values - z_min) / (z_max - z_min)

    z_normalized_red = np.where(z_normalized < 85, 255 - z_values, 0)  # Red range (under 85)
    z_normalized_blue = np.where((z_normalized >= 85) & (z_normalized < 170), 255 - z_values, 0)  # Blue range (85 to 170)
    z_normalized_green = np.where(z_normalized >= 170, 255 - z_values, 0)  # Green range (170 to 255)

    depth_image[white_pixels[0], white_pixels[1]] = z_values
    # Assign the corresponding color channels
    z_visual_rgb[white_pixels[0], white_pixels[1], 2] = z_normalized_red   # Red channel
    z_visual_rgb[white_pixels[0], white_pixels[1], 0] = z_normalized_green # Green channel
    z_visual_rgb[white_pixels[0], white_pixels[1], 1] = z_normalized_blue  # Blue channel

    # The above lines use Advanced Indexing in NumPy - look this up if unclear

    # print(f"shape of depth image: {filtered_image.shape}")

    return image, filtered_image, depth_image, z_visual_rgb

def callback_function(msg):
    rospy.loginfo("Message received by subscriber")
    image, filtered_image, depth_image, z_visual_rgb = print_data(msg)

    # Reverse the transformations and generate the point cloud from depth_image
    point_cloud = reverse_image_to_point_cloud(depth_image, x_min, x_max, y_min, y_max, image.shape[1], image.shape[0])
    
    # Publish the point cloud
    cloud_msg = create_point_cloud2(point_cloud)
    pub.publish(cloud_msg)
    
    cv2.imshow('Point Cloud Image', image)
    cv2.imshow('Filtered Image', filtered_image)
    cv2.imshow('Depth Image', depth_image)
    cv2.imshow('RGB Depth Image', z_visual_rgb)
    
    key = cv2.waitKey(3)
    if key == ord('q'):
        rospy.signal_shutdown("pressed q")

if __name__ == "__main__":
    rospy.init_node("python_subscriber")
    rospy.loginfo("PointCloud subscriber launched")

    # Get the camera parameters
    camera_height = rospy.get_param("camera_height", 1.0)  # Default to 1 meter
    camera_angle = rospy.get_param("camera_angle", -0.7854)  # Default to -0.7854 radians (45 degrees downwards)

    # Broadcast the static transform
    broadcast_static_transform(camera_height, camera_angle)

    sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_function)

    pub = rospy.Publisher("/converted_point_cloud", PointCloud2, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
    
    rospy.loginfo("Exiting program")
    rospy.sleep(1)
    sys.exit()
