The Ros Bags can be found at https://drive.google.com/drive/folders/1hVJtSm9x8AIl_QXRdKh3ZG6uII_5OwxN

Use `rosbag record [topic_name]` to record data to a file.
Use `rosbag play [file_name]` to replay the data.

## Info about Python files:

`view_bag.py` is used to visualize the raw color image data (2D).
`view_color_2d_img.py` does the same thing.
`view_pointcloud.py` is used to visualize the raw PointCloud data.
`realsense_cv_z_stream.py` is the file used to demo lane detection. It opens a window with the original PointCloud and filtered black and white image.

If unsure which topic to record, check the python file and see which topic we are subscribing to.

## ToDo:

- converting our ros depth image into pointcloud (for path planning)
- look into ROS static frame transforms to account for the camera being tilted down. According to Tarun, "we can write transforms from one sensor's coordinate frame (in this case the camera) to the xyz coordinate frame of the robot which is called base_link. base_link would be located at the center point between our wheels since the camera would be tilted, the transforms would have both a translation and rotation component"