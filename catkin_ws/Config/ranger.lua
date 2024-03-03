-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                       
  tracking_frame = "base_link",             -- set to frame id of IMU (imu_link) if available
  published_frame = "base_link",            -- should be "odom" if odometry frame available (e.g. from ekf or ukf), else base_link
  odom_frame = "odom",                      -- only used if provide_odom_frame is true
  provide_odom_frame = true,                -- publish odom_frame -> published_frame tf  (USE FALSE IF USING EKF / UKF)
  publish_frame_projected_to_2d = true,     -- zero out roll, pitch, and z-offset 
  use_pose_extrapolator = true,
  use_odometry = false,                     -- subscribes to /odom (SET TO TRUE FOR EKF / UKF)
  use_nav_sat = false,                      -- GPS data, if published as sensor_msgs/NavSatFix on /fix topic
  use_landmarks = false,                    
  num_laser_scans = 1,                      -- sensor_msgs/LaserScan on topic /scan
  num_multi_echo_laser_scans = 0,            
  num_subdivisions_per_laser_scan = 10,     -- irrelevant (this if for multi-echo laser scans)
  num_point_clouds = 1,                     -- pointcloud2 on topic /points2
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- WHAT DOES THIS DO??
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10 

return options

