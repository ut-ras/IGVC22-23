#!/usr/bin/env python
import rospy
import math

import actionlib
from geographiclib.geodesic import Geodesic
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from ranger_actionlib.msg import Goal


def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  origin_pose = rospy.wait_for_message('local_xy_origin', Pose)   
  origin_lat = origin_pose.position.y
  origin_long = origin_pose.position.x
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long


def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  azimuth = g['azi1']
  
  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lengths of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
  return x, y


class MoveBaseSeq():

    def __init__(self):

        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. 
        # # We use this to calculate our goal within the map frame.
        # self.origin_lat, self.origin_long = get_origin_lat_long()

        # Create action client; wait for 90 seconds for server to become available
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(90.0))
        if not wait:
            rospy.logerr("Action server unavailable.")
            rospy.signal_shutdown("Action server unavailable.")
            return
        rospy.loginfo("Connected to move base server!")

        # Set up publisher for current goal in map frame 
        self.map_curr_goal_pub = rospy.Publisher("/current_goal_map", Goal, queue_size=1)

    
    def do_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
        # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
        x, y = calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
        # Create move_base goal
        # self.publish_goal(x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch)
        return x, y, z

    
    # data: Goal msg received from /current_goal
    def to_goal_cb(self, data):

        frame = data.header.frame_id.lower()
        
        # If no cancel goal request
        if data.cancel is False:
            
            # Goal is in 'map' frame (e.g. initial goal of 5 m forward from starting spot)
            if frame == 'map':
                self.goal_cnt += 1

                rospy.loginfo('Goal message received from /current_goal. Frame: map')

                # Clear list of poses requested previously
                self.pose_seq.clear()

                # coordinate goals [x1, y1, z1, x2, y2, z2, z_n, y_n, z_n] RELATIVE TO MAP FRAME (not robot local frame)
                points_seq = [data.x, data.y, data.z]

                # yaw goals: [yaw1, yaw2, yaw_n] (degrees) RELATIVE TO MAP COORDINATE FRAME (not robot local frame)
                yaweulerangles_seq = [data.yaw]

                # List of goal quaternions:
                quat_seq = list()

                # Convert yaw goals to quarternions 
                for yawangle in yaweulerangles_seq:
                    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

                # Group point goals 
                n = 3
                # Returns a list of lists [[point1], [point2],...[pointn]]
                points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

                # Build list of poses 
                for point in points:
                    self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
                    n += 1

            # Goal is in 'gps' frame 
            else:
                self.goal_cnt += 1

                rospy.loginfo(f'Goal message received from /current_goal. Frame: {frame}')

                # Clear list of poses requested previously
                self.pose_seq.clear()

                # convert GPS frame to map frame coordinates
                x, y, z = self.do_gps_goal(goal_lat=data.y, goal_long=data.x)

                # coordinate goals [x1, y1, z1] RELATIVE TO MAP FRAME 
                points_seq = [x, y, z]

                # yaw goals: [yaw1, yaw2, yaw_n] (degrees) RELATIVE TO MAP COORDINATE FRAME 
                yaweulerangles_seq = [data.yaw]

                # List of goal quaternions:
                quat_seq = list()

                # Convert yaw goals to quarternions 
                for yawangle in yaweulerangles_seq:
                    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

                # Group point goals 
                n = 3
                # Returns a list of lists [[point1], [point2],...[pointn]]
                points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

                # Build list of poses 
                for point in points:
                    self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
                    n += 1

            self.movebase_client()

        # Cancel goal request received 
        else:
            self.client.cancel_goal()


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.pose_seq[0])+" is now being processed...")


    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))


    def done_cb(self, status, result):
        # Reference for terminal status values: 
        # http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")


    def movebase_client(self, x = None, y = None):
        
        # Publish current goal in map frame 
        goal_msg = Goal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = 'map'
        goal_msg.x = self.pose_seq[0].position.x 
        goal_msg.y = self.pose_seq[0].position.y
        goal_msg.z = 0.0
        goal_msg.yaw = -1
        goal_msg.cancel = False
        self.map_curr_goal_pub.publish(goal_msg)

        # Send goal to action server 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[0]

        rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[0]))

        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)


def main():

    rospy.init_node('nav_actions')
    goal_obj = MoveBaseSeq()

    rospy.loginfo('Waiting for message from /current_goal...')
    rospy.Subscriber("current_goal", Goal, goal_obj.to_goal_cb, queue_size=2)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")