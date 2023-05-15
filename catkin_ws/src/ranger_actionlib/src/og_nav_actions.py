#!/usr/bin/env python
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from ranger_actionlib.msg import Goal


### REFERENCE: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

class MoveBaseSeq():

    def __init__(self):

        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0


        # Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(30.0))

        if not wait:
            rospy.logerr("Action server unavailable.")
            rospy.signal_shutdown("Action server unavailable.")
            return
        
        rospy.loginfo("Connected to move base server!")

    
    # data: Goal msg received from /current_goal
    def to_goal_cb(self, data):
        
        # If no cancel goal request
        if data.cancel is False:
            self.goal_cnt += 1

            rospy.loginfo('Goal message received from /current_goal')

            # Clear list of poses requested previously
            self.pose_seq.clear()

            # coordinate goals [x1, y1, z1, x2, y2, z2, z_n, y_n, z_n] RELATIVE TO MAP FRAME (not robot local frame)
            points_seq = [data.x, data.y, data.z]

            # yaw goals: [yaw1, yaw2, yaw_n] (degrees) RELATIVE TO MAP COORDINATE FRAME (not robot local frame)
            # yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
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
        # rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")


    def done_cb(self, status, result):
        # self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
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


    def movebase_client(self):
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
    
    rospy.Subscriber("current_goal", Goal, goal_obj.to_goal_cb, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")