import rospy
import actionlib
import math
from ranger_actionlib.msg import Goal 
from move_base_msgs.msg import MoveBaseActionFeedback


rospy.init_node('goal_sender')
pub = rospy.Publisher('current_goal', Goal, queue_size=2, latch=True)

# [[x_n, y_n, z_n, yaw_n]...]  in meters & degrees 
# GOALS = [[1.0, 0.0, 0.0, -5.0], [2.65, 1.5, 0.0, -20.0], [3.0, 2.0, 0.0, -80.0]]

# TO DO: Declare GPS goals 
# GOALS = [[5.0, 0.0, 0.0, 0.0],              # This line moves 5 m forward in the map frame from the start position (edit as needed)
#          [long_1, lat_1, 0.0, yaw_1],       # Declare longitude and latitude waypoints (add or remove lines as necessary)
#          [long_2, lat_2, 0.0, yaw_2],       # yaw_n (degrees) is the z-axis rotation of the robot once it reached the waypoint 
#          [long_3, lat_3, 0.0, yaw_3],       # the +z axis points up from robot; safe to have values that make sense here
#          [long_4, lat_4, 0.0, yaw_4],       # All yaw values are relative to map frame
#          [long_5, lat_5, 0.0, yaw_5],       # If unsure of yaw value, use 0.0 
#          [long_6, lat_6, 0.0, yaw_6],
#          [long_7, lat_7, 0.0, yaw_7],
#          [long_8, lat_8, 0.0, yaw_8],
#          [long_9, lat_9, 0.0, yaw_9],
#          [long_10, lat_10, 0.0, yaw_10]]


GOALS = [[6.0, 0.0, 0.0, 0.0]]
CURR_GOAL = None
NAV_OVER = False


def pos_feedback_cb(data):
    # compute distance between current feedback pos. and current goal pos. (in map frame)
    # determine whether to send next goal (if available) based on distance tolerance 
    # if no goal is available, end navigation 
    global CURR_GOAL, GOALS, NAV_OVER
    curr_x = data.feedback.base_position.pose.position.x 
    curr_y = data.feedback.base_position.pose.position.y

    dist_to_goal = math.sqrt((curr_x - CURR_GOAL.x)**2 + (curr_y - CURR_GOAL.y)**2)

    # Close to current goal...
    if dist_to_goal <= 0.15 and len(GOALS) != 0:   
        GOALS.pop(0)  # remove current goal from GOALS list 

        to_goal = Goal()
        
        # publish Goal msg
        try: 
            to_goal.x, to_goal.y, to_goal.z, to_goal.yaw = GOALS[0][0], GOALS[0][1], GOALS[0][2], GOALS[0][3]
            to_goal.header.frame_id = "map"
            to_goal.header.stamp = rospy.Time.now()
            to_goal.cancel = False

            pub.publish(to_goal)
            rospy.loginfo('Published to /current_goal!')

        # Reached last goal to tolerance -- causes index error in GOALS after pop(0)
        # Publish goal with all -1.0 values and set cancel boolean to True
        # LAST WAYPOINT's YAW ANGLE IS IGNORED! Robot will come to a halt in it's current trajectory.
        except IndexError:
            to_goal.x, to_goal.y, to_goal.z, to_goal.yaw = -1.0, -1.0, -1.0, -1.0
            to_goal.header.frame_id = "map"
            to_goal.header.stamp = rospy.Time.now()
            to_goal.cancel = True
            pub.publish(to_goal)

            rospy.loginfo("REACHED LAST AVAILABLE WAYPOINT.")
            NAV_OVER = True  # toggle var to indicate all current waypoints have been exhausted 


    # If GOALS went from being empty to not empty (i.e. a new waypoint was added after robot came to a halt)
    # FIX THIS --- WON'T RUN BECAUSE NO FEEDBACK CALLBACK IF NOT MOVING
    elif NAV_OVER is True and len(GOALS) != 0:
        NAV_OVER = False 

        to_goal.x, to_goal.y, to_goal.z, to_goal.yaw = GOALS[0][0], GOALS[0][1], GOALS[0][2], GOALS[0][3]
        to_goal.header.frame_id = "map"
        to_goal.header.stamp = rospy.Time.now()
        to_goal.cancel = False

        pub.publish(to_goal)
        rospy.loginfo('Published to /current_goal!')


    else:
        rospy.loginfo('Waiting to get closer to current goal before publishing next goal...')


def current_goal_cb(msg):
    # Set current goal in map frame
    global CURR_GOAL
    CURR_GOAL = msg


def main():

    global CURR_GOAL, GOALS

    to_goal_1 = Goal()
    to_goal_1.header.frame_id = "map"
    to_goal_1.header.stamp = rospy.Time.now()
    to_goal_1.cancel = False 

    to_goal_1.x, to_goal_1.y, to_goal_1.z, to_goal_1.yaw = GOALS[0][0], GOALS[0][1], GOALS[0][2], GOALS[0][3]
    CURR_GOAL = to_goal_1

    # Wait for a subscriber to connect to the topic
    rospy.loginfo('Waiting for subscriber to /current_goal...')
    rospy.sleep(2)

    pub.publish(to_goal_1)
    rospy.loginfo('Published initial /current_goal in the map frame!')

    rospy.Subscriber('/current_goal_map', Goal, current_goal_cb, queue_size=1)
    rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, pos_feedback_cb, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()