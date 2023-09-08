#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math



class CmdVelToMotors:
    """
    Reference: https://github.com/avs2805/sabertooth_rospy
    
    This class has 4 methods:
    1. __init__ :   - creates a subscriber for cmd_vel (published from teleop_twist_joy).
                    - creates a publisher for wheel velocities
                    - sets constants values for wheel speed calculations.
    2. cmd_vel_cb:  - callback method for cmd_vel, executed every time a new callback message is received.
                    - calculates wheel velocities using differential drive kinematics Ref: https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
                    - publishes motor commands as separate wheel velocities

    3. map_val:     - linear interpolation of a given range of input values between an allowed max and min

    4. main:        - prevents the Python Main thread from exiting.
    """

    def __init__(self):
        # create subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_vel_cb, queue_size=1
        )
        

        self.pub_motor_cmd_str = rospy.Publisher(
            "motor_cmd_str", String, queue_size=3)
        

        self.cmd_msg_str = String()  # object for int32 message

        # constants (physical properties of the robot)
        # self.WHEEL_RADIUS = 0.06096  # radius of wheels (meters)
        # self.WHEEL_SEPARATION = 0.31  # width of the robot (meters)
        self.WHEEL_RADIUS = 0.1016  # radius of wheels (meters)
        self.WHEEL_SEPARATION = 0.4  # wheel base (meter); distance between drive wheels


    def cmd_vel_cb(self, vel):
        
        # read linear and angular velocities from joy_node
        self.lin_speed = vel.linear.x
        self.ang_speed = vel.angular.z

        # convert linear and angular inputs to left and right wheel velocities
        self.w_l = ((2 * self.lin_speed) - (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )

        self.w_r = ((2 * self.lin_speed) + (self.ang_speed * self.WHEEL_SEPARATION)) / (
            2.0 * self.WHEEL_RADIUS
        )
        rospy.loginfo("calc commands: w_l: %d, w_r:%d", self.w_l, self.w_r)

        # FOR MAX_VEL_X = 2.25 M/S AND MAX_VEL_THETA = 1.0 rad/s:
        # values calculated above are [-24, 24] --> map to [-99, 99]
        # Adjust in_min, in_max, out_min, outmax_max for different max_vel and max_theta (from diff drive eqn)
        self.w_l = self.map_val(self.w_l, -17, 17, -99, 99)
        self.w_r = self.map_val(self.w_r, -17, 17, -99, 99)

        rospy.loginfo("mapped commands: w_l: %d, w_r:%d", self.w_l, self.w_r)

        # Negative velocity --> set sign indicator to 1, else 0
        left_sign = "0" if self.w_l >= 1e-3 else "1"
        right_sign = "0" if self.w_r >= 1e-3 else "1"

        rounded_wl = int(math.floor(abs(self.w_l)))
        rounded_wr = int(math.floor(abs(self.w_r)))


        if rounded_wl < 10:
            rounded_wl = "0" + str(rounded_wl)
        else:
            rounded_wl = str(rounded_wl)

        if rounded_wr < 10:
            rounded_wr = "0" + str(rounded_wr)
        else:
            rounded_wr = str(rounded_wr)

        # Motor cmd str in format {LL}{RR}{L_sign}{R_sign}
        self.cmd_msg_str = str(rounded_wl + rounded_wr + left_sign + right_sign)
        
        for i in range(3):
            self.pub_motor_cmd_str.publish(self.cmd_msg_str)
            rospy.loginfo("Published!")
            rospy.sleep(0.25)

    def main(self):
        rospy.spin()

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == "__main__":
    rospy.init_node("CmdVelToMotors")
    cmdvel = CmdVelToMotors()
    cmdvel.main()
