#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sabertooth_ctrl.msg import MotorStr.msg
import math
import serial
import struct



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

        # constants (physical properties of the robot)
        # self.WHEEL_RADIUS = 0.06096  # radius of wheels (meters)
        # self.WHEEL_SEPARATION = 0.31  # width of the robot (meters)
        self.WHEEL_RADIUS = 0.1016  # radius of wheels (meters)
        self.WHEEL_SEPARATION = 0.4  # wheel base (meter); distance between drive wheels


    def cmd_vel_cb(self, vel):

        vel_msg = MotorStr()

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

        # Open the serial connection (change 'COM1' to the appropriate port)
        ser = serial.Serial('COM5', baudrate=9600, timeout=1)

        # Pack the integers into bytes and send them
        data = struct.pack('ii', self.w_l, self.w_r)
        ser.write(data)

        # Close the serial connection when done
        ser.close()

    def main(self):
        rospy.spin()

    def map_val(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == "__main__":
    rospy.init_node("CmdVelToMotors")
    cmdvel = CmdVelToMotors()
    cmdvel.main()
