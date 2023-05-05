#!/usr/bin/env python

from pysabertooth import Sabertooth
import serial.tools.list_ports as port
import time
import rospy
from sabertooth_ctrl.msg import TwoFloats


class SabertoothControl:
    """

    """

    def __init__(self, saber_baudrate, saber_addr, saber_timeout):
        self.BAUDRATE = saber_baudrate
        self.SABERTOOTH_ADDRESS = saber_addr
        self.SABERTOOTH_TIMEOUT = saber_timeout
        self.motor_cmd_sub = rospy.Subscriber(
            "motor_cmd", TwoFloats, self.motor_cmd_cb, queue_size=1
        )
        self.vel = TwoFloats()

    def initialize_sabertooth(self):
        rospy.loginfo("Detecting sabertooth....\n")
        portlist = list(port.comports())
        # print portlist
        serial_port = ""
        try:
            for p in portlist:
                # print p
                if "Sabertooth" in str(p):
                    serial_port = str(p).split(" ")
        except Exception as e:
            rospy.logerr(e)

        finally:
            self.saber = Sabertooth(
                serial_port[0],
                self.BAUDRATE,
                self.SABERTOOTH_ADDRESS,
                self.SABERTOOTH_TIMEOUT,
            )

            rospy.loginfo(
                "%s MC found and connected on port : %s %s", str(
                    serial_port[2]), str(serial_port[0]), str(serial_port[1])
            )

    def motor_cmd_cb(self, data):
        self.v_l = data.left
        self.v_r = data.right
        # rospy.loginfo("received commands: vl: %d, vr:%d", self.v_l, self.v_r)
        # drive(number, speed)
        # number: 1-2
        # speed: -100 - 100
        self.saber.drive(1, self.v_l)
        self.saber.drive(2, self.v_r)

    def main(self):
        # self.rospy_rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("sabertooth_driver")
    # TODO: get sabertooth params from parameter server/launch file
    sc = SabertoothControl(saber_baudrate=9600,
                            saber_addr=128, saber_timeout=0.1)

    sc.initialize_sabertooth()
    time.sleep(2)
    sc.main()
