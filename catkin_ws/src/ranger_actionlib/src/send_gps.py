import serial
import pynmea2
import rospy 
from geometry_msgs.msg import Pose

# CONNECT GPS MODULE TO COMPUTER OVER SERIAL 
# Configure the serial port
serial_port = "COM16"  # Replace with your actual serial port
baud_rate = 38400  # Replace with the baud rate used by your GPS device

# Open the serial port
ser = serial.Serial(serial_port, baud_rate)

# Read and parse GPS data
def main():
    rospy.init_node('send_gps')
    pub = rospy.Publisher('local_xy_gps', Pose, queue_size= 1, latch=True)
    rate = rospy.Rate(0.5) # 0.5 hz

    while not rospy.is_shutdown():
        pub_msg = Pose()
        # Read a line of data from the serial port
        line = ser.readline().decode('utf-8')
        msg = pynmea2.parse(line)

        pub_msg.position.y = msg.latitude  # latitude
        pub_msg.position.x = msg.longitude # longitude 
        pub_msg.position.z = 0.0

        pub_msg.orientation.x = 0.0
        pub_msg.orientation.y = 0.0
        pub_msg.orientation.z = 0.0
        pub_msg.orientation.w = 1.0

        pub.publish(pub_msg)
        rate.sleep()

    ser.close()


if __name__ == '__main__':
    main()