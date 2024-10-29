#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# global IMU_data #stores raw IMU data recieved from sensor
# global IMU_data #stores raw IMU data recieved from sensor
global publish_rate #rate at which IMU data has to be published


#Subscriber for raw imu data
#stores recieved data on IMU_data & calls transform
def rec(msg):
        
    IMU_data = msg
    IMU_data.header.frame_id = msg.header.frame_id
    IMU_data.orientation.x = msg.orientation.x
    IMU_data.orientation.y = msg.orientation.y
    IMU_data.orientation.z = msg.orientation.z

    IMU_data.angular_velocity.x = msg.angular_velocity.x
    IMU_data.angular_velocity.y = msg.angular_velocity.y
    IMU_data.angular_velocity.z = msg.angular_velocity.z

    IMU_data.linear_acceleration.x = msg.linear_acceleration.x
    IMU_data.linear_acceleration.y = msg.linear_acceleration.y
    IMU_data.linear_acceleration.z = msg.linear_acceleration.z

    transformed = transform(IMU_data)
    publish_msg(transformed)
        
        

def rec_back(msg):
        
    IMU_data = msg
    IMU_data.header.frame_id = msg.header.frame_id
    IMU_data.orientation.x = msg.orientation.x
    IMU_data.orientation.y = msg.orientation.y
    IMU_data.orientation.z = msg.orientation.z

    IMU_data.angular_velocity.x = msg.angular_velocity.x
    IMU_data.angular_velocity.y = msg.angular_velocity.y
    IMU_data.angular_velocity.z = msg.angular_velocity.z

    IMU_data.linear_acceleration.x = msg.linear_acceleration.x
    IMU_data.linear_acceleration.y = msg.linear_acceleration.y
    IMU_data.linear_acceleration.z = msg.linear_acceleration.z

    transformed = transform(IMU_data)
    publish_back_msg(transformed)
        

#used to transfrom Imu type data if needed
#TO BE UPDATED BASED ON FRAME OF REFERENCE FROM SENSOR
def transform(sensor_data):
    temp = sensor_data.orientation.x
    sensor_data.orientation.x = -1*sensor_data.orientation.y
    sensor_data.orientation.y = -1*temp
    sensor_data.orientation.z *= 1
        
    temp = sensor_data.angular_velocity.x
    sensor_data.angular_velocity.x = sensor_data.angular_velocity.y
    sensor_data.angular_velocity.y = temp
    sensor_data.angular_velocity.z *= -1

    temp = sensor_data.linear_acceleration.x
    sensor_data.linear_acceleration.x = -1*sensor_data.linear_acceleration.y
    sensor_data.linear_acceleration.y = -1*temp
    sensor_data.linear_acceleration.z *= 1
        
    return sensor_data

#publisher for transformed IMU message
#publishes at specified rate
def publish_msg(sensor_data):
        pub = rospy.Publisher('imu', Imu, queue_size=30)   # change name to /imu
        # rospy.loginfo("publisher setup- topic: /imu  @"+str(publish_rate))
        # rate = rospy.Rate(publish_rate)

        pub.publish(sensor_data)
        # rate.sleep()

def publish_back_msg(sensor_data):
        pub = rospy.Publisher('imu_back', Imu, queue_size=30)   # change name to /imu
        # rospy.loginfo("publisher setup- topic: /imu  @"+str(publish_rate))

        pub.publish(sensor_data)

if __name__ == '__main__':
        publish_rate = 30

        rospy.init_node("robot_imu_publisher")
        rospy.Subscriber("imu/data_raw", Imu, rec, queue_size=30)
        rospy.Subscriber("imu_back/data_raw", Imu, rec_back, queue_size=30)

        rospy.spin()
