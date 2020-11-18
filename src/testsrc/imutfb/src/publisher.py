#!/usr/bin/env python3
import rospy
from imutfb.msg import imu_message
from sensor_msgs.msg import Imu
from collections import namedtuple

# Download with "sudo apt install python3-tinkerforge"
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu_v2 import BrickIMUV2

# Sensor specific information
HOST = "localhost"
PORT = 4223
UID = "6fXE6x"

def StdDataHandler():
    msg = Imu()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "Std_IMU_Data"

    msg.linear_acceleration.x = imu.get_all_data().linear_acceleration[0]/100.0
    msg.linear_acceleration.y = imu.get_all_data().linear_acceleration[1]/100.0
    msg.linear_acceleration.z = imu.get_all_data().linear_acceleration[2]/100.0

    msg.angular_velocity.x = imu.get_all_data().angular_velocity[0]/16
    msg.angular_velocity.y = imu.get_all_data().angular_velocity[1]/16
    msg.angular_velocity.z = imu.get_all_data().angular_velocity[2]/16

    msg.orientation.w = imu.get_all_data().quaternion[0]/16383.0
    msg.orientation.x = imu.get_all_data().quaternion[1]/16383.0
    msg.orientation.y = imu.get_all_data().quaternion[2]/16383.0
    msg.orientation.z = imu.get_all_data().quaternion[3]/16383.0

    return msg

def AllDataHandler():
    msg = imu_message()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "All_IMU_Data"
    
    msg.linear_acceleration.x = imu.get_all_data().linear_acceleration[0]/100.0
    msg.linear_acceleration.y = imu.get_all_data().linear_acceleration[1]/100.0
    msg.linear_acceleration.z = imu.get_all_data().linear_acceleration[2]/100.0

    msg.acceleration.x = imu.get_all_data().acceleration[0]/100.0
    msg.acceleration.y = imu.get_all_data().acceleration[1]/100.0
    msg.acceleration.z = imu.get_all_data().acceleration[2]/100.0

    msg.gravity_vector.x = imu.get_all_data().gravity_vector[0]/100.0
    msg.gravity_vector.y = imu.get_all_data().gravity_vector[1]/100.0
    msg.gravity_vector.z = imu.get_all_data().gravity_vector[2]/100.0

    msg.angular_velocity.x = imu.get_all_data().angular_velocity[0]/16
    msg.angular_velocity.y = imu.get_all_data().angular_velocity[1]/16
    msg.angular_velocity.z = imu.get_all_data().angular_velocity[2]/16

    msg.orientation.w = imu.get_all_data().quaternion[0]/16383.0
    msg.orientation.x = imu.get_all_data().quaternion[1]/16383.0
    msg.orientation.y = imu.get_all_data().quaternion[2]/16383.0
    msg.orientation.z = imu.get_all_data().quaternion[3]/16383.0

    msg.euler_angle.x = imu.get_all_data().euler_angle[0]/16.0
    msg.euler_angle.y = imu.get_all_data().euler_angle[1]/16.0
    msg.euler_angle.z = imu.get_all_data().euler_angle[2]/16.0

    msg.magnetic_field.x = imu.get_all_data().magnetic_field[0]/16.0
    msg.magnetic_field.y = imu.get_all_data().magnetic_field[1]/16.0
    msg.magnetic_field.z = imu.get_all_data().magnetic_field[2]/16.0

    msg.temperature = imu.get_all_data().temperature

    #msg.calibration_status = imu.get_all_data().calibration_status

    return msg

def talker():
    #pub = rospy.Publisher('IMU_Brick', imu_message, queue_size=1)   # Name of the published ROS topic
    rospy.init_node('IMU_Brick_pub', anonymous=True)    # Name of the ROS Node
    pub1 = rospy.Publisher('IMU_Brick_All', imu_message, queue_size=1)   # Name of the published ROS topic
    pub2 = rospy.Publisher('IMU_Brick_Std', Imu, queue_size=1)   # Name of the published ROS topic
    r = rospy.Rate(10)  #10hz
    
    msg = imu_message()
    
    while not rospy.is_shutdown():
        msg1 = AllDataHandler()
        msg2 = StdDataHandler()
        #rospy.loginfo(msg)
        pub1.publish(msg1)
        pub2.publish(msg2)
        r.sleep()

if __name__ == '__main__':
    ipcon = IPConnection() # Create IP connection
    imu = BrickIMUV2(UID, ipcon) # Create device object
    ipcon.connect(HOST, PORT) # Connect to brickd

    try:
        talker()
    except rospy.ROSInterruptException: pass