#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

def callback(Image):
    data = Image.data
    datanp = np.array(data)
    #data3d = datanp.reshape((480,640,3))
    rospy.loginfo(data)



def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("usb_cam/image_raw", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    listener()