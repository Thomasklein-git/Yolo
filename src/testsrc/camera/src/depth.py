#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback)

  def callback(self,data):
    time1 = rospy.get_rostime()
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    

    packtime = data.header.stamp.secs + data.header.stamp.nsecs*10**-9
    time2 = rospy.get_rostime()    

    beforetime = time1.secs + time1.nsecs*10**-9
    aftertime = time2.secs + time2.nsecs*10**-9


    recievetime = beforetime - packtime
    Evaltime = aftertime - beforetime

    print("Time to recieve signal:", recievetime)
    print("Time to evaluate signal:", Evaltime)

    cv_image_nonan = np.where(np.isnan(cv_image),0, cv_image)
    cv_image_nonan *= 255/20 #cv_image_nonan.max()
    cv_image_norm = cv_image_nonan.astype('int8')

    print(cv_image_nonan[360,640])
    print(cv_image_norm[360,640])

    cv2.imshow("Image window", cv_image_norm)
    cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
