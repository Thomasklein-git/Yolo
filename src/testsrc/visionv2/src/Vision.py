#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class image_converter:

  def __init__(self):
    global yolo
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback)#, queue_size=1,buff_size=2**24)
    #self.image_sub = rospy.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage,self.callback)#, queue_size=1,buff_size=2**8)
    yolo = Load_Yolo_model()

  def callback(self,data):
    try:
      #np_arr = np.fromstring(data.data, np.uint8)
      #cv_image=cv2.imdecode(np_arr,cv2.COLOR_BGR2RGB)
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
      #cv_cam=cv_image
      #cv2.imshow("cam",cv_cam)
    except CvBridgeError as e:
      print(e)

    ####
    cv_image, bboxes=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
    #cv_image=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, CLASSES=TRAIN_CLASSES, rectangle_colors=(255,0,0)) # Used later for custom weigths
    # bboxes are 
    #print(bboxes)
    x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
    #print(x1)
    ####

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    

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
