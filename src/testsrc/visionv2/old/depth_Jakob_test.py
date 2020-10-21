#!/usr/bin/env python3
#from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class image_converter:

  def __init__(self):
    global yolo
    self.show=1 # 0: don't show 1: show
    self.active=1#0
    self.cv_image_cam=[]
    self.cv_image_depth=[]
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
    self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
    yolo = Load_Yolo_model()
 
  def callback_cam(self,data):
    if self.active==1:
      try:
        self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
        print(self.cv_image_cam.shape,"cam")
      except CvBridgeError as e:
        print(e)

      imagecv_cam=self.calculation()
      if self.show==1:
        cv2.imshow("Image cam window", imagecv_cam)
        cv2.waitKey(3)
      self.active=0
  
  def callback_depth(self,data):
    if self.active==0:
      try:
        self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
        print(self.cv_image_depth.shape,"depth")
      except CvBridgeError as e:
        print(e)
      self.active=1
    
  def calculation(self):
    imagecv_cam=[]
    imagecv_depth=[] 
    imagecv_cam=self.cv_image_cam
    imagecv_depth=self.cv_image_depth
    if imagecv_cam != []:
      imagecv_cam, bboxes=detect_image(yolo, imagecv_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
      x1, y1, x2, y2, _, _ = Give_boundingbox_coor_class(bboxes)
      print(x1,y1,x2,y2)
    if imagecv_depth != []:
      for i in range(len(bboxes)):
        patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
        center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
        Distance_to_center_of_bbox_wrt_global=imagecv_depth[center[1],center[0]] #height (y), width (x)
        print("dyb") 
    return imagecv_cam
 

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