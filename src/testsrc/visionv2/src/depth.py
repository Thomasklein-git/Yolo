#!/usr/bin/env python3
#from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class image_converter:

  def __init__(self):
    global yolo
    self.active=0
    self.cv_image_cam=[]
    self.cv_image_depth=[]
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
    self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
    yolo = Load_Yolo_model()
 
  def callback_cam(self,data):
    try:
      self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)

    if self.active==1:
      self.calculation()
    """
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    """
  
  def callback_depth(self,data):
    try:
      self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    
    if self.active==0:
      self.calculation()
    ###
    #cv_image_nonan = np.where(np.isnan(cv_image),0, cv_image)
    #cv_image_nonan *= 255/cv_image_nonan.max()
    #cv_image_norm = cv_image_nonan.astype('uint8')

    #print(cv_image_nonan[360,640])
    #print(cv_image_norm[360,640])
    ###
    
  def calculation(self):
    self.active=1
    imagecv_cam=[]
    imagecv_depth=[] 
    imagecv_cam=self.cv_image_cam
    imagecv_depth=self.cv_image_depth
    if imagecv_cam != []:
      imagecv_cam, bboxes=detect_image(yolo, imagecv_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
      #print(bboxes)
      x1, y1, x2, y2, _, _ = Give_boundingbox_coor_class(bboxes)
      print(x1,y1,x2,y2)
      print(bboxes)
      #cv2.imshow("Image cam window", imagecv_cam)
      #cv2.waitKey(3)
    #if imagecv_depth != []:
      #patch=(x2-x1,y2-y1) # gives width and heigth
      #center=(x1+patch[0]/2,y1+patch[1]/2) # gives center coodintes of bbox
      #print(center)
      #cv_image_bbox_sub=cv2.getRectSubPix(imagecv_depth,patch,center)
      #cv2.imshow("Image depth window", cv_image_bbox_sub)
      #cv2.waitKey(3)
    self.active=0
  
  

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
