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
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    if flag=0:
      self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
    else  
      self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback)

    yolo = Load_Yolo_model()
  
  def callback_cam(self,data):
    global bboxes, flag
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
      #self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback)
    except CvBridgeError as e:
      print(e)


    ####
    cv_image, bboxes=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
    #cv_image=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, CLASSES=TRAIN_CLASSES, rectangle_colors=(255,0,0)) # Used later for custom weigths
    # bboxes are 
    print(bboxes)
    #x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
    #print(x1)
    ####

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
    flag=1
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  
  def callback(self,data):
    #time1 = rospy.get_rostime()
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    
    """
    packtime = data.header.stamp.secs + data.header.stamp.nsecs*10**-9
    time2 = rospy.get_rostime()    

    beforetime = time1.secs + time1.nsecs*10**-9
    aftertime = time2.secs + time2.nsecs*10**-9


    recievetime = beforetime - packtime
    Evaltime = aftertime - beforetime

    print("Time to recieve signal:", recievetime)
    print("Time to evaluate signal:", Evaltime)
    """
    #print(bboxes,"hej")
    # Bounding box (from detection)
    #57.71543121, 228.3530426 , 562.47290039, 697.95489502
    x_1=int(57.77075195)
    y_1=int(228.1321106)
    x_2=int(562.04992676)
    y_2=int(697.75166321)
    print(x_1)
    patch=(x_2-x_1,y_2-y_1) # gives width and heigth
    center=(x_1+patch[0]/2,y_1+patch[1]/2) # gives center coodintes of bbox
    cv_image_bbox_sub=cv2.getRectSubPix(cv_image,patch,center)

    ###
    #cv_image_nonan = np.where(np.isnan(cv_image),0, cv_image)
    #cv_image_nonan *= 255/cv_image_nonan.max()
    #cv_image_norm = cv_image_nonan.astype('uint8')

    #print(cv_image_nonan[360,640])
    #print(cv_image_norm[360,640])
    ###
    
    cv2.imshow("Image window", cv_image_bbox_sub)
    cv2.waitKey(3)
    flag=0

    

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
