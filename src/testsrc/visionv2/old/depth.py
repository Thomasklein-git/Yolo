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
from yolov3.utils import detect_image, Load_Yolo_model#, Give_boundingbox_coor_class, k_means_depth
from agfh import k_means_depth, Give_boundingbox_coor_class
from yolov3.configs import *

import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class image_converter:

  def __init__(self):
    global yolo
    self.show=0 # 0: don't show 1: show
    self.dep_active=0
    self.cal_active=0
    self.cv_image_cam=[]
    self.cv_image_depth=[]
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
    self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
    yolo = Load_Yolo_model()
 
  def callback_cam(self,data):
    if self.dep_active==1:
      self.cal_active = 1
      try:
        self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
        #print(self.cv_image_cam.shape,"cam")
      except CvBridgeError as e:
        print(e)

      imagecv_cam,cv_image_bbox_sub,bboxes,img_seg=self.calculation()
      #cv2.imwrite("depth2.png",(cv_image_bbox_sub*2**16).astype(np.uint16))
      self.cal_active=0
      self.dep_active = 0
      if self.show==1:
        cv2.imshow("Image cam window", imagecv_cam)
        #cv2.imshow("Image seg",img_seg)
        #for i in range(len(bboxes)):
        #  cv2.imshow("Image depth window"+str(i),cv_image_bbox_sub[i])
        cv2.waitKey(3)
      
  
  def callback_depth(self,data):
    if self.cal_active==0:
      try:
        self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
        #print(self.cv_image_depth.shape,"depth")
      except CvBridgeError as e:
        print(e)
      self.dep_active=1
    
  def calculation(self):
    imagecv_cam=self.cv_image_cam
    imagecv_depth=self.cv_image_depth
    imagecv_depth_series=[]
    img_seg=[]
    if len(imagecv_cam)  != 0:
      imagecv_cam, bboxes=detect_image(yolo, imagecv_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
      x1, y1, x2, y2, _, C = Give_boundingbox_coor_class(bboxes)
      print("Bounding box of object(s) = ",x1,y1,x2,y2,C)
    if len(imagecv_depth) != 0:
      for i in range(len(bboxes)):
        patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
        center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
        cv_image_bbox_sub = cv2.getRectSubPix(imagecv_depth,patch,center) # Extract bbox in depth image
        cv_image_bbox_sub = np.where(np.isnan(cv_image_bbox_sub),0.3, cv_image_bbox_sub) # set nan to 0
        cv_image_bbox_sub = np.where(np.isinf(cv_image_bbox_sub),0.3, cv_image_bbox_sub) # set +/-inf to 0
        #print(cv_image_bbox_sub)
        #print(imagecv_depth.dtype)
        #avg_depth,img_seg=k_means_depth(cv_image_bbox_sub)
        avg_depth,img_seg=k_means_depth(cv_image_bbox_sub)
        print("Average depth of object [m]=", avg_depth)
        #cv2.imwrite("depth2"+str(i)+".png",(cv_image_bbox_sub*2**16).astype(np.uint16)) #safe images
        #cv2.imwrite("depthclean"+str(i)+".png",cv_image_bbox_sub)
        #cv2.imwrite("depthseg"+str(i)+".png",img_seg)
        print(imagecv_depth.shape)

        D_to_C_of_bbox_L=cv_image_bbox_sub[int(patch[1]/2),int(patch[0]/2)] #height (y), width (x) gives distance to center coordinate of bbox with resprct to local
        print("Distance to center of object [m]= ",D_to_C_of_bbox_L," Class number= ", C[i])
        #Distance_to_center_of_bbox_wrt_global=imagecv_depth[center[1],center[0]] #height (y), width (x)
        #print(Distance_to_center_of_bbox_wrt_global) 
        
        """
        ## For plotting
        max_depth=20 #Maximum depth the camera can detect objects [m]
        cv_image_bbox_sub *= 255/(max_depth/cv_image_bbox_sub) # relate meter to pixels
        cv_image_bbox_sub = cv_image_bbox_sub.astype('uint8') # pixel float to int
        D_to_C_of_bbox_L_p=cv_image_bbox_sub[int(patch[1]/2),int(patch[0]/2)]
        #print(D_to_C_of_bbox_L_p)
        # 1 pixel is approx 7.5 cm
        cv_image_bbox_sub = (np.where(cv_image_bbox_sub>(D_to_C_of_bbox_L_p+1),0,(np.where(cv_image_bbox_sub<(D_to_C_of_bbox_L_p-1),0,cv_image_bbox_sub)))) #makes every pixel black if outside thresshole
        cv_image_bbox_sub = np.where(cv_image_bbox_sub>(D_to_C_of_bbox_L_p-2),255,cv_image_bbox_sub)
        #(cv_image_bbox_sub.shape)
        """
        imagecv_depth_series.append(cv_image_bbox_sub)
    return imagecv_cam, imagecv_depth_series, bboxes, img_seg
 

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