#!/usr/bin/env python3

import cv2
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.point_cloud2 as pc2


import roslib
#from std_msgs.msg import String
from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
#from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *
from agfh import k_means_pointcloud
#from Trig import Simple_Pinhole, Advanced_Pinhole

class Depth_Comparison():
    def __init__(self):
        global yolo
        self.show=1 # 0: don't show 1: show
        self.pc_active=0
        self.cal_active=0
        self.cv_image_cam=[]
        #self.cv_image_depth=[]
        #self.pc=[]
        self.pc_image=[]
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
        #self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
        self.Cloud_sub = rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_cloud)

        yolo = Load_Yolo_model()
    
    def callback_cam(self,data):
        if self.pc_active==1:
            self.cal_active = 1
            try:
                self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
                #print(self.cv_image_cam.shape,"cam")
            except CvBridgeError as e:
                print(e)

            imagecv_cam=self.calculation()
            #cv2.imwrite("depth2.png",(cv_image_bbox_sub*2**16).astype(np.uint16))
            self.cal_active=0
            self.pc_active = 0
            if self.show==1:
                cv2.imshow("Image cam window", imagecv_cam)
                cv2.waitKey(3)

    def callback_cloud(self,data):
        if self.cal_active==0:
            image_height=720
            image_width=1280
            pc = pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z"))
            pc_list = []
            for p in pc:
                pc_list.append( [p[0],p[1],p[2]])
            pc_list=np.array(pc_list, dtype='float32') # Othervise cv2.getRectSubPix
            self.pc_image=pc_list.reshape((image_height,image_width,3))
            self.pc_active=1
 
    
    def calculation(self):
        imagecv_cam=self.cv_image_cam
        pc_image=self.pc_image
        if len(imagecv_cam)  != 0:
            imagecv_cam, bboxes=detect_image(yolo, imagecv_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
            x1, y1, x2, y2, _, C = Give_boundingbox_coor_class(bboxes)
            print("Bounding box of object(s) = ",x1,y1,x2,y2,C)
        
        if len(pc_image) != 0:
            for i in range(len(bboxes)):
                patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
                center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
                cv_image_bbox_sub = cv2.getRectSubPix(pc_image,patch,center) # Extract bbox in depth image
                avg_depth=k_means_pointcloud(cv_image_bbox_sub)
                print(avg_depth)

        
        return imagecv_cam#, imagecv_depth_series, bboxes, img_seg    

def main(args):
    DC = Depth_Comparison()
    
    rospy.init_node('Depth_Comparison', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows

if __name__ =='__main__':
    main(sys.argv)