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

from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
from collections import Counter

#from Trig import Simple_Pinhole, Advanced_Pinhole

def DBSCAN_pointcloud(img, eps=0.5):
    """
    img is point cloud image with three channels corresponding to x, y and z coordinate for each pixel.
    It must have the shape of [w,h,3]
    Coordinate system according to ZED2
    """
    imgre=img.reshape((-1,3)) # Flatten the image (pixel,3)
    imgre_wo_nan = imgre[~np.isnan(imgre).any(axis=1)] # remove rows with nan
    #imgre_wo_nan_and_inf = imgre_wo_nan[~np.isinf(imgre_wo_nan).any(axis=1)] # remove rows with inf
    imgre_scale = StandardScaler().fit_transform(imgre_wo_nan)

    DB_scale = DBSCAN(eps=eps,min_samples=5).fit(imgre_scale)
    # Calculation of average depth
    label=DB_scale.labels_
    label=np.transpose(np.asarray([label])) # In order to concatenate shape[label,1]
    Sort=Counter(label.flatten()).most_common() 
    print(Sort)
    """
    label_max=Sort[0][0]
    print(label.shape)
    
    # Extraxt depth data from point cloud
    xcoord=[] 
    for x in range(len(imgre_wo_nan)):
        xcoord.append(imgre_wo_nan[x][0])
    xcoord=np.transpose(np.array([xcoord])) # shape[xcoord,1] 
    print(xcoord.shape)
    
    a=np.concatenate((label,xcoord),axis=1) # Put label and xcoord (depth) side by side shape[pixel,2]
    print(a.shape)
    
    b=[] # Depth values from a which is at label_max
    for i in range(len(a)):
        if a[i,0]==label_max:
            b.append(a[i,1])
    avg_depth=np.mean(b)
    """
    avg_depth=1
    return avg_depth

class Depth_Comparison():
    def __init__(self):
        global yolo
        self.show=0 # 0: don't show 1: show
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
                avg_depth_kmeans=k_means_pointcloud(cv_image_bbox_sub)
                print(avg_depth_kmeans, "kmeans")
                avg_depth_kmeans2=k_means_pointcloud(cv_image_bbox_sub,k=2)
                print(avg_depth_kmeans2, "kmeans2")
                #avg_depth_DBSCAN=DBSCAN_pointcloud(cv_image_bbox_sub)
                #print(avg_depth_DBSCAN,"DBSCAN")

        
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