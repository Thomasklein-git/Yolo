#!/usr/bin/env python3

import cv2
import rospy
import sys
#from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans
from collections import Counter


import roslib
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
#from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
#from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
#from yolov3.configs import *
#from Trig import Simple_Pinhole, Advanced_Pinhole

def k_means_pointcloud(x,y,z,k=3,max_iter=1000,tol=1e-4):
    """
    xyz are xyz-coordinates for each pixel. It must have the shape of [nr of pixels,]
    Coordinate system according to ZED2
    """
    xyz=[x,y,z]
    xyz=np.transpose(xyz)
    xyz_scale = StandardScaler().fit_transform(xyz)
    KM_scale = KMeans(n_clusters=k, max_iter=max_iter, tol=tol, random_state=0).fit(xyz_scale)
    # Calculation of average depth
    label=KM_scale.labels_
    label=np.transpose(np.asarray([label])) # In order to concatenate shape[label,1]
    Sort=Counter(label.flatten()).most_common() 
    label_max=Sort[0][0]
    
    x=np.array([x]) # shape[1,x] 
    x=np.transpose(x) # shape [x,1]
    a=np.concatenate((label,x),axis=1) # Put label and x (depth) side by side shape[pixel,2]
    b=[] # Depth values from a which is at label_max
    for i in range(len(a)):
        if a[i,0]==label_max:
            b.append(a[i,1])
    avg_depth=np.mean(b)

    return avg_depth

class Depth_Comparison():
    def __init__(self):
        #global yolo
        #self.show=1 # 0: don't show 1: show
        #self.active=1
        #self.cam_active=1
        #self.cv_image_cam=[]
        #self.cv_image_depth=[]
        #self.image_pub = rospy.Publisher("image_topic_2",Image)
        #self.bridge = CvBridge()
        #self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
        #self.image_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
        self.Cloud_sub = rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_cloud)

        #yolo = Load_Yolo_model()
        
    def callback_cloud(self,data):
        image_height=720
        image_width=1280
        pc = pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z"))
        pc_list = []
        xcoord=[]
        ycoord=[]
        zcoord=[]
        for p in pc:
            pc_list.append( [p[0],p[1],p[2]])
        print(pc_list[0:2],"list")
        pc_list=np.array(pc_list)
        print(pc_list.shape, "pc_list")
        print(pc_list[0:2])
        pc_image=pc_list.reshape((image_height,image_width,3))
        print(pc_image.shape)
        for x in range(len(pc_list)):
            xcoord.append(pc_list[x][0])
        for y in range(len(pc_list)):
            ycoord.append(pc_list[y][1])
        for z in range(len(pc_list)):
            zcoord.append(pc_list[z][2])
        
        #avg_depth=k_means_pointcloud(xcoord,ycoord,zcoord,k=3,max_iter=1000,tol=1e-4)
        #print(avg_depth)
        



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