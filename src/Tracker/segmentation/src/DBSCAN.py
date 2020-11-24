#!/usr/bin/env python3
import os
import sys
import rospy
import cv2

from sensor_msgs.msg import PointCloud2, Image, TimeReference
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

dir_to_Tracker=os.path.dirname(os.path.dirname(os.path.dirname( __file__ )))
dir_to_Scripts = os.path.join(dir_to_Tracker,"Scripts") 
sys.path.append(dir_to_Scripts)

from agfh import *

class Cloud_segmentation:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node("Segmentation")

        print("[INFO] Loading modules...")
        self.bridge = CvBridge()
        
        print("[INFO] Loading config...")
        self.time_list = []
        self.pc_list = []
        self.cv_image = []
        self.queue_lim = 30


        print("[INFO] Initializing ROS publishers...")
        self.bboxes_pub = rospy.Publisher("/Tracker/Segmentation/Boxes",Detection2DArray, queue_size=1) #Bboxes for object_handler 
        
        print("[INFO] Initialize ROS Subscribers...")
        #rospy.Subscriber("/Detection/Boxes",Detection2DArray)
        
        cloud_sub = message_filters.Subscriber("/Tracker/Pc2ToImage/Cloud", Image, queue_size=1)
        timer_sub = message_filters.Subscriber("/Tracker/Timer", TimeReference, queue_size=1)

        print("[INFO] Loading complete")
        mf = message_filters.TimeSynchronizer([cloud_sub,timer_sub],queue_size=30)
        #mf.registerCallback(self.callback_timer)

        self.callback_segmentation()
        """
        #### Test til rapport
        #### Python ####
        pyt_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2, queue_size=1)
        mf_pyt = message_filters.TimeSynchronizer([pyt_sub,timer_sub],queue_size=30)
        mf_pyt.registerCallback(self.callback_pyt)
        #rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_pyt,queue_size=1)#)
        
        
        #### c++ ####
        cpp_sub = message_filters.Subscriber("/Tracker/Pc2ToImage/Cloud",Image, queue_size=1)
        mf_cpp = message_filters.TimeSynchronizer([cpp_sub,timer_sub],queue_size=30)
        mf_cpp.registerCallback(self.callback_cpp)
        #rospy.Subscriber("yolo/CloudImage",Image,self.callback_cpp,queue_size=1)
        """

    def callback_segmentation(self):
        boxes = rospy.wait_for_message("/Tracker/Detection/Boxes",Detection2DArray)
        time1 = rospy.Time.now().to_sec()
        # Making sure that time on pc_list and cv_image is the same as time on the bboxes
        image = False
        i = 0
        for time in self.time_list:
            if time == boxes.header.stamp.to_sec():
                image = True
                #pc_list  = self.pc_list[i]
                cv_image = self.cv_image[i]
                break
            i += 1
        # Maybe add delete of all pc_list and cv image from before time stamp.
        
        if image == True:
            PC_image_bbox_sub_series = []
            for box in boxes.detections:
                PC_image_bbox_sub = cv2.getRectSubPix(cv_image,(int(box.bbox.size_x),int(box.bbox.size_y)),(int(box.bbox.center.x),int(box.bbox.center.y))) # Extract bbox in depth image
                PC_image_bbox_sub_series.append(PC_image_bbox_sub)
            _, segmentation_img, xyzcoord_series, labels_series = DBSCAN_pointcloud(PC_image_bbox_sub_series, boxes.detections)
            for i in range(len(boxes.detections)):
                boxes.detections[i].results[0].pose.pose.position.x = xyzcoord_series[i][0]
                boxes.detections[i].results[0].pose.pose.position.y = xyzcoord_series[i][1]
                boxes.detections[i].results[0].pose.pose.position.z = xyzcoord_series[i][2]
            
            self.bboxes_pub.publish(boxes)
        time2 = rospy.Time.now().to_sec()
        print(time2-time1, "Time DBScan")
        self.callback_segmentation()

    def callback_timer(self,image,timer):
        time1 = rospy.Time.now().to_sec()
        cv_image    = np.array(self.bridge.imgmsg_to_cv2(image)) # Image with distance in channels x,y,z

        time2 = rospy.Time.now().to_sec()
        pc_list     = np.reshape(np.reshape(cv_image,(image.height,image.width*3)).T, image.height*image.width*3)

        time3 = rospy.Time.now().to_sec()
        self.time_list.append(image.header.stamp.to_sec())
        #self.pc_list.append(pc_list)
        self.cv_image.append(cv_image)

        time4 = rospy.Time.now().to_sec()
        if len(self.time_list) > self.queue_lim:
            del self.time_list[0]
            #del self.pc_list[0]
            del self.cv_image[0]
        time5 = rospy.Time.now().to_sec()
        print(time2-time1 , "Time1")
        #print(time3-time2 , "Time2")
        #print(time4-time3 , "Time3")
        #print(time5-time4 , "Time4")

    def callback_cpp(self,image,timer):
        time1 = timer.time_ref.to_sec()
        self.cv_image = np.array(self.bridge.imgmsg_to_cv2(image)) # Image with distance in channels x,y,z
        #self.pc_list  = np.reshape(np.reshape(self.cv_image,(image.height,image.width*3)).T, image.height*image.width*3)
        time2 = rospy.Time.now().to_sec()
        print(time2-time1 , "CPP")

    def callback_pyt(self,cloud,timer):
        time1 = timer.time_ref.to_sec()
        pc_list, cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
        time2 = rospy.Time.now().to_sec()
        print(time2-time1 , "python")
        
        

def main(args):
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)