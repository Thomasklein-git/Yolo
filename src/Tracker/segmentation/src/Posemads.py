#!/usr/bin/env python3
import os
import sys
import rospy
import cv2

from sensor_msgs.msg import PointCloud2, Image, TimeReference
from geometry_msgs.msg import PoseStamped
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
        self.Poses = []
        self.queue_lim = 60


        print("[INFO] Initializing ROS publishers...")
        self.bboxes_pub = rospy.Publisher("/Tracker/Segmentation/Boxes",Detection2DArray, queue_size=1) #Bboxes for object_handler 
        
        print("[INFO] Initialize ROS Subscribers...")
        #rospy.Subscriber("/Detection/Boxes",Detection2DArray)
        Pose_sub = message_filters.Subscriber("/vehicle_position", PoseStamped, queue_size=1)
        boxes_sub = message_filters.Subscriber("/Tracker/Detection/Boxes",Detection2DArray, queue_size=1)

        mf = message_filters.TimeSynchronizer([Pose_sub,boxes_sub],queue_size=30)
        mf.registerCallback(self.callback)

        #self.callback_segmentation()

    def callback(self,Pose,boxes):
        for i in range(len(boxes.detections)):
                boxes.detections[i].results[0].pose.pose.position.x = Pose.pose.position.x
                boxes.detections[i].results[0].pose.pose.position.y = Pose.pose.position.y
                boxes.detections[i].results[0].pose.pose.position.z = Pose.pose.position.z
        self.bboxes_pub.publish(boxes)


    def callback_segmentation(self):
        boxes = rospy.wait_for_message("/Tracker/Detection/Boxes",Detection2DArray)

        # Making sure that time on pc_list and cv_image is the same as time on the bboxes
        image = False
        i = 0
        for time in self.time_list:
            if time == boxes.header.stamp.to_sec():
                image = True
                #pc_list  = self.pc_list[i]
                cv_image = self.Poses[i]
                break
            i += 1
        # Maybe add delete of all pc_list and cv image from before time stamp.
        if len(self.cv_image) > 0:
            cv_image = self.cv_image[-1]
            image = True
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
        else:
            print("No match")
        self.callback_segmentation()

    def callback_timer(self,Pose,timer):
        Poses       = [Pose.position.x,Pose.position.y, Pose.position.z]

        #cv_image    = np.array(self.bridge.imgmsg_to_cv2(image)) # Image with distance in channels x,y,z
        #pc_list     = np.reshape(np.reshape(cv_image,(image.height,image.width*3)).T, image.height*image.width*3)

        self.time_list.append(Pose.header.stamp.to_sec())
        #self.pc_list.append(pc_list)
        self.Poses.append(Poses)

        if len(self.time_list) > self.queue_lim:
            del self.time_list[0]
            #del self.pc_list[0]
            del self.Poses[0]
        print("timer callback")
        

def main(args):
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)