#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2DArray

class Standard_deviation_cal():
    def __init__(self):
        #rospy.init_node('Create_goal')
        rospy.Subscriber("/Tracker/Segmentation/Boxes", Detection2DArray, self.Append_boxes,queue_size=200)
        
    def Append_boxes(self,boxes):
        all_coord=[]
        x=boxes.detections[0].results[0].pose.pose.position.x
        print(x)


if __name__ == '__main__':
    try:
        Standard_deviation_cal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass