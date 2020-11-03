#!/usr/bin/env python3
import sys
import rospy
import cv2

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError

from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
from yolov3.yolov3 import *

class object_detector:
    def __init__(self):
        self.yolo = Load_Yolo_model()

        self.bridge = CvBridge()
        self.bbox_pub = rospy.Publisher("/yolo/bboxes",Detection2DArray, queue_size=1)
        rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_yolo)

    def callback_yolo(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)

        _ , bboxes=detect_image(self.yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))

        detections = []
        for Object in bboxes:
            detection = Detection2D()
            hypo = ObjectHypothesisWithPose()
            #Start x
            x1 = Object[0]
            #End x
            x2 = Object[2]
            #Start y
            y1 = Object[1]
            #end y
            y2 = Object[3]

            #Size x
            Sx = x2-x1
            #Size y
            Sy = y2-y1
            #Center x
            Cx = x1+Sx/2
            #Center y
            Cy = y1+Sy/2

            detection.bbox.center.x = Cx
            detection.bbox.center.y = Cy
            detection.bbox.size_x   = Sx
            detection.bbox.size_y   = Sy

            #Class probability
            
            hypo.score = Object[4]
            #Class ID
            hypo.id    = Object[5]

            detection.results = hypo
            detections.append(detection)

        detect = Detection2DArray()
        detect.header = image.header
        detect.detections = detections
        self.bbox_pub.publish(detect)




def main(args):
	rospy.init_node('YOLO', anonymous=True)
	yolo = object_detector()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)