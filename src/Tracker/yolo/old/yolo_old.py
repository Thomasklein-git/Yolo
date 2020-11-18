#!/usr/bin/env python3
import sys
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
from yolov3.yolov3 import *

class object_detector:
    def __init__(self):
        self.yolo = Load_Yolo_model()
        self.bridge = CvBridge()

        self.bbox_pub = rospy.Publisher("/yolo/bboxes",Detection2DArray, queue_size=1)
        self.time_pub = rospy.Publisher("/yolo/Timer",Image, queue_size=1,tcp_nodelay=True)

        rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_images)
        rospy.Subscriber("/yolo/Time",Image,self.callback_yolo,tcp_nodelay=True)

    def callback_images(self, image):
        self.image = image
        self.time_pub.publish()

    
    def callback_yolo(self, image):
        image = self.image
        cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)

        _ , bboxes=detect_image(self.yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
        detect = Detection2DArray()
        detect.header = image.header

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


            hypo.id = int(Object[5])
            hypo.score = Object[4]


            detection.results = [hypo,]
            detect.detections.append(detection)

        self.bbox_pub.publish(detect)

def main(args):
	rospy.init_node('YOLO', anonymous=True)
	yolo = object_detector()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)