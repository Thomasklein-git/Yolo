#!/usr/bin/env python3
import sys
import rospy

from sensor_msgs.msg import Image, CompressedImage, TimeReference
#from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Time
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
from yolov3.yolov3 import *

class object_detector:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node('Detection')

        print("[INFO] Loading modules...")
        self.yolo = Load_Yolo_model()
        self.bridge = CvBridge()

        print("[INFO] Loading config...")
        # Create local variables
        self.timer = TimeReference()

        print("[INFO] Initialize ROS publishers...")
        # Create Topics to publish
        self.boxes_pub = rospy.Publisher("/Detection/Boxes",Detection2DArray, queue_size=1)
        self.timer_pub = rospy.Publisher("/Tracker/Timer",TimeReference, queue_size=1)

        print("[INFO] Initialize ROS Subscribers...")
        rospy.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage)
        # Create subscriptions

        print("[INFO] Loading complete")
        # Init callback
        self.callback()

    def callback(self):
        #image = rospy.wait_for_message("/zed2/zed_node/left/image_rect_color",Image)
        image = rospy.wait_for_message("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage)
        time1 = rospy.Time.now().to_sec()
        self.timer.header = image.header
        self.timer.time_ref = rospy.Time.now() 
        self.timer_pub.publish(self.timer)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")
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
            detection.is_tracking = False
            detect.detections.append(detection)

        self.boxes_pub.publish(detect)
        # Reload the callback loop 
        time2 = rospy.Time.now().to_sec()
        print(time2-time1, "Yolo Time")

        self.callback()     

def main(args):
    yolo = object_detector()
    rospy.spin()

if __name__ =='__main__':
	main(sys.argv)