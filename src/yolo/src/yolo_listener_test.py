#!/usr/bin/env python3
import sys
import rospy
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class yolo_listener:
    def __init__(self):
        self.bridge = CvBridge()
        image_sub  = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
        detect_sub = message_filters.Subscriber("/yolo/bboxes",Detection2DArray)

        #mf = message_filters.ApproximateTimeSynchronizer([image_sub,detect_sub],10,0.07)
        mf = message_filters.TimeSynchronizer([image_sub,detect_sub],100)
        mf.registerCallback(self.callback)



    def callback(self, image, detect):
        cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
        print("msg")
        """
        for Object in detect.detections:
            Start_x = Object.bbox.center.x - Object.bbox.size_x/2
            End_x   = Object.bbox.center.x + Object.bbox.size_x/2
            Start_y = Object.bbox.center.y - Object.bbox.size_y/2
            End_y   = Object.bbox.center.y + Object.bbox.size_y/2

            cv2.rectangle(cv_image, (int(Start_x), int(Start_y)), (int(End_x), int(End_y)), (0, 0, 255), 2)
        cv2.imshow("Image_window", cv_image)
        cv2.waitKey(1)
        """
        




    


def main(args):
	rospy.init_node('YOLO_listener', anonymous=True)
	yolo = yolo_listener()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)