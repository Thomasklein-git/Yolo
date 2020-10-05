#!/usr/bin/env python3
from collections import OrderedDict
import numpy as np

from yolov3.utils import read_class_names
from yolov3.configs import *

class ObjectIdentifier():
    def __init__(self):
        self.uid = []
        self.classNum = []
        self.score =[]
        self.centroid_x = []
        self.centroid_y = []
        self.OcclusionCount = 0
        self.state = []
    
    def new_object(self,bbox):
        self.classNum = bbox[5]
        self.score = bbox[4]
        self.centroid_x.append(int((bbox[0] + bbox[2]) / 2.0))
        self.centroid_y.append(int((bbox[1] + bbox[3]) / 2.0))
        self.state = "Visible"

    def update_object


    def V2D(object):
        self.state = "Ocludded"

    def D2L(object):
        self.state = "Lost"
        



class IDtracker():
    def __init__(self):
        # Initialize first object number
        self.nextObjectID = 0

        # Find class names and number of classes from model
        self.classes = read_class_names(YOLO_COCO_CLASSES)
        self.num_classes = len(self.classes)

        # Ordered dics keep track of the insertion order of the inputs to each dict
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()

        self.MaxMissedFrames = 50

    #def register(rects):

    #def deregister(self, objectID):

    #def update(self, bboxes):
        # bboxes contain (x_start, y_start, x_end, y_end, score, class)
        # Checking number of unique classes in current image

        #current_objects = np.zeros((len(bboxes),4), dtype=object)
        #i = 0
        #for row in bboxes:
        #    cX = int((row[0]+row[2]) / 2.0)
        #    cY = int((row[1]+row[3]) / 2.0)
        #    current_objects[i] =(cX,cY,row[4],int(row[5]))
        #    i += 1
            #np.append(current_objects,[cX,cY,row[4],int(row[5])],axis=0) 
        
        # Compare number of currently found boxes with the stored number of boxes. 
        

        # Compare number of inputs with number of each class in stored data





