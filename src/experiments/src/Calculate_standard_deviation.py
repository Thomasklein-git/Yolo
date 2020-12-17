#!/usr/bin/env python3

import rospy
import numpy as np
from vision_msgs.msg import Detection2DArray
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import scipy.stats

class Standard_deviation_cal():
    def __init__(self):
        rospy.init_node("STD")
        self.all_coord=np.array([])
        self.no_seg_boxes=60
        rospy.Subscriber("/Tracker/Segmentation/Boxes", Detection2DArray, self.Append_boxes,queue_size=200)
   
        
    def Append_boxes(self,boxes):
        x=boxes.detections[0].results[0].pose.pose.position.x
        y=boxes.detections[0].results[0].pose.pose.position.y
        z=boxes.detections[0].results[0].pose.pose.position.z

        self.all_coord=np.append(self.all_coord,np.array([x,y,z]),axis=0)
        print("Ready to calculate in (",int(len(self.all_coord)/3),":",self.no_seg_boxes,")",end="\r")
        if len(self.all_coord)==3*self.no_seg_boxes:
            Cal_standard_deviation(self.all_coord)
    
def Cal_standard_deviation(all_coord):
    Coord_reshape=np.transpose(np.reshape(all_coord,(-1,3))) #shape=[3,no_seg_boxes]
    #save_to_file("test.csv",Coord_reshape[0])
    x=pd.Series(Coord_reshape[0])
    y=pd.Series(Coord_reshape[1])
    z=pd.Series(Coord_reshape[2])
    results_x=x.describe()
    results_y=y.describe()
    results_z=z.describe()
    print("****Results for x****")
    print(results_x)
    print("****Results for y****")
    print(results_y)
    print("****Results for z****")
    print(results_z)

    conf_x=mean_confidence_interval(x,confidence=0.95)
    conf_y=mean_confidence_interval(y,confidence=0.95)
    conf_z=mean_confidence_interval(z,confidence=0.95)
    print(conf_x,"conf_x")
    print(conf_y,"conf_y")
    print(conf_z,"conf_z")

    fig = plt.figure()
    gs = gridspec.GridSpec(3, 1, figure=fig)
    plt.title('Boxplot of x, y, z coordinates')
    #plt.title('Boxplot of x coordinates')
    plt.axis('off')
    ax = fig.add_subplot(gs[0, 0])
    ax.boxplot((x), vert=False, showmeans=True, meanline=True,
           labels=('x'), patch_artist=False, widths=0.9,
           medianprops={'linewidth': 2, 'color': 'purple'},
           meanprops={'linewidth': 2, 'color': 'red'})
    ax.axvline(conf_x[1],linewidth=2, color='darkorange') #min_conf
    ax.axvline(conf_x[2],linewidth=2, color='darkorange') #max_conf
    
    ay = fig.add_subplot(gs[1,0])
    ay.boxplot((y), vert=False, showmeans=True, meanline=True,
           labels=('y'), patch_artist=False, widths=0.9,
           medianprops={'linewidth': 2, 'color': 'purple'},
           meanprops={'linewidth': 2, 'color': 'red'})
    ay.axvline(conf_y[1],linewidth=2, color='darkorange') #min_conf
    ay.axvline(conf_y[2],linewidth=2, color='darkorange') #max_conf
    
    az = fig.add_subplot(gs[2,0])
    az.boxplot((z), vert=False, showmeans=True, meanline=True,
           labels=('z'), patch_artist=False, widths=0.9,
           medianprops={'linewidth': 2, 'color': 'purple'},
           meanprops={'linewidth': 2, 'color': 'red'})
    az.axvline(conf_z[1],linewidth=2, color='darkorange') #min_conf
    az.axvline(conf_z[2],linewidth=2, color='darkorange') #max_conf
    
    plt.show()
    rospy.signal_shutdown("reason")

def mean_confidence_interval(data, confidence=0.95):
    # https://www.kite.com/python/examples/702/scipy-compute-a-confidence-interval-from-a-dataset
    n = len(data)
    m, se = np.mean(data), scipy.stats.sem(data) # Cal mean and standard error of the mean
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h

def save_to_file(name,text):
    with open(name, mode='wt', encoding='utf-8') as myfile:
        #for lines in text:
        myfile.write(str(text))

if __name__ == '__main__':
    try:
        Standard_deviation_cal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass