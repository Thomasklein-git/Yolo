# Create new packages
catkin_create_pkg trackerv3 cv_bridge roslib rospy sensor_msgs std_msgs stereo_msgs

# Versions
- Trackerv1

This tracker version is based on "https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/" which uses the detection CNN Caffe. The tracking algorithm is the shortest distance between centroids. Can handle one class only (face).
```
roslaunch usb_cam usb_cam-test.launch
rosrun tracker Trackerv1.py
```
- Trackerv2

This tracker version uses YOLOv3 CNN for detecting objects. The tracking algorithm is based on the shortest distance between in class controids. Can handle multiple classes. Uses wieghts from COCO (80 classes)
```
roslaunch usb_cam usb_cam-test.launch
rosrun trackerv2 Trackerv2.py
```
- Trackerv3

This tracker version uses YOLOv3 CNN for detecting objects. The tracker uses depth data to calculate the shortest distance between in class centroids in 3D. Can handle multiple classes. Uses wieghts from COCO (80 classes)
```
roslaunch
rosrun trackerv3 Trackerv3.py
```
