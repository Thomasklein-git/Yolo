# IPs
klein
```
10.122.12.69
```
Jakob
```
10.122.12.25
```

ubuntu
```
10.122.12.159
raspberry
```
tank
```
10.122.12.106
thomas
```
# Access to "tank"
```
ssh tank@10.122.12.106
```
Pass: thomas
```
export ROS_MASTER_URI=http://10.122.12.106:11311
export ROS_IP=10.122.12.106
```
Start Zed2 
```
cd tank_ws
roslaunch zed_wrapper zed2.launch
```

# Yolo

## To start Vision.py from webcam
- Open terminal
```
source /opt/ros/noetic/setup.bash
roslaunch usb_cam usb_cam-test.launch
```
- Open another terminal
```
cd yolo_ws
source devel/setup.bash
rosrun visionv2 Vision.py
```
