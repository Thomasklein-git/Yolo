# Yolo

## To start Vision.py from webcam
# Open terminal
source /opt/ros/noetic/setup.bash
roslaunch usb_cam usb_cam-test.launch
# Open another terminal
cd yolo/ws
source devel/setup.bash
rosrun visionv2 Vision.py
