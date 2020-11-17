# Commands
- Open terminal and ssh to tank
```
cd tank_ws/bagfiles
rosbag record -O name node1 node2 node3
```
- Play 
```
rosbag play name
```
- Play loop
```
rosbag play -l name
```
- Play one topic
```
rosbag play name --topics /topic
```
- Record limit number of messages from topic
```
rosbag record -O LimitTest.bag -l 1 /zed2/zed_node/left/image_rect_color
```

- Record for standard deviation experiment
```
rosbag record -O test.bag -l 30 /zed2/zed_node/left/image_rect_color/compressed /zed2/zed_node/point_cloud/cloud_registered
```
