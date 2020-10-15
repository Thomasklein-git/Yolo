# Commands
- Open terminal and ssh to tank
```
cd tank_ws/bagfiles
rosbag record -O name node1 node2 node3
```
- Play 
'''
rosbag play name
'''
- Play loop
'''
rosbag play -l name
'''