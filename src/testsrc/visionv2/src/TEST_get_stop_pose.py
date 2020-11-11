from geometry_msgs.msg import PoseStamped
from agfh import *
import numpy as np

def cal_pose_stop(pose_goal,pose_vehicle,distance_keep):
    vec_v2g=pose_goal-pose_vehicle #vec from vehicle to goal
    vec_v2g_stop=vec_v2g/np.linalg.norm(vec_v2g)*distance_keep #vector from goal to stop
    pose_goal_stop=pose_goal-vec_v2g_stop
    return pose_goal_stop

Waypoints=[]
Waypoints1=Create_PoseStamped_msg([8,5,0],"hej",1)
Waypoints2=Create_PoseStamped_msg([1,1,0],"hej",1)

Waypoints.append(Waypoints1)
Waypoints.append(Waypoints2)


Goal_m=Waypoints[0]
print(Goal_m,"Goal")
Pose=Create_PoseStamped_msg([6,1,0],"hej",1)

vec_Goal_map = np.array([Goal_m.pose.position.x,Goal_m.pose.position.y])
vec_Vehicle_map = np.array([Pose.pose.position.x,Pose.pose.position.y])
distance_keep=np.sqrt(2**2+1**2)
xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,distance_keep)

Waypoints[0].pose.position.x=xy_goal_stop[0]
Waypoints[0].pose.position.y=xy_goal_stop[1]
print(Waypoints[0],"stop waypoint")