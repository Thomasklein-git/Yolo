from geometry_msgs.msg import PoseStamped
from agfh import *

P_old=Create_PoseStamped_msg([4,3,0],"hej",1)
P_new=Create_PoseStamped_msg([2,5,0],"hej",1)
print(np.array([P_old.pose.position.x,P_old.pose.position.y,P_old.pose.position.z]))
#vec_old_new=np.array([-2,2,0])
vec_old_new=0
#q=get_new_orientation(0,0,vec_old_new,Points=False)
q=get_new_orientation(P_old,P_new,vec_old_new,Points=True)
print(q)

P_new_ori=P_new
P_new_ori.pose.orientation.x=q[0]
P_new_ori.pose.orientation.y=q[1]
P_new_ori.pose.orientation.z=q[2]
P_new_ori.pose.orientation.w=q[3]
print(P_new_ori)