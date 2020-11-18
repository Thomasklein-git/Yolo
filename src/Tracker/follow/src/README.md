# Files
- Base_pose.py
Read the current goal when it is publiced and moves the base to that location.
- Create_goal.py
Creates a list of waypoints to follow in the future and releases them one by one as the vehicle gets close to the current goal. 
- Move_base
Takes care of moving the baseframe with the Vehicle when the Vehicle is moved.
- Publish_pose.py
Publishes a StampedPose responding to a new point to Track.
