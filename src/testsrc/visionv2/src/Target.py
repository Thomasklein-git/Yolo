#!/usr/bin/env python3

import sys
import os
import rospy

from std_srvs.srv import SetBool

class Choose_target():
    def __init__(self):
        rospy.init_node("Choose_target")

        rospy.Service("Choose_target_service",SetBool,self.callback_server)

    def callback_server(self):
        rospy.set_param(/Target_UID)
        print("param changed")
        


def main(args):
	Choose_target()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
        try:
            rospy.delete_param(/Target_UID)
        except KeyError:
            print("No parameter named /Target_UID")

		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)