#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import pickle as pkl
import numpy as np

def talker():
	"""
	INSERT DESCRIPTION
	"""
	pub = rospy.Publisher('pose_and_gripper', Float32MultiArray, queue_size=10)
	rospy.init_node('vr_publisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	# load in the trajectory
	# iterate through the elements
	# publish the x,y,z,gripper width as a list?
	pose_traj = pkl.load(open('vr_control/scripts/Pick_and_Place_Motion.p','rb'))

	for elem in pose_traj:
		msg = Float32MultiArray()
		# msg["layout"] = 1
		msg.data = elem
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

	# while not rospy.is_shutdown():
	# 	hello_str = "hello world %s" % rospy.get_time()
	# 	rospy.loginfo(hello_str)
	# 	pub.publish(hello_str)
	# 	rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass