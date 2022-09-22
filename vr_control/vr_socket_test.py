import pickle as pkl
import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy
import UdpComms as U
import time

# NOTE: In order to turn this script into live-immitating the VR controller pose/gripper width, we would need
# a subscriber in here that loads in the trajectory step by step. Instead of iterating through pose_traj in a 
# for loop, would have a while loop for while getting commands from VR ros node (or whatever we call it)

# NOTE: It actually would be helpful to get more steps in the trajectory, as one of the issues is that the 
# gripper closing can lag behind because the step size between trajectory points is quite large (e.g. 0.8 to 
# 0.6 in one step), which since the gripper and pose are decoupled can cause the timings to mis-align

if __name__ == "__main__":

	fa = FrankaArm()
	fa.reset_joints()

	pose = fa.get_pose()
	print("Robot Resting Pose: ", pose)

	print('start socket')
	#change IP
	sock = U.UdpComms(udpIP="192.168.137.46", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)

	i = 0
	dt = 0.02
	rate = rospy.Rate(1 / dt)
	pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
	T = 100

	fa = FrankaArm()
	fa.reset_joints()
	pose = fa.get_pose()

	fa.goto_gripper(0, grasp=True)

	reset = False

	while True:
		sock.SendData('Sent from Python: ' + str(i)) # Send this string to other application
		
		data = sock.ReadReceivedData() # read data

		if data != None: # if NEW data has been received since last ReadReceivedData function call
			print(data)
			position, goal_width = data.split('\t')
			position = np.array(position[1:-1].split(', ')).astype(np.float)
			position = np.array([position[2] + 0.6, -position[0], position[1] + 0.02])
			goal_width = float(goal_width)
			pose.translation = position

			if not reset:
				fa.goto_pose(pose)
				fa.goto_pose(pose, duration=T, dynamic=True, buffer_time=10,
					cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0])
				reset = True

				init_time = rospy.Time.now().to_time()
			else:
				timestamp = rospy.Time.now().to_time() - init_time
				traj_gen_proto_msg = PosePositionSensorMessage(
					id=i, timestamp=timestamp,
					position=pose.translation, quaternion=pose.quaternion
				)
				ros_msg = make_sensor_group_msg(
					trajectory_generator_sensor_msg=sensor_proto2ros_msg(
						traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
					)

				rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
				pub.publish(ros_msg)
				rate.sleep()
			# fa.goto_pose(pose)

			i+=1
			#goal_width = 2*float(data)
			# fa.goto_gripper(goal_width, block=False, grasp=False, speed=0.1)
			# print("\nDesired Gripper Width: ", goal_width, "Current Gripper State: ", fa.get_gripper_width())
			# time.sleep(1)

			# SOLUTION BRAINSTORM:
				# Current Problems: Latent delay, when grasp=True there is an overshoot
				# and gripper oscillation (with speed fixed to 0.15 this is particularly bad)

				# try terminating grasp action before sending another goto_gripper command
				# have a check if object in view before grasp and only if so, then use grasp = True
				# have x + delta_t * v to scale the desired gripper width to account for latency 

			# min_gripper_delay = 0.2
			# gripper_update_rate = 1 + int(min_gripper_delay/dt)
			# if i % gripper_update_rate == 0:
			# 	difference = goal_width - fa.get_gripper_width()
			# 	speed = abs(difference/(dt*gripper_update_rate ))
			# 	# print("\nSpeed: ", speed)
			# 	if speed > 0.15:
			# 	    speed = 0.15
			# 	# speed = 0.15
			# 	if abs(difference) > 0.001:
			# 		if difference >= 0:
			# 			# print("opening")
			# 			fa.goto_gripper(goal_width, block=False, grasp=False, speed=speed)
			# 		else:
			# 			# print("closing")
			# 			fa.goto_gripper(goal_width, block=False, grasp=False, speed=speed)
			# 		print("i:  ", i, "   Desired Gripper Width: ", goal_width, "Current Gripper State: ", fa.get_gripper_width())
				# else:
					# print("do nothing :)")

			# rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
			# pub.publish(ros_msg)
			rate.sleep()
