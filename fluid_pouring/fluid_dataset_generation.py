import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import queue
import time
import os
from frankapy import FrankaArm

# want to have two threads:
	# 1) robot moves through a series of translation and rotation
	# 2) camera records video stream and saves raw images to dataset (each image labeled 1-N for each trajectory cycle)

# Dataset format:
	# each folder named the container
		# contains folder named "run_1" .... "run_N"
			# contains images of the container with liquid at each step in the trajectory

# want the static camera to save the images from the stream every second?

# Variables to vary for this data collection:
	# vessel
	# how much water
	# how the robot is holding the bottle (i.e. grabbing the cap, grabbing the middle, etc.)
	# the trajectories the robot moves the bottle along


# control thread:

# mkdir "run_N"
# cycle through different motions:
	# move in a straight line forward/backward
	# move up and down
	# rotate the angle of the wrist in both directions
	# spin bottle
	# twist at various angles


def vision_loop(moving_queue):
	base = os.path.dirname(os.path.realpath(__file__))
	save_path = '/Dataset'
	save_path = base + save_path

	directory_name = 'run_' + str(9)
	dir_path = os.path.join(save_path, directory_name)

	if not os.path.exists(dir_path):
		os.mkdir(dir_path)

	# ------ Configure depth an color streams ------
	W = 848
	H = 480

	# Side mounted camera
	pipeline = rs.pipeline()
	config = rs.config()
	config.enable_device('151322069488')
	config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
	pipeline.start(config)

	# align stream
	aligned_stream = rs.align(rs.stream.color)

	robot_moving = True
	i = 1

	while robot_moving:
		frames = pipeline.wait_for_frames()
		frames = aligned_stream.process(frames)
		color_frame = frames.get_color_frame()
		depth_frame = frames.get_depth_frame().as_depth_frame()
		color_image = np.asanyarray(color_frame.get_data())

		# robot_moving = moving_queue.get()

		# if robot_moving != False:
		# 	robot_moving = True

		if i % 30 == 0:
			filename = '/img' + str(int(i/30)) + '.png'
			file_path = dir_path + filename
			cv2.imwrite(file_path, color_image)

		i+=1

		# Show the images
		cv2.imshow("Side Image", color_image)
		cv2.waitKey(1)
		
	print("Exited while loop!")

	# TODO: create a termination handler to automatically tell the vision system to stop after the control loop finishes

def control_loop(fa, moving_queue):
	pose = fa.get_pose()

	# # -------- Pouring Hard Coded for Data Collection ---------
	# # test next position
	# pose.translation = np.array([0.7, 0.2, 0.5])
	# fa.goto_pose(pose)

	# # rotate wrist forward
	# joints = fa.get_joints()
	# joints[5] += np.deg2rad(20)
	# fa.goto_joints(joints, ignore_virtual_walls=True)

	# # rotate wrist backward
	# joints[5] -= np.deg2rad(40)
	# fa.goto_joints(joints, ignore_virtual_walls=True)

	# # rotate wrist backward more
	# joints[5] -= np.deg2rad(40)
	# fa.goto_joints(joints, ignore_virtual_walls=True)




	# # -------- Translation and Rotation for Data Collection -------
	# # test next position
	# pose.translation = np.array([0.75, 0.2, 0.35])
	# fa.goto_pose(pose)

	# # test next position 
	# pose.translation = np.array([0.5, 0.2, 0.35])
	# fa.goto_pose(pose)

	# # go back to center
	# pose.translation = np.array([0.5, 0.2, 0.35])
	# fa.goto_pose(pose)

	# # move up
	# pose.translation = np.array([0.5, 0.2, 0.5])
	# fa.goto_pose(pose)

	# # move down
	# pose.translation = np.array([0.5, 0.2, 0.3])
	# fa.goto_pose(pose)

	# # go back to center
	# pose.translation = np.array([0.5, 0.2, 0.35])
	# fa.goto_pose(pose)

	# # rotate wrist forward
	# joints = fa.get_joints()
	# joints[5] += np.deg2rad(20)
	# fa.goto_joints(joints, ignore_virtual_walls=True)

	# # # rotate wrist forward more
	# # joints = fa.get_joints()
	# # joints[5] += np.deg2rad(20)
	# # fa.goto_joints(joints, ignore_virtual_walls=True)

	# # rotate wrist backward
	# joints[5] -= np.deg2rad(40)
	# fa.goto_joints(joints, ignore_virtual_walls=True)

	# # rotate wrist backward more
	# joints[5] -= np.deg2rad(40)
	# fa.goto_joints(joints, ignore_virtual_walls=True)

	moving_queue.put(False)


if __name__ == "__main__":
	fa = FrankaArm()
	fa.reset_pose()
	fa.reset_joints()
	fa.open_gripper()

	time.sleep(2)

	# go to starting location
	pose = fa.get_pose()
	pose.translation = np.array([0.5, 0.2, 0.35])
	fa.goto_pose(pose)

	# rotate wrist to pick the cup up from the side
	joints = fa.get_joints()
	joints[5] += np.deg2rad(45)
	fa.goto_joints(joints, ignore_virtual_walls=True)

	# close gripper
	fa.goto_gripper(0.03)

	time.sleep(5)

	moving_queue = queue.Queue()

	vision = threading.Thread(target=vision_loop, args=(moving_queue,))
	position_tracking = threading.Thread(target=control_loop, args=(fa,moving_queue,))
	vision.start()
	position_tracking.start()