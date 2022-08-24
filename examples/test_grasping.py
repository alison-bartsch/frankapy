from frankapy import FrankaArm
import numpy as np
import argparse
import time
from autolab_core import RigidTransform, Point


if __name__ == "__main__":
	"""
	This is a very simple file that tests various grasping (grasp = True or False) to
	see what objects trigger failures for the grasp = False.
	"""
	print("Starting robot")
	fa = FrankaArm()

	print("Opening Grippers")
	# Open Gripper
	fa.close_gripper()
	fa.open_gripper()
	# Reset Pose
	fa.reset_pose()
	# Reset Joints
	fa.reset_joints()

	print("Testing Grasps!")
	time.sleep(4)

	# testing grasps
	# print("Grasp = True")
	# fa.goto_gripper(0.02, grasp=True, force=10.0)
	# fa.open_gripper()

	print("grasp = False")
	fa.goto_gripper(0.0, grasp=False)
	fa.open_gripper()