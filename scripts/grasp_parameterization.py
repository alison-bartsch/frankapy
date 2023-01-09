import time
import numpy as np
from frankapy import FrankaArm
from scipy.spatial.transform import Rotation

def goto_grasp(fa, x, y, z, rx, ry, rz, d):
	"""
	Parameterize a grasp action by the position [x,y,z] Euler angle rotation [rx,ry,rz], and width [d] of the gripper.
	This function was designed to be used for clay moulding, but in practice can be applied to any task.

	:param fa:  franka robot class instantiation
	"""
	pose = fa.get_pose()
	starting_rot = pose.rotation
	orig = Rotation.from_matrix(starting_rot)
	orig_euler = orig.as_euler('xyz', degrees=True)
	rot_vec = np.array([rx, ry, rz])
	new_euler = orig_euler + rot_vec
	r = Rotation.from_euler('xyz', new_euler, degrees=True)
	pose.rotation = r.as_matrix()
	pose.translation = np.array([x, y, z])

	fa.goto_pose(pose)
	fa.goto_gripper(d)
	time.sleep(3)

	# TODO: clip the values of each parameter (or return an error) - particularly rx, ry, rz, and d [0, 0.08]
	


if __name__ == '__main__':
	fa = FrankaArm()

	# reset franka to its home joints
	fa.open_gripper()
	fa.reset_joints()

	# move to hovering position
	pose = fa.get_pose()
	overhead_pose = pose
	overhead_pose.translation = np.array([0.66, 0, 0.35])
	pose.translation = np.array([0.66, 0, 0.35])
	fa.goto_pose(pose)
	pose = fa.get_pose()
	
	# goto various arbitraty grasps
	goto_grasp(fa, 0.69, 0, 0.25, 20, 15, 20, 0.01)
	fa.open_gripper()
	fa.goto_pose(overhead_pose)

	goto_grasp(fa, 0.68, 0, 0.25, 0, 0, 0, 0.01)
	fa.open_gripper()
	fa.goto_pose(overhead_pose)

	goto_grasp(fa, 0.65, 0, 0.25, 0, 0, 50, 0.04)
	fa.open_gripper()
	fa.goto_pose(overhead_pose)