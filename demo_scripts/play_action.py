import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

"""
Given a 4DOF action from panda-gym environment, execute
on real Franka arm.
"""

if __name__ == "__main__":
    fa = FrankaArm()
    
    # reset franka to its home joints
    fa.reset_joints()

    # read functions
    T_ee_world = fa.get_pose()

	sim_action = np.array([0, 0, 0, 0]) # Sim action is a change in ee position + change in gripper width
	sim_action = sim_action + np.array([0.6, 0, 0, 0]) # change x offset from simulation frame to real-world frame
	delta_width = sim_action[-1]
	sim_action = sim_action[0:3]

	T_ee_world.translation += sim_action * 0.05
    fa.goto_pose(T_ee_world)
    # NOTE: experiment with using goto_pose_delta() instead of goto_pose()
    current_width = fa.get_gripper_width()
    ga.goto_gripper(current_width + delta_width * 0.2)
