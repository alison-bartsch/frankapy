import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm

"""
This function is a hard-coded demo of the pick and place task
with a block to familiarize with the code-based control. 
"""

if __name__ == "__main__":
    fa = FrankaArm()
    
    # reset franka to its home joints
    # fa.close_gripper()
    # fa.reset_pose()
    # fa.reset_joints()
    pose = fa.get_pose()
    print("\nRobot Pose: ", pose)
    pose.translation = np.array([3.07052791e-01, -5.60250537e-06, 0.4])
    # # pose.rotation = 
    fa.goto_pose(pose)