import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

"""
This function is a hard-coded demo of the throwing task
with a ball to familiarize with the code-based control. 
"""

# NOTE: because of the experimental setup, we primarily want
# to restrict the Arm's movement to the x and z planes

if __name__ == "__main__":
    fa = FrankaArm()
    
    # reset franka to its home joints
    fa.reset_joints()

    # read functions
    T_ee_world = fa.get_pose()
    print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))

    joints = fa.get_joints()
    print('Joints: {}'.format(joints))

    gripper_width = fa.get_gripper_width()
    print('Gripper width: {}'.format(gripper_width))


    # move to where the ball is and pick it up
    T_ee_world = fa.get_pose()
    print('EE World: ', T_ee_world)
    T_ee_world.translation += [0, 0, -0.38]
    fa.goto_pose(T_ee_world)

    # grasp the ball
    print('Close gripper to a specified position')
    fa.goto_gripper(0.03)

    fa.reset_joints()

    # move to where the ball is and pick it up
    T_ee_world = fa.get_pose()
    print('EE World: ', T_ee_world)
    T_ee_world.translation += [0.3, 0, -0.32]
    fa.goto_pose(T_ee_world)

    # open gripper
    fa.open_gripper()

    # reset franka back to home
    fa.reset_joints()

    print('Opening gripper all the way')
    fa.open_gripper()