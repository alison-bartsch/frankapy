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

    print('Close gripper to a specified position')
    fa.goto_gripper(0.03)

    print('\nTossing joint changes')
    joints = fa.get_joints()
    print('Joints: ', joints)
    joints[5] -= np.deg2rad(10)
    joints[3] -= np.deg2rad(10)
    fa.goto_joints(joints)
    print('Joints: ', joints)
    joints[5] += np.deg2rad(30)
    joints[3] += np.deg2rad(15)
    fa.goto_joints(joints)
    print('Joints: ', joints)

    # print('\nRotating wrist joint')
    # joints = fa.get_joints()
    # joints[5] -= np.deg2rad(10)
    # fa.goto_joints(joints)
    # joints[5] += np.deg2rad(30)
    # fa.goto_joints(joints)

    # print('\nRotating joint 3')
    # joints = fa.get_joints()
    # print('Joints: ', joints)
    # joints[3] += np.deg2rad(10)
    # fa.goto_joints(joints)
    # joints[3] -= np.deg2rad(10)
    # fa.goto_joints(joints)

    # reset franka back to home
    fa.reset_joints()

    print('Opening gripper all the way')
    fa.open_gripper()