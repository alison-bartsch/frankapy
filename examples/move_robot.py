import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import time


if __name__ == "__main__":
    """
    This script demonstrates how to move the robot using the various movement commands in franka_arm.py
    """
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


    # move forward
    T_ee_world = fa.get_pose()
    T_ee_world.translation = np.array([0.55, 0, 0.4])
    fa.goto_pose(T_ee_world)

    fa.goto_gripper(0.038)

    T_ee_world.translation = np.array([0.55, -0.1, 0.4])
    fa.goto_pose(T_ee_world)

    fa.open_gripper()

    time.sleep(2)
    fa.reset_joints()

    # # gripper controls
    # print('Closing gripper')
    # time.sleep(2)
    # fa.close_gripper()
    # gripper_width = fa.get_gripper_width()
    # print('Gripper width: {}'.format(gripper_width))

    # # print('Opening gripper to a specified position')
    # # fa.goto_gripper(0.04)
    # # gripper_width = fa.get_gripper_width()
    # # print('Gripper width: {}'.format(gripper_width))

    # # move forward
    # T_ee_world = fa.get_pose()
    # T_ee_world.translation = np.array([0.4, 0.1, 0.35])
    # fa.goto_pose(T_ee_world)

    # print('Opening gripper all the way')
    # fa.open_gripper()
    # gripper_width = fa.get_gripper_width()
    # print('Gripper width: {}'.format(gripper_width))


    # # reset robot
    # fa.reset_pose()
    # fa.reset_joints()






    # assert False

    # # joint controls
    # print('\nRotating last joint')
    # joints = fa.get_joints()
    # joints[6] += np.deg2rad(20)
    # fa.goto_joints(joints)
    # joints[6] -= np.deg2rad(20)
    # fa.goto_joints(joints)

    # print('\nRotating wrist joint')
    # joints = fa.get_joints()
    # joints[5] += np.deg2rad(10)
    # fa.goto_joints(joints)
    # joints[5] -= np.deg2rad(10)
    # fa.goto_joints(joints)

    # # end-effector pose control
    # print('\nTranslation')
    # T_ee_world = fa.get_pose()
    # print('EE World: ', T_ee_world)
    # T_ee_world.translation += [0.1, 0, 0.1]
    # fa.goto_pose(T_ee_world)
    # print('EE World: ', fa.get_pose())
    # T_ee_world.translation -= [0.1, 0, 0.1]
    # fa.goto_pose(T_ee_world)

    # print('\nRotation in end-effector frame')
    # T_ee_rot = RigidTransform(
    #     rotation=RigidTransform.x_axis_rotation(np.deg2rad(45)),
    #     from_frame='franka_tool', to_frame='franka_tool'
    # )
    # T_ee_world_target = T_ee_world * T_ee_rot
    # fa.goto_pose(T_ee_world_target)
    # fa.goto_pose(T_ee_world)

    # reset franka back to home
    fa.reset_joints()