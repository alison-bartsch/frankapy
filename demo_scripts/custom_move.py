from frankapy import FrankaArm

'''
This function moves the Franka arm to any desired position. 
'''


def move_to_pose(x, y, z):
    T_ee_world = fa.get_pose()
    print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))
    T_ee_world.translation += [x, y, z]
    print("Moving the arm {}, {}, {}".format(x, y, z))
    print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))
    fa.goto_pose(T_ee_world)


if __name__ == "__main__":
    fa = FrankaArm()

    # reset franka to its home joints
    fa.reset_joints()

    # read functions
    T_ee_world = fa.get_pose()
    print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))

    joints = fa.get_joints()
    print('Joints: {}'.format(joints))

    # move the arm in x direction
    move_to_pose(0.3, 0, 0)

    # move the arm in -z direction
    move_to_pose(0, 0, -0.3)

    # move the arm in -x direction
    move_to_pose(-0.3, 0, 0)

    # move the arm in z direction
    move_to_pose(0, 0, 0.3)
