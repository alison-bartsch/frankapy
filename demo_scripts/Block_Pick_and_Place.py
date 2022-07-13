import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import pickle

"""
This function is a hard-coded demo of the throwing task
with a ball to familiarize with the code-based control. 
"""

# NOTE: because of the experimental setup, we primarily want
# to restrict the Arm's movement to the x and z planes

if __name__ == "__main__":
    
    with open('Pick_and_Place_Motion.p', 'rb') as pickle_file:
        way_points = pickle.load(pickle_file)
    
    way_points = way_points[13:]

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


    # move to each waypoint
    for waypoint in way_points:
        T_ee_world = fa.get_pose()
        print('Way Point:', waypoint)
        T_ee_world.translation = waypoint[0:3] + np.array([0, 0, 0.05])
        fa.goto_pose(T_ee_world, ignore_virtual_walls=True)
        fa.goto_gripper(waypoint[3])    

    # reset franka back to home
    fa.reset_joints()

    print('Opening gripper all the way')
    fa.open_gripper()