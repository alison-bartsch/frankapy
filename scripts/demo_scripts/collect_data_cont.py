import cv2
import numpy as np
import pyrealsense2 as rs
from autolab_core import RigidTransform
from frankapy import FrankaArm, FrankaConstants

"""
This file is to use as a reference for how to call each camera by the 
respective serial number as well as which transform file is applicable
for each camera.
"""
W = 848
H = 480

# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee.tf"

def random_rotation_matrix():
    fconst = FrankaConstants()
    theta = np.random.uniform(0, np.pi/4.)  # Rotation around z-axis
    phi = np.random.uniform(0, np.pi/4.)    # Rotation around y-axis
    psi = np.random.uniform(0, np.pi/4.)    # Rotation around x-axis
    print(theta*180/np.pi, phi*180/np.pi, psi*180/np.pi)

    # psi = 0.1
    # phi = 0.1
    # theta = 0.1
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    cos_phi, sin_phi = np.cos(phi), np.sin(phi)
    cos_psi, sin_psi = np.cos(psi), np.sin(psi)

    # Z-Y-X Euler angles rotation matrix
    Rz = np.array([[cos_theta, -sin_theta, 0],
                   [sin_theta, cos_theta, 0],
                   [0, 0, 1]])
    Ry = np.array([[cos_phi, 0, sin_phi],
                   [0, 1, 0],
                   [-sin_phi, 0, cos_phi]])
    Rx = np.array([[1, 0, 0],
                   [0, cos_psi, -sin_psi],
                   [0, sin_psi, cos_psi]])

    rotation_matrix = Rz.dot(Ry).dot(Rx).dot(fconst.HOME_POSE.rotation)

    return rotation_matrix


if __name__ == "__main__":

    
    fa = FrankaArm()
    fconst = FrankaConstants()
    # reset franka to its home joints
    fa.close_gripper()
    fa.reset_joints()

    pose = fa.get_pose()
    pose.translation = np.array([0.4, 0, 0.1])
    fa.goto_pose(pose, block=True)

    i = 0
    
    # z_positions = [0.1, 0.2, 0.3, 0.4, 0.4, 0.5]
    # x_positions = [0.4, 0.5, 0.6, 0.7]
    # y_positions = [-0.15, -0.1, 0, 0.1, 0.15]

    z_positions = [-0.1, 0.1]
    x_positions = [-0.1, 0.1]
    y_positions = [-0.1, 0.1]
    for i in range(10):

        rotation = random_rotation_matrix()
        rotated_translation = (rotation@np.array([[0], [0], [-0.25]])).flatten()
        pose.translation = rotated_translation + np.array([0.4, 0, 0.04])
        pose.rotation = rotation
        # fa.goto_pose(pose)
        for z in z_positions:
            for x in x_positions:
                for y in y_positions:

                    # print('Current pose: ', T_ee_world)
                    pose.translation += np.array([x, y, z])
                    TransMatrix = np.vstack((np.hstack((pose.rotation, pose.translation[:, None])), [0, 0, 0 ,1]))
                    if fa.is_joints_in_collision_with_boxes(): #Function inside Franka Arm
                        continue
                    fa.goto_pose(pose, block = True)
                    print('New pose: ', fa.get_pose())
                    fa.wait_for_skill()
        fa.reset_joints()  
        print("\nRobot Pose: ", pose.rotation)
        print("Done")
    # print("\nRobot Pose: ", pose)
    # T_ee_world = fa.get_pose()
    # # print('Current pose: ', T_ee_world)
    # T_ee_world.translation = np.array([0.3, -0.2, 0.3])
    # fa.goto_pose(T_ee_world, block = True)


    # z_positions = [0.1, 0.2, 0.3, 0.4, 0.4, 0.5]
    # x_positions = [0.4, 0.5, 0.6, 0.7]
    # y_positions = [-0.15, -0.1, 0, 0.1, 0.15]
    # T_ee_world = fa.get_pose()
    # for z in z_positions:
    #     for x in x_positions:
    #         for y in y_positions:
                
    #             # print('Current pose: ', T_ee_world)
    #             T_ee_world.translation = np.array([x, y, z])
    #             fa.goto_pose(T_ee_world, block = True)
    #             print('New pose: ', fa.get_pose())
    #             fa.wait_for_skill()
    # fa.reset_joints()  
