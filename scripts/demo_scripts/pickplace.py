import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
from frankapy import FrankaConstants
"""
This function is a hard-coded demo of the pick and place task
with a block to familiarize with the code-based control. 
"""
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
    
    # reset franka to its home joints
    fa.close_gripper()
    # fa.reset_pose()
    fa.reset_joints()
    pose = fa.get_pose()
    pose.translation = np.array([0.4, 0, 0.1])
    fa.goto_pose(pose, block=True)
    print("\nRobot Pose: ", pose.rotation)

    
    rotation = random_rotation_matrix()
    rotated_translation = (rotation@np.array([[0], [0], [-0.25]])).flatten()
    # rotated_translation[0] *= -1
    print(rotated_translation)
    # pose.translation = np.array([3.07052791e-01, -5.60250537e-06, 0.3])
    pose.translation = rotated_translation + np.array([0.4, 0, 0.04])
    
    # pose.rotation = random_rotation_matrix()
    pose.rotation = rotation
    fa.goto_pose(pose)
    print("\nRobot Pose: ", pose.rotation)
    print("Done")