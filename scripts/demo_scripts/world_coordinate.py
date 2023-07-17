import numpy as np
import json
import numpy as np

# from autolab_core import RigidTransform

# from frankapy import FrankaArm


def get_extrinsic(file_path):
    R = []
    
    with open(file_path, 'r') as file:
        # T = []
        count  = 0
        for line in file:
            count += 1
            line = line.strip()
            if line:
                if count == 3:
                    values = line.split()
                    T = [float(value) for value in values]
                elif count > 3:    
                    values = line.split()
                    row = [float(value) for value in values]
                    R.append(row)
    T = np.array(T)
    R = np.array(R)
    # print(R)
    RT = np.column_stack((R, T))
    P = np.vstack((RT, [0, 0, 0, 1]))
    return P

def get_intrinsics(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    fx = data["_fx"]
    fy = data["_fy"]
    cx = data["_cx"]
    cy = data["_cy"]

    # Build the intrinsic matrix
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])

    return K

def get_coordinates(image, x, y, P, K):
    pass
    

# import numpy as np

# def convert_image_to_world(img, P, K):
#     K_inv = np.linalg.inv(K)
#     R = P[:3, :3]
#     T = P[:3, 3]
#     # print(T)
#     R_inv = np.linalg.inv(R)
#     camera_center = -np.dot(R_inv, T)

#     img_hom = np.column_stack((img, np.ones(len(img))))
#     intermediate_results = np.dot(K_inv, img_hom.T)
#     wc_hom = np.dot(R_inv, intermediate_results) + camera_center[:, np.newaxis]
#     print(wc_hom)
#     # wc_cart = wc_hom[:3] / wc_hom[3]

#     # return wc_cart.T
#     return 0


if __name__ == "__main__":
    
    file_path = 'calib/realsense_ee_orginal.tf'
    P = get_extrinsic(file_path)
    # print(P)
    
    file_path = 'calib/realsense_intrinsics.intr'
    K = get_intrinsics(file_path)
    # print(K.shape)
    
    # wc = convert_image_to_world(np.array([[0,0]]), P, K)
    # print(wc)
    # fa = FrankaArm()
    
    # # reset franka to its home joints
    # # fa.close_gripper()
    # # fa.reset_pose()
    # # fa.reset_joints()
    # pose = fa.get_pose()
    # print("\nRobot Pose: ", pose)
    # pose.translation = np.array([3.07052791e-01, -5.60250537e-06, 0.4])
    # # # pose.rotation = 
    # fa.goto_pose(pose)