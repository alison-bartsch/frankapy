import numpy as np
import json
import numpy as np
from frankapy.vision_utils import *
from autolab_core import RigidTransform

from frankapy import FrankaArm

import pyrealsense2 as rs


W = 848
H = 480

# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee_orginal.tf"

def get_extrinsic(file_path):
    R = []
    
    with open(file_path, 'r') as file:
        T = []
        count  = 0
        for line in file:
            # print(line)
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
    print(T, R)
    T = np.array(T)
    R = np.array(R)
    print(R)
    print(T)
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
    



if __name__ == "__main__":
    
    P = get_extrinsic(REALSENSE_TF_CAM_1)
    # print(P)
    
    K = get_intrinsics(REALSENSE_INTRINSICS_CAM_1)
    # print(K.shape)
    
    # wc = convert_image_to_world(np.array([[0,0]]), P, K)
    # print(wc)
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('220222066259')
    config_1.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    colorizer = rs.colorizer()
    p1 = pipeline_1.start(config_1)
    ds1 = p1.get_device().first_depth_sensor().get_depth_scale()
    aligned_stream_1 = rs.align(rs.stream.color)
    
    
    fa = FrankaArm()
    
    # # reset franka to its home joints
    fa.close_gripper()
    fa.reset_joints()
    pose = fa.get_pose()
    # print("\nRobot Pose: ", pose)
    
    
    while(True):
        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = aligned_stream_1.process(frames_1)
        color_frame_1 = frames_1.get_color_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        
        depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame_1).get_data())
        depths = np.clip(depth_image_1, 0, 1500)
        # depth_image_1 = np.uint16(depths)*255.0/1500.0
        depth_image_1 = (depths * 255.0 / 1500.0).astype(np.uint8)
        depth_image_1 = cv2.cvtColor(depth_image_1, cv2.COLOR_GRAY2BGR)
        d_c = depth_image_1[int(W/2),int(H/2)]*1500/255.*ds1
        centroid = np.array([W/2,H/2, d_c])
        xyz = get_object_center_point_in_world_realsense_3D_camera_point(centroid, K, P, pose)
        xyz = np.array([xyz[0], xyz[1] + 0.065, xyz[2] + 0.02])
        print(xyz)
        # Show the images
        cv2.imshow("Camera 1", color_image_1)
        cv2.imshow("Camera 1 Depth", depth_image_1)
        cv2.imshow("Camera 1 depth colormap", depth_colormap)
        
        k = cv2.waitKey(1)
        if k==27:
            break
        # elif k== ord('s'):
        #     print("Saving....")

        #     # Save the image
        #     cam1_filename_c = str(i) + "_cam1.jpg"
        #     cam1_filename_d = str(i) + "_cam1.png"
            
        #     print(np.unique((depth_image_1/255.0)*1500*ds1))
        #     # print(np.unique(depth_colormap))
        #     print(np.unique(depth_image_1))
        #     cv2.imwrite(cam1_filename_c, color_image_1)
        #     cv2.imwrite(cam1_filename_d, depth_image_1)
        #     i += 1
            
            
    cv2.destroyAllWindows()
    
    
    # pose.translation = np.array([3.07052791e-01, -5.60250537e-06, 0.4])
    # # # # pose.rotation = 
    # fa.goto_pose(pose)
    