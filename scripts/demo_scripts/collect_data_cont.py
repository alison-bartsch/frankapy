import cv2
import numpy as np
import pyrealsense2 as rs
from autolab_core import RigidTransform
from frankapy import FrankaArm

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


if __name__ == "__main__":

    fa = FrankaArm()
    
    # reset franka to its home joints
    fa.close_gripper()
    # fa.reset_pose()
    fa.reset_joints()
    # pose = fa.get_pose()
    # print("\nRobot Pose: ", pose)
    # T_ee_world = fa.get_pose()
    # # print('Current pose: ', T_ee_world)
    # T_ee_world.translation = np.array([0.3, -0.2, 0.3])
    # fa.goto_pose(T_ee_world, block = True)

    # cv2.namedWindow('My Window')
    # image = 255*np.ones((H, W, 3), dtype=np.uint8)
    i = 0
    # while(True):
        # # Camera 1
        # frames_1 = pipeline_1.wait_for_frames()
        # frames_1 = aligned_stream_1.process(frames_1)
        # color_frame_1 = frames_1.get_color_frame()
        # color_image_1 = np.asanyarray(color_frame_1.get_data())
        
        # depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        # depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        # depth_colormap = np.asanyarray(colorizer.colorize(depth_frame_1).get_data())
        # depths = np.clip(depth_image_1, 0, 1500)
        # # depth_image_1 = np.uint16(depths)*255.0/1500.0
        # depth_image_1 = (depths * 255.0 / 1500.0).astype(np.uint8)
        
        
        
        # # Show the images
        # cv2.imshow("Camera",image )
        # cv2.imshow("Camera 1 Depth", depth_image_1)
        # cv2.imshow("Camera 1 depth colormap", depth_colormap)
        # color_video.write(color_image_1)
        # depth_video.write(depth_image_1)
        
        # k = cv2.waitKey(1)
        # if k==27:
            # break
        # elif k== ord('s'):
        #     print("Saving....")
        #     # Save the image
        #     cam1_filename_c = "/socket_Imgs/color/" + str(i) + "_cam1.jpg"
        #     cam1_filename_d = "/socket_Imgs/depth/" + str(i) + "_cam1.png"
            
        #     print(np.unique((depth_image_1/255.0)*1500*ds1))
        #     # print(np.unique(depth_colormap))
            
        #     cv2.imwrite(cam1_filename_c, color_image_1)
        #     cv2.imwrite(cam1_filename_d, depth_image_1)
        #     i += 1
            
    z_positions = [0.1, 0.2, 0.3, 0.4, 0.4, 0.5]
    x_positions = [0.4, 0.5, 0.6, 0.7]
    y_positions = [-0.15, -0.1, 0, 0.1, 0.15]
    T_ee_world = fa.get_pose()
    for z in z_positions:
        for x in x_positions:
            for y in y_positions:
                
                # print('Current pose: ', T_ee_world)
                T_ee_world.translation = np.array([x, y, z])
                fa.goto_pose(T_ee_world, block = True)
                print('New pose: ', fa.get_pose())
                fa.wait_for_skill()
    fa.reset_joints()  
        # elif k == ord('y'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation += [0., 0.1, 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('z'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation += [0., 0., 0.1]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
            
        # elif k == ord('c'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0.1, 0., 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('u'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0., 0.1, 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('a'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0., 0., 0.1]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        
            
            
    # cv2.destroyAllWindows()