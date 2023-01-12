import os
import cv2
import time
import queue
import threading
import numpy as np
import pickle as pkl
import pyrealsense2 as rs
from frankapy import FrankaArm

"""
Goal: record the robot's pose and all 5 camera's RGBD info at each timestep and save.
    - run on a separate thread to work in conjunction with VR teleoperation
"""

# THREAD 1: have the robot execute some arbitrary trajectory
# THREAD 2: have the robot save the pose and RGBD information (perhaps point clouds too???? maybe too data hungy) every N seconds

def vision_loop(fa, moving, filename, folder_name):
    # ----- Create Save Folders ------
    save_path = 'Dataset/' + folder_name
    if not os.path.exists(save_path):
        os.mkdir(save_path)

    image_save_path = save_path + '/color_images'
    os.mkdir(image_save_path)
    # depth_save_path = save_path + '/depth_images'
    # os.mkdir(depth_save_path)
    cam1_image_save_path = image_save_path + '/camera1'
    os.mkdir(cam1_image_save_path)
    cam2_image_save_path = image_save_path + '/camera2'
    os.mkdir(cam2_image_save_path)
    cam3_image_save_path = image_save_path + '/camera3'
    os.mkdir(cam3_image_save_path)
    cam4_image_save_path = image_save_path + '/camera4'
    os.mkdir(cam4_image_save_path)
    cam5_image_save_path = image_save_path + '/camera5'
    os.mkdir(cam5_image_save_path)
    # cam1_depth_save_path = depth_save_path + '/camera1'
    # os.mkdir(cam1_depth_save_path)
    # cam2_depth_save_path = depth_save_path + '/camera2'
    # os.mkdir(cam2_depth_save_path)
    # cam3_depth_save_path = depth_save_path + '/camera3'
    # os.mkdir(cam3_depth_save_path)
    # cam4_depth_save_path = depth_save_path + '/camera4'
    # os.mkdir(cam4_depth_save_path)
    # cam5_depth_save_path = depth_save_path + '/camera5'
    # os.mkdir(cam5_depth_save_path)


    W = 848
    H = 480

    # ----- Camera 1 (end-effector) -----
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('220222066259')
    config_1.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # ----- Camera 2 (static) -----
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device('151322066099')
    config_2.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # ----- Camera 3 (static) -----
    pipeline_3 = rs.pipeline()
    config_3 = rs.config()
    config_3.enable_device('151322069488')
    config_3.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_3.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # ----- Camera 4 (static) -----
    pipeline_4 = rs.pipeline()
    config_4 = rs.config()
    config_4.enable_device('151322061880')
    config_4.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_4.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # ----- Camera 5 (static) -----
    pipeline_5 = rs.pipeline()
    config_5 = rs.config()
    config_5.enable_device('151322066932')
    config_5.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_5.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # start streaming
    pipeline_1.start(config_1)
    pipeline_2.start(config_2)
    pipeline_3.start(config_3)
    pipeline_4.start(config_4)
    pipeline_5.start(config_5)

    # align stream
    aligned_stream_1 = rs.align(rs.stream.color)
    aligned_stream_2 = rs.align(rs.stream.color)
    aligned_stream_3 = rs.align(rs.stream.color)
    aligned_stream_4 = rs.align(rs.stream.color)
    aligned_stream_5 = rs.align(rs.stream.color)

    # point clouds
    point_cloud_1 = rs.pointcloud()
    point_cloud_2 = rs.pointcloud()
    point_cloud_3 = rs.pointcloud()
    point_cloud_4 = rs.pointcloud()
    point_cloud_5 = rs.pointcloud()

    cam1_list = []
    cam2_list = []
    cam3_list = []
    cam4_list = []
    cam5_list = []
    depth1_list = []
    depth2_list = []
    depth3_list = []
    depth4_list = []
    depth5_list = []
    timesteps = []
    ee_position = []
    ee_widths = []
    joint_angles = []
    rotation_matrices = []
    quaternions = []

    previous_time = time.time()
    i = 0

    while moving.empty():
        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = aligned_stream_1.process(frames_1)
        color_frame_1 = frames_1.get_color_frame()
        depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        points_1 = point_cloud_1.calculate(depth_frame_1)
        verts_1 = np.asanyarray(points_1.get_vertices()).view(np.float32).reshape(-1, W, 3)

        # Camera 2
        frames_2 = pipeline_2.wait_for_frames()
        frames_2 = aligned_stream_2.process(frames_2)
        color_frame_2 = frames_2.get_color_frame()
        depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        points_2 = point_cloud_2.calculate(depth_frame_2)
        verts_2 = np.asanyarray(points_2.get_vertices()).view(np.float32).reshape(-1, W, 3)

        # Camera 3
        frames_3 = pipeline_3.wait_for_frames()
        frames_3 = aligned_stream_3.process(frames_3)
        color_frame_3 = frames_3.get_color_frame()
        depth_frame_3 = frames_3.get_depth_frame().as_depth_frame()
        color_image_3 = np.asanyarray(color_frame_3.get_data())
        depth_image_3 = np.asanyarray(depth_frame_3.get_data())
        points_3 = point_cloud_3.calculate(depth_frame_3)
        verts_3 = np.asanyarray(points_3.get_vertices()).view(np.float32).reshape(-1, W, 3)

        # Camera 4
        frames_4 = pipeline_4.wait_for_frames()
        frames_4 = aligned_stream_4.process(frames_4)
        color_frame_4 = frames_4.get_color_frame()
        depth_frame_4 = frames_4.get_depth_frame().as_depth_frame()
        color_image_4 = np.asanyarray(color_frame_4.get_data())
        depth_image_4 = np.asanyarray(depth_frame_4.get_data())
        points_4 = point_cloud_4.calculate(depth_frame_4)
        verts_4 = np.asanyarray(points_4.get_vertices()).view(np.float32).reshape(-1, W, 3)

        # Camera 5
        frames_5 = pipeline_5.wait_for_frames()
        frames_5 = aligned_stream_5.process(frames_5)
        color_frame_5 = frames_5.get_color_frame()
        depth_frame_5 = frames_5.get_depth_frame().as_depth_frame()
        color_image_5 = np.asanyarray(color_frame_5.get_data())
        depth_image_5 = np.asanyarray(depth_frame_5.get_data())
        points_5 = point_cloud_5.calculate(depth_frame_5)
        verts_5 = np.asanyarray(points_5.get_vertices()).view(np.float32).reshape(-1, W, 3)

        # Robot Pose
        current_pose = fa.get_pose()
        current_joints = fa.get_joints()
        gripper_width = fa.get_gripper_width()

        current_time = time.time()
        timestep = previous_time - current_time

        # Save the Images
        cv2.imwrite(cam1_image_save_path + '/img' + str(i) + '.png', color_image_1)
        cv2.imwrite(cam2_image_save_path + '/img' + str(i) + '.png', color_image_2)
        cv2.imwrite(cam3_image_save_path + '/img' + str(i) + '.png', color_image_3)
        cv2.imwrite(cam4_image_save_path + '/img' + str(i) + '.png', color_image_4)
        cv2.imwrite(cam5_image_save_path + '/img' + str(i) + '.png', color_image_5)
        # cv2.imwrite(cam1_depth_save_path + '/img' + str(i) + '.png', depth_image_1)
        # cv2.imwrite(cam2_depth_save_path + '/img' + str(i) + '.png', depth_image_2)
        # cv2.imwrite(cam3_depth_save_path + '/img' + str(i) + '.png', depth_image_3)
        # cv2.imwrite(cam4_depth_save_path + '/img' + str(i) + '.png', depth_image_4)
        # cv2.imwrite(cam5_depth_save_path + '/img' + str(i) + '.png', depth_image_5)

        # Append the data
        depth1_list.append(depth_image_1)
        depth2_list.append(depth_image_2)
        depth3_list.append(depth_image_3)
        depth4_list.append(depth_image_4)
        depth5_list.append(depth_image_5)
        timesteps.append(timestep)
        ee_position.append(current_pose.translation)
        ee_widths.append(gripper_width)
        joint_angles.append(current_joints)
        rotation_matrices.append(current_pose.rotation)
        quaternions.append(current_pose.quaternion)





        # # Append the data
        # cam1_list.append(color_image_1)
        # cam2_list.append(color_image_2)
        # cam3_list.append(color_image_3)
        # cam4_list.append(color_image_4)
        # cam5_list.append(color_image_5)
        # depth1_list.append(depth_image_1)
        # depth2_list.append(depth_image_2)
        # depth3_list.append(depth_image_3)
        # depth4_list.append(depth_image_4)
        # depth5_list.append(depth_image_5)
        # timesteps.append(timestep)
        # ee_position.append(current_pose.translation)
        # ee_widths.append(gripper_width)
        # joint_angles.append(current_joints)
        # rotation_matrices.append(current_pose.rotation)
        # quaternions.append(current_pose.quaternion)

        # Save images to the folder in the order
        # save the name + str(i)
        
        # print("got the images!")
        time.sleep(0.05)
        print("\nCurrent Pose: ", current_pose)
        previous_time = current_time

        i+=1
    
    # Save the Data
    with open(save_path + '/depth_cam1.pkl', 'wb') as pkl_f:
        pkl.dump(depth1_list, pkl_f)
    
    with open(save_path + '/depth_cam2.pkl', 'wb') as pkl_f:
        pkl.dump(depth2_list, pkl_f)
    
    with open(save_path + '/depth_cam3.pkl', 'wb') as pkl_f:
        pkl.dump(depth3_list, pkl_f)
    
    with open(save_path + '/depth_cam4.pkl', 'wb') as pkl_f:
        pkl.dump(depth4_list, pkl_f)

    with open(save_path + '/depth_cam5.pkl', 'wb') as pkl_f:
        pkl.dump(depth5_list, pkl_f)

    with open(save_path + '/timestep.pkl', 'wb') as pkl_f:
        pkl.dump(timesteps, pkl_f)

    with open(save_path + '/ee_position.pkl', 'wb') as pkl_f:
        pkl.dump(ee_position, pkl_f)
    
    with open(save_path + '/ee_widths.pkl', 'wb') as pkl_f:
        pkl.dump(ee_widths, pkl_f)
    
    with open(save_path + '/joint_angles.pkl', 'wb') as pkl_f:
        pkl.dump(joint_angles, pkl_f)
    
    with open(save_path + '/rotation_matrices.pkl', 'wb') as pkl_f:
        pkl.dump(rotation_matrices, pkl_f)
    
    with open(save_path + '/quaternions.pkl', 'wb') as pkl_f:
        pkl.dump(quaternions, pkl_f)



    # Data Format:
        # /Trajectory [folder]
            # timesteps [array]
            # RGB Images [folder]
                # img 1 [.png]
            # Depth Images [folder]
                # img 1 [depth format]
            # ee positions [array]
            # ee widths [array]
            # joint angles [array]
            # rotation matrices [array]
            # quaternions [array]
    



    # # save to .pkl file
    # traj_dict = {}
    # traj_dict["cam1_list"] = cam1_list
    # traj_dict["cam2_list"] = cam2_list
    # traj_dict["cam3_list"] = cam3_list
    # traj_dict["cam4_list"] = cam4_list
    # traj_dict["cam5_list"] = cam5_list
    # traj_dict["depth1_list"] = depth1_list
    # traj_dict["depth2_list"] = depth2_list
    # traj_dict["depth3_list"] = depth3_list
    # traj_dict["depth4_list"] = depth4_list
    # traj_dict["depth5_list"] = depth5_list
    # traj_dict["timesteps"] = timesteps
    # traj_dict["ee_position"] = ee_position
    # traj_dict["ee_widths"] = ee_widths
    # traj_dict["joint_angles"] = joint_angles
    # traj_dict["rotation_matrices"] = rotation_matrices
    # traj_dict["quaternions"] = quaternions

    # save_path = 'Dataset/' + filename
    # with open(save_path, 'wb') as pkl_f:
    #     pkl.dump(traj_dict, pkl_f)
    #     print("\n\nSaved the data!")





    # # test the quality of the saved data
    # with open('Dataset/data_test.pkl', 'rb') as pkl_r:
    #     loaded_dict = pkl.load(pkl_r)
    
    # test_image = loaded_dict["cam1_list"][1] 
    # print("\nTest Image: ", test_image)
    # cv2.imshow("Visualize Color Image", test_image)
    # cv2.waitKey(1000)


    # Time Since Skill Started

    # Save Images and Pose
        # save as a .pkl as a dictionary???

    # DATA TO STORE:
        # images
        # depth
        # point cloud?
        # timestep
        # end effector position & width
        # joint angles
        # rotation matrix
        # quaternion

    # NOTE: maybe have the QR codes in the corners to calibrate post-data collection???


def action_loop(fa, moving):
    pose = fa.get_pose()
    pose.translation = np.array([0.5, 0, 0.5])
    # franka_queue.put(pose)
    fa.goto_pose(pose)
    print("\nPose: ", pose)
    pose.translation = np.array([0.55, 0, 0.5])
    # franka_queue.put(pose)
    fa.goto_pose(pose)
    print("\nPose: ", pose)
    # pose.translation = np.array([0.6, 0, 0.5])
    # # franka_queue.put(pose)
    # fa.goto_pose(pose)
    # print("\nPose: ", pose)
    # pose.translation = np.array([0.62, 0, 0.5])
    # # franka_queue.put(pose)
    # fa.goto_pose(pose)
    # print("\nPose: ", pose)

    moving.put("action loop stopped")



if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_pose()
    fa.reset_joints()

    moving = queue.Queue()

    filename = 'data_test.pkl'
    folder_name = 'Trajectory1'

    vision = threading.Thread(target=vision_loop, args=(fa, moving, filename, folder_name,))
    action = threading.Thread(target=action_loop, args=(fa, moving,))
    vision.start()
    action.start()

    # pose = fa.get_pose()
    # franka_queue = queue.Queue()
    # franka_queue.put(pose)


    # vision = threading.Thread(target=vision_loop, args=(franka_queue,))
    # action = threading.Thread(target=action_loop, args=(fa, franka_queue,))
    # vision.start()
    # action.start()
    vision.join()
    action.join()