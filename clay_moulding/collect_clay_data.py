import os
import cv2
import time
import queue
import random
import threading
import numpy as np
import pickle as pkl
import pyrealsense2 as rs
from frankapy import FrankaArm
from scipy.spatial.transform import Rotation

"""
Steps:
    1) Outside loop that gets triggered by hitting enter (for us to reset the clay to more uniform shape)
    2) Inside for loop (iterate over specific number of grasps - number TBD)
    3) Record point cloud from each camera
    4) Randomly generate grasp parameters and save
    5) Execute grasp
    6) Record point cloiud from each camera

Dataset Structure:
    Dataset/
        Trajectory1/
            State1/
                PointCloud1
                PointCloud2
                PointCloud3
                PointCloud4
                PointCloud5
            Action1/
                GraspParams
            State2/
                ...
            Action2/
                ...
            ...
        Trajectory2/
            ...
        ...
"""

def goto_grasp(fa, x, y, z, rx, ry, rz, d):
	"""
	Parameterize a grasp action by the position [x,y,z] Euler angle rotation [rx,ry,rz], and width [d] of the gripper.
	This function was designed to be used for clay moulding, but in practice can be applied to any task.

	:param fa:  franka robot class instantiation
	"""
	pose = fa.get_pose()
	starting_rot = pose.rotation
	orig = Rotation.from_matrix(starting_rot)
	orig_euler = orig.as_euler('xyz', degrees=True)
	rot_vec = np.array([rx, ry, rz])
	new_euler = orig_euler + rot_vec
	r = Rotation.from_euler('xyz', new_euler, degrees=True)
	pose.rotation = r.as_matrix()
	pose.translation = np.array([x, y, z])

	fa.goto_pose(pose)
	fa.goto_gripper(d)
	time.sleep(3)

# home joints
fa = FrankaArm()

# reset franka to its home joints
fa.open_gripper()
fa.reset_joints()
original_pose = fa.get_pose()

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

traj=0
while True:
    input("\nClay is reset: Press Enter to continue...")

    # create the trajectory save directory here!
    save_dir = 'Dataset/Trajectory' + str(traj)
    os.mkdir(save_dir)

    for i in range(5):

        # edge case for first action, need to record point cloud before and after
        if i == 0:
            # save the point clouds to state_i folder
            state_save = save_dir + '/State' + str(i)
            os.mkdir(state_save)

            # Camera 1
            frames_1 = pipeline_1.wait_for_frames()
            frames_1 = aligned_stream_1.process(frames_1)
            color_frame_1 = frames_1.get_color_frame()
            depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
            color_image_1 = np.asanyarray(color_frame_1.get_data())
            points_1 = point_cloud_1.calculate(depth_frame_1)
            cam_1_save = state_save + '/pc1.ply'
            points_1.export_to_ply(cam_1_save, color_frame_1)

            # Camera 2
            frames_2 = pipeline_2.wait_for_frames()
            frames_2 = aligned_stream_2.process(frames_2)
            color_frame_2 = frames_2.get_color_frame()
            depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
            color_image_2 = np.asanyarray(color_frame_2.get_data())
            depth_image_2 = np.asanyarray(depth_frame_2.get_data())
            points_2 = point_cloud_2.calculate(depth_frame_2)
            cam_2_save = state_save + '/pc2.ply'
            points_2.export_to_ply(cam_2_save, color_frame_2)

            # Camera 3
            frames_3 = pipeline_3.wait_for_frames()
            frames_3 = aligned_stream_3.process(frames_3)
            color_frame_3 = frames_3.get_color_frame()
            depth_frame_3 = frames_3.get_depth_frame().as_depth_frame()
            color_image_3 = np.asanyarray(color_frame_3.get_data())
            depth_image_3 = np.asanyarray(depth_frame_3.get_data())
            points_3 = point_cloud_3.calculate(depth_frame_3)
            cam_3_save = state_save + '/pc3.ply'
            points_3.export_to_ply(cam_3_save, color_frame_3)

            # Camera 4
            frames_4 = pipeline_4.wait_for_frames()
            frames_4 = aligned_stream_4.process(frames_4)
            color_frame_4 = frames_4.get_color_frame()
            depth_frame_4 = frames_4.get_depth_frame().as_depth_frame()
            color_image_4 = np.asanyarray(color_frame_4.get_data())
            points_4 = point_cloud_4.calculate(depth_frame_4)
            cam_4_save = state_save + '/pc4.ply'
            points_4.export_to_ply(cam_4_save, color_frame_4)

            # Camera 5
            frames_5 = pipeline_5.wait_for_frames()
            frames_5 = aligned_stream_5.process(frames_5)
            color_frame_5 = frames_5.get_color_frame()
            depth_frame_5 = frames_5.get_depth_frame().as_depth_frame()
            color_image_5 = np.asanyarray(color_frame_5.get_data())
            points_5 = point_cloud_5.calculate(depth_frame_5)
            cam_5_save = state_save + '/pc5.ply'
            points_5.export_to_ply(cam_5_save, color_frame_5)

        # move to hovering pose
        overhead_pose = original_pose
        overhead_pose.translation = np.array([0.66, 0, 0.4])
        fa.goto_pose(overhead_pose)
        pose = fa.get_pose()
        
        # ------------------ randomly generate grasp parameters --------------------
        x = round(random.uniform(0.6, 0.7), 2)
        y = round(random.uniform(-0.03, 0.03), 2)
        z = round(random.uniform(0.22, 0.27), 2)
        rx = random.randint(-50, 50)
        ry = random.randint(-50, 50)
        rz = random.randint(-90, 90)
        d = round(random.uniform(0.005, 0.075), 3)

        # save the grasp action as action_i
        grasp_action = np.array([x, y, z, rx, ry, rz, d])
        np.save(save_dir + '/action' + str(i) + '.npy', grasp_action)

        # execute grasp
        goto_grasp(fa, x, y, z, rx, ry, rz, d)
        fa.open_gripper()

        # move robot back to hovering position
        fa.goto_pose(overhead_pose)

        # save the point clouds to state_(i+1) folder
        state_save = save_dir + '/State' + str(i+1)
        os.mkdir(state_save)

        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = aligned_stream_1.process(frames_1)
        color_frame_1 = frames_1.get_color_frame()
        depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        points_1 = point_cloud_1.calculate(depth_frame_1)
        cam_1_save = state_save + '/pc1.ply'
        points_1.export_to_ply(cam_1_save, color_frame_1)

        # Camera 2
        frames_2 = pipeline_2.wait_for_frames()
        frames_2 = aligned_stream_2.process(frames_2)
        color_frame_2 = frames_2.get_color_frame()
        depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        points_2 = point_cloud_2.calculate(depth_frame_2)
        cam_2_save = state_save + '/pc2.ply'
        points_2.export_to_ply(cam_2_save, color_frame_2)

        # Camera 3
        frames_3 = pipeline_3.wait_for_frames()
        frames_3 = aligned_stream_3.process(frames_3)
        color_frame_3 = frames_3.get_color_frame()
        depth_frame_3 = frames_3.get_depth_frame().as_depth_frame()
        color_image_3 = np.asanyarray(color_frame_3.get_data())
        depth_image_3 = np.asanyarray(depth_frame_3.get_data())
        points_3 = point_cloud_3.calculate(depth_frame_3)
        cam_3_save = state_save + '/pc3.ply'
        points_3.export_to_ply(cam_3_save, color_frame_3)

        # Camera 4
        frames_4 = pipeline_4.wait_for_frames()
        frames_4 = aligned_stream_4.process(frames_4)
        color_frame_4 = frames_4.get_color_frame()
        depth_frame_4 = frames_4.get_depth_frame().as_depth_frame()
        color_image_4 = np.asanyarray(color_frame_4.get_data())
        points_4 = point_cloud_4.calculate(depth_frame_4)
        cam_4_save = state_save + '/pc4.ply'
        points_4.export_to_ply(cam_4_save, color_frame_4)

        # Camera 5
        frames_5 = pipeline_5.wait_for_frames()
        frames_5 = aligned_stream_5.process(frames_5)
        color_frame_5 = frames_5.get_color_frame()
        depth_frame_5 = frames_5.get_depth_frame().as_depth_frame()
        color_image_5 = np.asanyarray(color_frame_5.get_data())
        points_5 = point_cloud_5.calculate(depth_frame_5)
        cam_5_save = state_save + '/pc5.ply'
        points_5.export_to_ply(cam_5_save, color_frame_5)

    traj+=1
