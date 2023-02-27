import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

import pyrealsense2 as rs
import cv2

"""
This is a demo that uses the Intel RealSense D415 camera
to estimate the position of a cup and call the robot to 
pick it up.
"""

if __name__ == "__main__":
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

    # get the cup position using realsense camera
    cup_pos = np.zeros(3)

    W = 848
    H = 480

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    print("[INFO] start streaming...")
    pipeline.start(config)

    aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
    point_cloud = rs.pointcloud()

    print("[INFO] loading model...")
    # download model from: https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API#run-network-in-opencv
    net = cv2.dnn.readNetFromTensorflow("faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb", "faster_rcnn_inception_v2_coco_2018_01_28.pbtxt")

    # only detect in the first frame (don't continue to update)
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame().as_depth_frame()

    points = point_cloud.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)  # xyz

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())

    # skip empty frames
    if not np.any(depth_image):
    	print("no depth")
    	# continue

    print("[INFO] found a valid depth frame")
    color_image = np.asanyarray(color_frame.get_data())

    scaled_size = (int(W), int(H))
    net.setInput(cv2.dnn.blobFromImage(color_image, size=scaled_size, swapRB=True, crop=False))
    detections = net.forward()

    # isolate individual elements detected to calculate position
    for detection in detections[0,0]:
        score = float(detection[2])
        idx = int(detection[1])

        # NOTE: this only works when a single cup is in frame!!!
        if score > 0.8 and idx == 46:
        	left = detection[3] * W
        	top = detection[4] * H
        	right = detection[5] * W
        	bottom = detection[6] * H
        	width = right - left
        	height = bottom - top

        	bbox = (int(left), int(top), int(width), int(height))

        	p1 = (int(bbox[0]), int(bbox[1]))
        	p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        	cv2.rectangle(color_image, p1, p2, (255, 0, 0), 2, 1)

        	# x,y,z of bounding box
        	obj_points = verts[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])].reshape(-1, 3)

        	zs = obj_points[:,2]
        	z = np.median(zs)
        	xs = obj_points[:,0]
        	ys = obj_points[:,1]
        	ys = np.delete(ys, np.where((zs < z - 1) | (zs > z + 1))) # take only y for close z to prevent including background

        	x_pos = np.median(xs)
        	y_pos = np.median(ys)
        	z_pos = z

        	cup_pos = np.array([x_pos, y_pos, z_pos])

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(10)
    # pipeline.stop()

    print("\nCup Position: ", cup_pos)


    # translate camera position to robot position
    # Camera z -> robot x (only +ive)
    # Camera y -> robot z (inverse of each other I think)
    # Camera x -> robot y (inverse of each other I think)

    cup_pos = np.array([cup_pos[2], -cup_pos[0], -cup_pos[1]])
    print("Cup Position Altered X,Y,Z to robot frame: ", cup_pos)
    modifier = np.array([0.07, -0.15, 0.16])
    cup_pos = cup_pos + modifier
    print("Modified Cup Position with Y translation: ", cup_pos)

    # assert False




    # move to directly above where cup is
    T_ee_world = fa.get_pose()
    print('EE World: ', T_ee_world)
    above_cup_pos = cup_pos + np.array([0, 0, 0.15])
    T_ee_world.translation = above_cup_pos
    fa.goto_pose(T_ee_world)

    # move down to cup location
    T_ee_world = fa.get_pose()
    T_ee_world.translation = cup_pos
    fa.goto_pose(T_ee_world)

    # pick up cup
    # print('Close gripper to a specified position')
    fa.close_gripper()

    # move to goal location (hardcoded location)
    T_ee_world = fa.get_pose()
    goal_pos = np.array([0.4, -0.1, 0.1])
    T_ee_world.translation = goal_pos 
    fa.goto_pose(T_ee_world)

    # place cup
    fa.open_gripper()

    # reset franka back to home
    fa.reset_joints()
