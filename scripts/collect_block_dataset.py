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
- while loop where each loop through is triggered by pressing a key
- internal while loop for moving the robot camera
    - robot in human control mode to move the ee-camera to different locations
- save the images to a data folder
- have the wood blocks of different colors/shapes
"""

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

while True:
    input("\nFixed Cameras Capture: Press Enter to continue...")

    # Camera 2
    frames_2 = pipeline_2.wait_for_frames()
    frames_2 = aligned_stream_2.process(frames_2)
    color_frame_2 = frames_2.get_color_frame()
    depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
    color_image_2 = np.asanyarray(color_frame_2.get_data())
    cv2.imwrite('Block_Data' + '/img' + str(time.time()) + '.png', color_image_2)

    # Camera 3
    frames_3 = pipeline_3.wait_for_frames()
    frames_3 = aligned_stream_3.process(frames_3)
    color_frame_3 = frames_3.get_color_frame()
    depth_frame_3 = frames_3.get_depth_frame().as_depth_frame()
    color_image_3 = np.asanyarray(color_frame_3.get_data())
    cv2.imwrite('Block_Data' + '/img' + str(time.time()) + '.png', color_image_3)

    # Camera 4
    frames_4 = pipeline_4.wait_for_frames()
    frames_4 = aligned_stream_4.process(frames_4)
    color_frame_4 = frames_4.get_color_frame()
    depth_frame_4 = frames_4.get_depth_frame().as_depth_frame()
    color_image_4 = np.asanyarray(color_frame_4.get_data())
    cv2.imwrite('Block_Data' + '/img' + str(time.time()) + '.png', color_image_4)

    # Camera 5
    frames_5 = pipeline_5.wait_for_frames()
    frames_5 = aligned_stream_5.process(frames_5)
    color_frame_5 = frames_5.get_color_frame()
    depth_frame_5 = frames_5.get_depth_frame().as_depth_frame()
    color_image_5 = np.asanyarray(color_frame_5.get_data())
    cv2.imwrite('Block_Data' + '/img' + str(time.time()) + '.png', color_image_5)

    for i in range(8):
        input("EE Camera Img " + str(i) + " Press enter to continue...")

        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = aligned_stream_1.process(frames_1)
        color_frame_1 = frames_1.get_color_frame()
        depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        cv2.imwrite('Block_Data' + '/img' + str(time.time()) + '.png', color_image_1)




# while collecting images:
    # hit enter to continue
    # capture images from cameras 2-5
    # for i in range(8):
        # hit s to continue
        # capture image from camera 1