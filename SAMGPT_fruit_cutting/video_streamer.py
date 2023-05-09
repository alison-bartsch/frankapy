import os
import cv2
import time
import numpy as np
import pyrealsense2 as rs

# Package versions:
print("Np version: ", np.__version__)
print("CV2 version: ", cv2.__version__)

# Camera 2 (static)
serial_number = '151322066099'  # Replace with the desired camera's serial number
video_duration = 10  # Video duration in seconds

W = 848
H = 480

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(serial_number)
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

# start streaming
pipeline.start(config)

# align stream
aligned_stream = rs.align(rs.stream.color)

# Create a window to display the video
cv2.namedWindow('Video Stream', cv2.WINDOW_AUTOSIZE)

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_path = os.path.join(os.path.expanduser('~'), 'cam_2_video.mp4')
out = cv2.VideoWriter(output_path, fourcc, 30, (W, H))

start_time = time.time()

while time.time() - start_time < video_duration:
    # Get the frames
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    # Write the color frame to the video file
    out.write(color_image)

    # Display the color frame
    cv2.imshow('Color Stream', color_image)
    cv2.waitKey(1)

# Release resources and close the window
out.release()
pipeline.stop()
cv2.destroyAllWindows()
