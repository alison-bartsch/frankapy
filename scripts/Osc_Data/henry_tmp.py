import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Define params
width, height = 848, 480
fps = 30
rec_time = 5. # sec
save_path = './videodata'

serial_1 = '220222066259'
serial_2 = '151322066099'
serial_3 = '151322069488'
serial_4 = '151322061880'
serial_5 = '151322066932'


# RealSense camera 3 / Check with other data collection codes
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(serial_4) # replace
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

# Start the camera stream
pipeline.start(config)

# Capture videos
cap = cv2.VideoCapture(pipeline)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('videoprac.mp4', fourcc, fps, (width,height))

while True:
    ret, frame = cap.read ()

    timestamp = time.time()
    filename = f"{save_path}/video_{timestamp}.mp4"
    
    cv2.imshow ('window', frame)

    #out.write (frame)

    # Record the video
    #start_time = time.time()
    #while (time.time() - start_time) < rec_time:
        
        # Wait for frames
     #   frames = pipeline.wait_for_frames()
     #   color_frame = frames.get_color_frame()

        # color frame to numpy array
     #   color_image = np.asanyarray(color_frame.get_data())

        # color frame to video file (out)
    #    out.write(color_image)

        # Display if needed
        #cv2.imshow('Window', color_image)
        #cv2.waitKey(0)

    if cv2.waitKey(0) == ord('q'):
        break

out.release()

    # Wait for the robot moving?
    #time.sleep(5)

pipeline.stop()

# need a line to break out of loop
# rs.stream.depth