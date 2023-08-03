import torch
import warnings
import sys
import torchvision
import os
# import pyrealsense2 as rs
import cv2
import numpy as np
import matplotlib.pyplot as plt
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import time
W = 848
H = 480

# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee.tf"

if __name__ == '__main__':
    
    # pipeline_1 = rs.pipeline()
    # config_1 = rs.config()
    # config_1.enable_device('220222066259')
    # config_1.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    # config_1.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    # colorizer = rs.colorizer()
    # p1 = pipeline_1.start(config_1)
    # ds1 = p1.get_device().first_depth_sensor().get_depth_scale()
    # aligned_stream_1 = rs.align(rs.stream.color)
    
    #============= Checking for cuda =======================
    print("Checking for cuda...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    if device.type == 'cuda':
        print('CUDA is found! Executing on %s.......'%torch.cuda.get_device_name(0))
    else:
        warnings.warn('CUDA not found! Execution may be slow......')


	#============= Loading the SAM Model =======================
#    url = 'https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth'
#    filename = 'sam_vit_h_4b8939.pth'

	# Make sure the file is present in the 'Models/' folder
    model_path_SAM = os.path.join('Models', 'sam_vit_h_4b8939.pth')
    print(model_path_SAM)


    sam_checkpoint = "sam_vit_h_4b8939.pth"
    model_type = "vit_h"

    # device = "cuda"

    sam = sam_model_registry[model_type](checkpoint=model_path_SAM)
    sam.to(device=device)

    predictor = SamPredictor(sam)


    video_path = 'data_color.avi'
    cap = cv2.VideoCapture(video_path)
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    output_path = 'output_video.avi'
    output_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'XVID'), fps, output_size)

    # Process each frame of the video
    while cap.isOpened():
        start = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame from BGR to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        input_point = np.array([[480,240]])
        input_label = np.array([1])
        predictor.set_image(frame)
        masks, scores, logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
        multimask_output=False,
          )
        h, w = masks.shape[-2:]
        mask_image = masks.reshape(h, w, 1)
        final_sam_mask = np.concatenate([mask_image] * 3, axis=2)
        print(np.unique(mask_image))
        end = time.time()
        print(end - start)
        cv2.imshow('Frame', mask_image)
        cv2.imshow('Mask', final_sam_mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video capture and writer objects
    cap.release()
    out.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()
