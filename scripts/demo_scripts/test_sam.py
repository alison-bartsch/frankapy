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

W = 848
H = 480

# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee.tf"

def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)

def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))

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

    if os.path.isfile(model_path_SAM):
        print("SAM model file exists!")
        model_type = "default"

        sam = sam_model_registry[model_type](checkpoint=model_path_SAM)
        sam.to(device=device)


        #TODO: Change SAM parameters
        SAM = SamAutomaticMaskGenerator(sam)
        # SAM = SamAutomaticMaskGenerator(model=sam,
        #                                 # points_per_side=32,
        #                                 # pred_iou_thresh=0.86,
        #                                 # stability_score_thresh=0.92,
        #                                 # crop_n_layers=1,
        #                                 crop_n_points_downscale_factor=2,
        #                                 min_mask_region_area=100,  # Requires open-cv to run post-processing
        #                                 ) 
                            
    else:
        warnings.warn("The file does not exits.")
        
    #    if os.path.isfile("sam_vit_h_4b8939.pth"):
    #        print("File already exists!")
    #
    #    else:
    #        urllib.request.urlretrieve(url, filename, reporthook=show_progress)
    #        print("\nDownload complete!")
        
    #    sam_checkpoint = "sam_vit_h_4b8939.pth"

    # Load the video
    video_name = 'videos/data_color.avi'
    video_path = os.path.join('', video_name) # Load the appropriate video path 
    if os.path.isfile(video_path):
        print("Video file exists!")
        video = cv2.VideoCapture(video_path)
    else:
        warnings.warn("The file does not exits.")


    # Specifying output video file:
    # output_path = os.path.join('Videos/Test Videos', 'sam_mask_live_centroid_video.mp4')  # Load the appropriate video path
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # output_video = cv2.VideoWriter(output_path, fourcc, 30.0, (int(video.get(3)), int(video.get(4))))

    # Get total number of frames
    num_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    frame_counter = 0
    while video.isOpened():
        ret, frame = video.read()
        
        # Break the loop if the video ends
        if not ret:
            break
        cropped_mask = SAM.generate(frame)
        for i, (mask) in enumerate(zip(mask)):
            plt.figure(figsize=(10,10))
            plt.imshow(frame)
            show_mask(mask, plt.gca())
            # show_points(input_point, input_label, plt.gca())
            plt.axis('off')
            plt.show()
        #frame_counter += 1

    print("Process Finished!!!")
    # print(f"Output video saved at: {output_path}")
    video.release()
    # output_video.release()
    cv2.destroyAllWindows()



