import cv2
import torch
from pathlib import Path
from ultralytics import YOLO
import numpy as np
from segment_anything import sam_model_registry, SamPredictor
import time
import os
import open3d as o3d
import warnings 

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

if device.type == 'cuda':
        print('CUDA is found! Executing on %s.......'%torch.cuda.get_device_name(0))
else:
    warnings.warn('CUDA not found! Execution may be slow......')

# yolo_weights = 'segment_best.pt'
model_path_YOLO = os.path.join('Models', 'best.pt')
if os.path.isfile(model_path_YOLO):
        print("YOLO model file exists!")
else: 
    warnings.warn("YOLO model does not exist.")
    quit()

SAM = True
OPEN3D = True

model = YOLO(model_path_YOLO) 
model.to(device)

	#============= Loading the SAM Model =======================
#    url = 'https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth'
#    filename = 'sam_vit_h_4b8939.pth'

model_type = "vit_h"

model_path_SAM = os.path.join('Models', 'sam_vit_h_4b8939.pth')

if os.path.isfile(model_path_SAM):
        print("SAM model file exists!")
else: 
    warnings.warn("SAM model does not exist.")
    quit()

if SAM:
    sam = sam_model_registry[model_type](checkpoint=model_path_SAM)
    sam.to(device=device)
    predictor = SamPredictor(sam)


color_path = 'data_color.avi'
depth_path = 'data_depth.avi'
cap = cv2.VideoCapture(color_path)
dep = cv2.VideoCapture(depth_path)

fps = int(cap.get(cv2.CAP_PROP_FPS))
fps_depth = int(dep.get(cv2.CAP_PROP_FPS))


output_path = 'output_video.avi'
output_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'XVID'), fps, output_size)


while cap.isOpened():
    ret, color = cap.read()
    ret_d, depth = dep.read()
    
    if (not ret) or (not ret_d) :
        break

    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)


    if SAM:
        predictor.set_image(color)
        final_sam_mask = None
    
    yolo_results = model(color)
    yolo_boxes = yolo_results[0].boxes
    yolo_masks = yolo_results[0].masks
    
    
    for i in range(yolo_boxes.shape[0]):
        box = yolo_boxes[i].xyxy.tolist()[0]
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        
        if SAM:
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2)/2)
            input_point = np.array([[x_c,y_c]])
            input_label = np.array([1])
            
            sam_masks, scores, logits = predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=False,)
            
            h, w = sam_masks.shape[-2:]
            mask_image = sam_masks.reshape(h, w, 1)
            
            if final_sam_mask is None:
                final_sam_mask = mask_image
            else:
                final_sam_mask += mask_image
            
        color = cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if SAM:
        final_sam_mask = np.concatenate([final_sam_mask] * 3, axis=2)

        

        
        cv2.imshow("YOLO + SAM mask", final_sam_mask.astype(np.float32))
    if OPEN3D:
        depths = ((depth*final_sam_mask)*1500/255.0).astype(np.uint8)
        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(depths)
        
        ds1 = 0.001
        cam = o3d.camera.PinholeCameraIntrinsic(w, h, 621.70715, 621.9764, 323.3644, 244.8789)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image,
                    depth_image,
                    depth_scale=1.0 / ds1,
                    depth_trunc= 1,
                    convert_rgb_to_intensity=False)
        temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, cam)
        cl, ind = temp.remove_statistical_outlier(nb_neighbors=200,
                                                        std_ratio=1.0)
        o3d.visualization.draw_geometries([temp.select_by_index(ind)])
        cv2.imshow("Depths", depths.astype(np.float32))

    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

    # Write the frame to the output video
    out.write(color)

    # Display the frame (optional)
    cv2.imshow('Frame', color)

    if yolo_masks is not None:
        mask = np.sum(yolo_masks.data.cpu().numpy(), axis= 0)
        cv2.imshow('YOLO Segmentation Mask', mask)
    else: continue
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and writer objects
cap.release()
out.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
