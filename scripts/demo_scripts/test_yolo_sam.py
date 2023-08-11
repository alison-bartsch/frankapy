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
from scipy.ndimage import gaussian_filter
import time 

W = 848
H = 480


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

# if OPEN3D:
#     vis = o3d.visualization.Visualizer()
#     vis.create_window()


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

depth_data =np.load("depth_data.npy")
print(depth_data.shape)

frame_count = 0
while cap.isOpened():
    start = time.time()
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
    
    detections = yolo_boxes.shape[0]
    for i in range(detections):
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
            
        # color = cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)

    if SAM and final_sam_mask is not None:
        # final_sam_mask = np.concatenate([final_sam_mask] * 3, axis=2)
        cv2.imshow("YOLO + SAM mask", final_sam_mask.astype(np.float32))


    if OPEN3D:
        if SAM and final_sam_mask is not None:
            # depths = ((depth*final_sam_mask)*1500/255.0).astype(np.uint8)

            # depths = depth_data[frame_count]*final_sam_mask[:,:,0]
            depths = gaussian_filter(depth_data[frame_count], sigma=1)
            depths = depths*final_sam_mask[:,:,0]
            print(np.unique(depths))
        # else: depths = ((depth)*1500/255.0).astype(np.uint8)
        else: depths = depth_data[frame_count]
        color_image = o3d.geometry.Image(color)
        depth_image = o3d.geometry.Image(depths)
        
        ds1 = 0.001
        cam = o3d.camera.PinholeCameraIntrinsic(W, H, 621.70715, 621.9764, 323.3644, 244.8789)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_image,
                    depth_image,
                    depth_scale=1.0 / ds1,
                    depth_trunc= 1.5,
                    convert_rgb_to_intensity=False)
        temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, cam)
        
        # vis.add_geometry(temp)
        # vis.poll_events()
        # vis.update_renderer()

        # cl, ind = temp.remove_statistical_outlier(nb_neighbors=200,
        #                                                 std_ratio=1.0)
        o3d.visualization.draw_geometries([temp])

        ## Plane fitting through cropped pointcloud
        plane_model, inliers = temp.segment_plane(distance_threshold=0.001,
                                         ransac_n = 3,
                                         num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        
        ## X, Y, Z orientations
        mag = np.linalg.norm(np.array(plane_model))
        rx = np.arccos(a/mag)
        ry = np.arccos(b/mag)
        rz = np.arccos(c/mag)
        print(rx*180./np.pi,ry*180./np.pi,rz*180./np.pi)

        inlier_cloud = temp.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = temp.select_by_index(inliers, invert=True)
        # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        
        ## cv2.imshow("Depths", depths.astype(np.float32))

    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

    # Write the frame to the output video
    out.write(color)

    # Display the frame (optional)
    cv2.imshow('Frame', color)

    if yolo_masks is not None:
        mask = np.sum(yolo_masks.data.cpu().numpy(), axis= 0)
        cv2.imshow('YOLO Segmentation Mask', mask)
    else: continue

    end = time.time()
    print("Time for 1 frame:", end - start)
    k = cv2.waitKey(1)
    frame_count += 1
    if k == 27:
         break 
    if k == ord('o'):
         OPEN3D = True
    
# Release video capture and writer objects
cap.release()
out.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
