import cv2
import numpy as np
import pyrealsense2 as rs
# from autolab_core import RigidTransform
# from frankapy import FrankaArm
# from frankapy.vision_utils import *
import os
import open3d as o3d
import torch
import warnings
from ultralytics import YOLO
from segment_anything import sam_model_registry, SamPredictor
from scipy.ndimage import gaussian_filter

"""
This file is used to detect the bounding boxes (the usb ports) live while using franka
"""

W = 848
H = 480

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

if device.type == 'cuda':
        print('CUDA is found! Executing on %s.......'%torch.cuda.get_device_name(0))
else:
    warnings.warn('CUDA not found! Execution may be slow......')

# yolo_weights = 'segment_best.pt'
model_path_YOLO = os.path.join('Models', 'best_train9.pt') #best.pt
if os.path.isfile(model_path_YOLO):
        print("YOLO model file exists!")
else: 
    warnings.warn("YOLO model does not exist.")
    quit()

SAM = False
OPEN3D = False

model = YOLO(model_path_YOLO) 
model.to(device)

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


# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee_orginal.tf"

if __name__ == "__main__":
    
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device('220222066259')
    config_1.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    colorizer = rs.colorizer()
    p1 = pipeline_1.start(config_1)
    ds1 = p1.get_device().first_depth_sensor().get_depth_scale()
    aligned_stream_1 = rs.align(rs.stream.color)
    point_cloud = rs.pointcloud()
    # fa = FrankaArm()
    
    # reset franka to its home joints
    # fa.close_gripper()
    # # fa.reset_pose()
    # fa.reset_joints()
    # pose = fa.get_pose()
    # print("\nRobot Pose: ", pose)
    # transform = RigidTransform.load(REALSENSE_TF_CAM_1)

    i = 0
    while(True):
        # Camera 1
        frames_1 = pipeline_1.wait_for_frames()
        frames_1 = aligned_stream_1.process(frames_1)
        if i == 0:
            intrinsics = frames_1.profile.as_video_stream_profile().intrinsics
            out = o3d.camera.PinholeCameraIntrinsic(848, 480, intrinsics.fx,
                                                    intrinsics.fy, intrinsics.ppx,
                                                    intrinsics.ppy)
            i += 1
        color_frame_1 = frames_1.get_color_frame()
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        point_cloud.map_to(color_frame_1)
        color_image_1 = cv2.rectangle(color_image_1, (420,125),(540,180), color=(0,255,255), thickness= 2)
        depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame_1).get_data())
        depths = np.clip(depth_image_1, 0, 1500)
        # depth_image_1 = np.uint16(depths)*255.0/1500.0
        depth_image_1 = (depths * 255.0 / 1500.0).astype(np.uint8)

        ####################  YOLO - SAM intergration with live Camera stream  ###########################################

        if SAM:
            predictor.set_image(color_image_1)
            final_sam_mask = None
    
        yolo_results = model(color_image_1)
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
                
            color = cv2.rectangle(color_image_1, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imshow("YOLO segmentation", color)


        if SAM and final_sam_mask is not None:
            # final_sam_mask = np.concatenate([final_sam_mask] * 3, axis=2)
            cv2.imshow("YOLO + SAM mask", final_sam_mask.astype(np.float32))


        if OPEN3D:
            if SAM:
                # depths = ((depth*final_sam_mask)*1500/255.0).astype(np.uint8)

                # depths = depth_data[frame_count]*final_sam_mask[:,:,0]
                depths = gaussian_filter(depths, sigma=1)
                depths = depths*final_sam_mask[:,:,0]
                print(np.unique(depths))
            # else: depths = ((depth)*1500/255.0).astype(np.uint8)
            else: depths = depths
            color_image = o3d.geometry.Image(color_image_1)
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
            # cl, ind = temp.remove_statistical_outlier(nb_neighbors=200,
            #                                                 std_ratio=1.0)
            # # o3d.visualization.draw_geometries([temp])

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

        color = cv2.cvtColor(color_image_1, cv2.COLOR_RGB2BGR)


        # print(type(depth_frame_1))
        
        # points = point_cloud.calculate(depth_frame_1)
        # tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32)
        # verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)
        # print(type(verts))

        # xyz = get_object_center_point_in_world_realsense_3D_camera_point(verts[0,0], np.eye(3), transform, fa.get_pose())


        # # Show the images
        # cv2.imshow("Camera",color_image_1 )
        # cv2.imshow("Camera 1 Depth", depth_image_1)
        # cv2.imshow("Camera 1 depth colormap", depth_colormap)

        
        # color_video.write(color_image_1)
        # depth_video.write(depth_image_1)
        
        k = cv2.waitKey(1)
        if k==27:
            break
        # elif k == ord('p'):
        #     pose = fa.get_pose()
        #     print(pose.rotation)
        #     print(pose.translation)
        #     t1 = np.eye(4)
        #     t1[:3,:3] = pose.rotation
        #     t1[:3,3] = pose.translation
        #     print(t1)
        #     print(transform)
        #     t2 = np.eye(4)
        #     t2[:3,:3] = transform.rotation
        #     t2[:3,3] = transform.translation
        #     print(t2)

        elif k== ord('s'):
            # print("Saving....")
            # pose = fa.get_pose()
            # t1 = np.eye(4)
            # t1[:3,:3] = pose.rotation
            # t1[:3,3] = pose.translation
            # t2 = np.eye(4)
            # t2[:3,:3] = transform.rotation
            # t2[:3,3] = transform.translation
            cam = o3d.camera.PinholeCameraIntrinsic(W, H, 621.70715, 621.9764, 323.3644, 244.8789)
            # (420,125),(540,180)
            depths[:125,:] = 0
            depths[180:,:] = 0
            depths[:,:420] = 0
            depths[:,540:] = 0
            color_image = o3d.geometry.Image(color_image_1)
            # depths = np.zeros_like(depths)
            depth_image = o3d.geometry.Image(depths)
            # print(depths[420:540, 125:180])
            print(depths.shape)
            
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        color_image,
                        depth_image,
                        depth_scale=1.0 / ds1,
                        depth_trunc= 1,
                        convert_rgb_to_intensity=False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                        rgbd_image, cam)
            # temp = temp.uniform_down_sample(every_k_points=5)
            cl, ind = temp.remove_statistical_outlier(nb_neighbors=200,
                                                            std_ratio=1.0)
            # o3d.visualization.draw_geometries([temp])
            o3d.visualization.draw_geometries([temp.select_by_index(ind)])
            

            # the bounding box coordinates of the ROI
            
        #     # Save the image
        #     cam1_filename_c = "/socket_Imgs/color/" + str(i) + "_cam1.jpg"
        #     cam1_filename_d = "/socket_Imgs/depth/" + str(i) + "_cam1.png"
            
        #     print(np.unique((depth_image_1/255.0)*1500*ds1))
        #     # print(np.unique(depth_colormap))
            
        #     cv2.imwrite(cam1_filename_c, color_image_1)
        #     cv2.imwrite(cam1_filename_d, depth_image_1)
        #     i += 1
            
            
        # elif k == ord('x'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation += [0.1, 0., 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('y'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation += [0., 0.1, 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('z'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation += [0., 0., 0.1]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
            
        # elif k == ord('c'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0.1, 0., 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('u'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0., 0.1, 0.]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        # elif k == ord('a'):
        #     T_ee_world = fa.get_pose()
        #     print('Current pose: ', T_ee_world)
        #     T_ee_world.translation -= [0., 0., 0.1]
        #     fa.goto_pose(T_ee_world)
        #     print('New pose: ', fa.get_pose())
            
        
            
            
    cv2.destroyAllWindows()