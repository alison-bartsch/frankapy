import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# W = 848
# H = 480

# # ----- Camera 2 (static) -----
# pipeline_2 = rs.pipeline()
# config_2 = rs.config()
# config_2.enable_device('151322066099')
# config_2.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
# config_2.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

# pipeline_2.start(config_2)
# aligned_stream_2 = rs.align(rs.stream.color)
# point_cloud_2 = rs.pointcloud()

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

W = 848
H = 480

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_device('151322066099')
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
align = rs.align(rs.stream.color)

vis = o3d.visualization.Visualizer()
vis.create_window('PCD', width=1280, height=720)
pointcloud = o3d.geometry.PointCloud()
geom_added = False
    
while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)
    profile = frames.get_profile()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    img_depth = o3d.geometry.Image(depth_image) # np.asanyarray(depth_image.get_data())
    img_color = o3d.geometry.Image(color_image) # np.asanyarray(color_image.get_data())
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, convert_rgb_to_intensity=False)
    
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pointcloud.points = pcd.points
    pointcloud.colors = pcd.colors
    
    if geom_added == False:
        vis.add_geometry(pointcloud)
        geom_added = True
    
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    
    cv2.imshow('bgr', color_image)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    

pipeline.stop()
cv2.destroyAllWindows()
vis.destroy_window()






# # Camera 2
# frames_2 = pipeline_2.wait_for_frames()
# frames_2 = aligned_stream_2.process(frames_2)
# color_frame_2 = frames_2.get_color_frame()
# depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
# color_image_2 = np.asanyarray(color_frame_2.get_data())
# depth_image_2 = np.asanyarray(depth_frame_2.get_data())
# point_cloud_2.map_to(color_frame_2)
# points_2 = point_cloud_2.calculate(depth_frame_2)
# cam_2_save = 'pc2.ply'
# points_2.export_to_ply(cam_2_save, color_frame_2)

# pointcloud = o3d.io.read_point_cloud(cam_2_save)
# cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(cl)
# vis.poll_events()
# vis.update_renderer()

# i = 0
# while True:
    # # Camera 2
    # frames_2 = pipeline_2.wait_for_frames()
    # frames_2 = aligned_stream_2.process(frames_2)
    # color_frame_2 = frames_2.get_color_frame()
    # depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
    # color_image_2 = np.asanyarray(color_frame_2.get_data())
    # depth_image_2 = np.asanyarray(depth_frame_2.get_data())
    # point_cloud_2.map_to(color_frame_2)
    # points_2 = point_cloud_2.calculate(depth_frame_2)

    # print("\nPoints: ", points_2)
    # cam_2_save = 'pc2.ply'
    # points_2.export_to_ply(cam_2_save, color_frame_2)

    # pointcloud = o3d.io.read_point_cloud(cam_2_save)
    # cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # # o3d.visualization.draw_geometries([cl])
    # vis.update_geometry()
    # vis.poll_events()
    # vis.update_renderer()









# vis = o3d.visualization.Visualizer()
# vis.create_window()
# pcd = o3d.geometry.PointCloud()
# vis.add_geometry(pcd)
# pcd.clear()

# while True:
#     # Camera 2
#     frames_2 = pipeline_2.wait_for_frames()
#     frames_2 = aligned_stream_2.process(frames_2)
#     color_frame_2 = frames_2.get_color_frame()
#     depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()
#     point_cloud_2.map_to(color_frame_2)
#     points_2 = point_cloud_2.calculate(depth_frame_2)

#     vtx_2 = np.asanyarray(points_2.get_vertices())
#     pts_2 = [(pt[0], pt[1], pt[2]) for pt in vtx_2]
#     pcd.points = o3d.utility.Vector3dVector(pts_2)
#     vis.update_geometry(pcd)
#     vis.poll_events()
#     vis.update_renderer()