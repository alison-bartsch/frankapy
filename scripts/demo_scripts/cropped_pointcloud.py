import cv2
import numpy as np
import pyrealsense2 as rs
# from autolab_core import RigidTransform
# from frankapy import FrankaArm
# from frankapy.vision_utils import *
import open3d as o3d

"""
This file is to use as a reference for how to call each camera by the 
respective serial number as well as which transform file is applicable
for each camera.
"""
W = 848
H = 480




# ----- Camera 1 (end-effector) -----
REALSENSE_INTRINSICS_CAM_1 = "calib/realsense_intrinsics.intr"
REALSENSE_TF_CAM_1 = "calib/realsense_ee_orginal.tf"

if __name__ == "__main__":
    # fx =  621.70715
    # fy = 621.9764
    # cx = 323.3644
    # cy = 244.8789
    cam = o3d.camera.PinholeCameraIntrinsic(W, H, 621.70715, 621.9764, 323.3644, 244.8789)
    color_img = cv2.imread("color/frame_1950.jpg")
    depth_img = cv2.imread("depth/frame_1950.jpg")
    depths = np.asarray(depth_img*1500/255.0).astype(np.float32)
    
    color_img = cv2.rectangle(color_img, (425,105), (580,160),color=(255, 0, 0),thickness= 2)
    # print
    # print(type(depths[0,0,0]))
    color_image = o3d.geometry.Image(color_img)
    depth_image = o3d.geometry.Image(depths)
    # color_image = o3d.geometry.Image(color_img[:848, :480])
    # depth_image = o3d.geometry.Image(depths[:848, :480])
    # cam.width = 200
    # cam.height = 100
    # cam = o3d.camera.PinholeCameraIntrinsic(200, 100, 621.70715, 621.9764, 323.3644, 244.8789)
    
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / 0.001,
                depth_trunc= 1,
                convert_rgb_to_intensity=False)
    temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, cam)
    # temp = temp.uniform_down_sample(every_k_points=5)
    cl, ind = temp.remove_statistical_outlier(nb_neighbors=200,
                                                    std_ratio=1.0)
    # o3d.visualization.draw_geometries([temp])
    o3d.visualization.draw_geometries([temp.select_by_index(ind)])
    
    cv2.imshow("color", color_img)
    cv2.imshow("depth", depth_img)
    
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyAllWindows()
    # pipeline_1 = rs.pipeline()
    # config_1 = rs.config()
    # config_1.enable_device('220222066259')
    # config_1.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    # config_1.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    # colorizer = rs.colorizer()
    # p1 = pipeline_1.start(config_1)
    # ds1 = p1.get_device().first_depth_sensor().get_depth_scale()
    # aligned_stream_1 = rs.align(rs.stream.color)
    # point_cloud = rs.pointcloud()
    # fa = FrankaArm()
    
    # reset franka to its home joints
    # fa.close_gripper()
    # # fa.reset_pose()
    # fa.reset_joints()
    # pose = fa.get_pose()
    # print("\nRobot Pose: ", pose)
    # transform = RigidTransform.load(REALSENSE_TF_CAM_1)
    
    # pose.translation = np.array([3.07052791e-01, -5.60250537e-06, 0.4])
    # # # pose.rotation = 
    # fa.goto_pose(pose)

    # fourcc = cv2.VideoWriter_fourcc(*'XVID') 
    # color_filename = 'data_color.avi' 
    # depth_filename = 'data_depth.avi' 
    # output_fps = 30.0  # Frames per second
    # output_size = (W, H) 
    # color_video = cv2.VideoWriter(color_filename, fourcc, output_fps, output_size)
    # depth_video = cv2.VideoWriter(depth_filename, fourcc, output_fps, output_size)


    # for i in range(650):
    # cv2.namedWindow('My Window')
    # image = 255*np.ones((H, W, 3), dtype=np.uint8)
    
    
    # i = 0
    # while(True):
    #     # Camera 1
    #     frames_1 = pipeline_1.wait_for_frames()
    #     frames_1 = aligned_stream_1.process(frames_1)
    #     if i == 0:
    #         intrinsics = frames_1.profile.as_video_stream_profile().intrinsics
    #         out = o3d.camera.PinholeCameraIntrinsic(848, 480, intrinsics.fx,
    #                                                 intrinsics.fy, intrinsics.ppx,
    #                                                 intrinsics.ppy)
    #         i += 1
    #     color_frame_1 = frames_1.get_color_frame()
    #     color_image_1 = np.asanyarray(color_frame_1.get_data())
    #     point_cloud.map_to(color_frame_1)
        
    #     depth_frame_1 = frames_1.get_depth_frame().as_depth_frame()
    #     depth_image_1 = np.asanyarray(depth_frame_1.get_data())
    #     depth_colormap = np.asanyarray(colorizer.colorize(depth_frame_1).get_data())
    #     depths = np.clip(depth_image_1, 0, 1500)
    #     # depth_image_1 = np.uint16(depths)*255.0/1500.0
    #     depth_image_1 = (depths * 255.0 / 1500.0).astype(np.uint8)
    #     # print(type(depth_frame_1))
        
    #     points = point_cloud.calculate(depth_frame_1)
    #     tex = np.asanyarray(points.get_texture_coordinates()).view(np.float32)
    #     verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)
    #     # print(type(verts))
    #     xyz = get_object_center_point_in_world_realsense_3D_camera_point(verts[0,0], np.eye(3), transform, fa.get_pose())
    #     # Show the images
    #     cv2.imshow("Camera",color_image_1 )
    #     cv2.imshow("Camera 1 Depth", depth_image_1)
    #     cv2.imshow("Camera 1 depth colormap", depth_colormap)
    #     # color_video.write(color_image_1)
    #     # depth_video.write(depth_image_1)
        
    #     k = cv2.waitKey(1)
    #     if k==27:
    #         break
    #     elif k== ord('s'):
    #         print("Saving....")
    #         pcd = o3d.geometry.PointCloud()
    #         pcd.points = o3d.utility.Vector3dVector(verts.reshape(-1, 3))
    #         o3d.io.write_point_cloud("data.ply", pcd)
    #         # # ply = rs.save_to_ply("pcd.ply")
    #         pcd_load = o3d.io.read_point_cloud("data.ply")
    #         uni_down_pcd = pcd.uniform_down_sample(every_k_points=20)
    #         # o3d.visualization.draw_geometries([pcd_load])
    #         o3d.visualization.draw_geometries([uni_down_pcd])
    #         cl, ind = uni_down_pcd.remove_statistical_outlier(nb_neighbors=500,
    #                                                 std_ratio=1.0)
    #         o3d.visualization.draw_geometries([uni_down_pcd.select_by_index(ind)])
    #         print(len(pcd_load.points),len(uni_down_pcd.points),len(uni_down_pcd.select_by_index(ind).points))
            
            
    #         # depth_image = o3d.geometry.Image(
    #         #     np.array(depth_frame_1.get_data()))
    #         # color_temp = np.asarray(color_frame_1.get_data())
    #         # color_image = o3d.geometry.Image(color_temp)

    #         # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    #         #     color_image,
    #         #     depth_image,
    #         #     depth_scale=1.0 / ds1,
    #         #     depth_trunc=1.5,
    #         #     convert_rgb_to_intensity=False)
    #         # temp = o3d.geometry.PointCloud.create_from_rgbd_image(
    #         #     rgbd_image, out)
    #     #     # Save the image
    #     #     cam1_filename_c = "/socket_Imgs/color/" + str(i) + "_cam1.jpg"
    #     #     cam1_filename_d = "/socket_Imgs/depth/" + str(i) + "_cam1.png"
            
    #     #     print(np.unique((depth_image_1/255.0)*1500*ds1))
    #     #     # print(np.unique(depth_colormap))
            
    #     #     cv2.imwrite(cam1_filename_c, color_image_1)
    #     #     cv2.imwrite(cam1_filename_d, depth_image_1)
    #     #     i += 1
            
            
    #     elif k == ord('x'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation += [0.1, 0., 0.]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
    #     elif k == ord('y'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation += [0., 0.1, 0.]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
    #     elif k == ord('z'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation += [0., 0., 0.1]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
            
    #     elif k == ord('c'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation -= [0.1, 0., 0.]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
    #     elif k == ord('u'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation -= [0., 0.1, 0.]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
    #     elif k == ord('a'):
    #         T_ee_world = fa.get_pose()
    #         print('Current pose: ', T_ee_world)
    #         T_ee_world.translation -= [0., 0., 0.1]
    #         fa.goto_pose(T_ee_world)
    #         print('New pose: ', fa.get_pose())
            
        
            
            
    # cv2.destroyAllWindows()