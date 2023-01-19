import numpy as np
import open3d as o3d
import pyrealsense2 as rs

if __name__ == "__main__":
    file_path = 'Dataset/Trajectory0/State0/pc4.ply'
    # file_path = 'Dataset/Test_Save_Formats/test9.ply'

    # visualize
    print("\nloading point cloud and render...")
    pointcloud = o3d.io.read_point_cloud(file_path)
    print(pointcloud)
    print(np.asarray(pointcloud.points))
    o3d.visualization.draw_geometries([pointcloud])

    # downsample
    print("\ndownsample...")
    cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    o3d.visualization.draw_geometries([cl])



    # print("\nloading point cloud and render...")
    # pointcloud = o3d.io.read_point_cloud(file_path)
    # print(pointcloud)
    # print(np.asarray(pointcloud.points))
    # o3d.visualization.draw_geometries([pointcloud])

    # # print("\ndownsample the point cloud with a voxel of 0.05...")
    # # down_pointcloud = pointcloud.voxel_down_sample(voxel_size=0.05)
    # # o3d.visualization.draw_geometries([down_pointcloud])






    # W = 848
    # H = 480

    # # ----- Camera 2 (static) -----
    # pipeline_2 = rs.pipeline()
    # config_2 = rs.config()
    # config_2.enable_device('151322066099')
    # config_2.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    # config_2.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

    # # start streaming
    # pipeline_2.start(config_2)

    # # align stream
    # aligned_stream_2 = rs.align(rs.stream.color)

    # # point clouds
    # point_cloud_2 = rs.pointcloud()

    # # # added testing
    # # colorizer = rs.colorizer()
    # # points = rs.points()

    # # Camera 2
    # frames_2 = pipeline_2.wait_for_frames()

    # # colorized = colorizer.process(frames_2)
    # # ply = rs.save_to_ply(file_path)
    # # ply.set_option(rs.save_to_ply.option_ply_binary, True)
    # # ply.set_option(rs.save_to_ply.option_ply_normals, True)
    # # ply.process(colorized)
    # # print("Done!")



    # frames_2 = aligned_stream_2.process(frames_2)
    # color_frame_2 = frames_2.get_color_frame()
    # depth_frame_2 = frames_2.get_depth_frame().as_depth_frame()

    # # testing
    # point_cloud_2.map_to(color_frame_2)
    # # points2 = point_cloud_2.calculate(depth_frame_2)

    # points_2 = point_cloud_2.calculate(depth_frame_2)
    # cam_2_save = file_path
    # points_2.export_to_ply(cam_2_save, color_frame_2)




    # # visualize
    # print("\nloading point cloud and render...")
    # pointcloud = o3d.io.read_point_cloud(file_path)
    # print(pointcloud)
    # print(np.asarray(pointcloud.points))
    # o3d.visualization.draw_geometries([pointcloud])

    # # downsample
    # print("\ndownsample...")
    # cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # o3d.visualization.draw_geometries([cl])