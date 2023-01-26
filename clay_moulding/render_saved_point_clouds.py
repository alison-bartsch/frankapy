import numpy as np
import open3d as o3d
import pyrealsense2 as rs

if __name__ == "__main__":
    pc1_path = 'Dataset/Trajectory0/State0/pc1.ply'
    pc2_path = 'Dataset/Trajectory3/State0/pc2.ply'
    pc3_path = 'Dataset/Trajectory0/State0/pc3.ply'
    pc4_path = 'Dataset/Trajectory0/State0/pc4.ply'
    pc5_path = 'Dataset/Trajectory0/State0/pc5.ply'

    # visualize
    print("\nloading point cloud and render...")
    pointcloud = o3d.io.read_point_cloud(pc2_path)
    print(pointcloud)
    print("\nPoints: ", pointcloud.points)
    print("\nColors: ", pointcloud.colors)
    o3d.visualization.draw_geometries([pointcloud])

    # statistical outlier removal
    cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    o3d.visualization.draw_geometries([cl])

    # remove points outside of sphere
    center = np.array([0, 0, 0])
    radius = 0.9
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors)
    distances = np.linalg.norm(points - center, axis=1)
    print("\nDistances Length: ", distances.shape)
    print("\nPointcloud Length: ", len(np.asarray(pointcloud.points)))
    indices = np.where(distances <= radius)
    pointcloud.points = o3d.utility.Vector3dVector(points[indices])
    pointcloud.colors = o3d.utility.Vector3dVector(colors[indices])

    o3d.visualization.draw_geometries([pointcloud])


    # downsampling
    down_pointcloud = pointcloud.voxel_down_sample(voxel_size=0.005)
    o3d.visualization.draw_geometries([down_pointcloud])


    
    # # crop pointcloud
    # min_bound = np.array([-0.9, -0.9, -0.9])
    # max_bound = np.array([0.9, 0.9, 0.9])
    # cropped = o3d.geometry.crop_point_cloud(pointcloud, min_bound, max_bound)
    # o3d.visualization.draw_geometries([cropped])




    # # downsample
    # print("\ndownsample...")
    # cl, ind = pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # o3d.visualization.draw_geometries([cl])






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