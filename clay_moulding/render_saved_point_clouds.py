import numpy as np
import open3d as o3d
import os 
#import pyrealsense2 as rs

def load_data(path):
    """
    Load point cloud from .ply file path input and print properties
    """
    print("Loading point cloud...")
    point_cloud = o3d.io.read_point_cloud(path)
    #print(point_cloud)
    #print("\nPoints: ", point_cloud.points)
    #print("\nColors: ", point_cloud.colors)
    return point_cloud
    
def remove_outliers(point_cloud):
    """
    1. Accept raw point cloud 
    2. Remove statistical outliers 
    3. Return filtered point cloud
    """
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return cl

def sphere_crop(point_cloud):
    """
    1. Accept raw point cloud or point cloud with outliers removed
    2. Crop points based on defined sphere parameters
    3. Return cropped point cloud
    """
    # define parameters for sphere crop
    center = np.array([0, 0, 0])
    radius = 0.9

    # split point cloud into verticies and color data
    points = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors) 
    distances = np.linalg.norm(points - center, axis=1)
    #print("\nDistances Length: ", distances.shape)
    #print("\nPointcloud Length: ", len(np.asarray(point_cloud.points)))
    
    # crop verticies given sphere parameters
    indices = np.where(distances <= radius)
    point_cloud.points = o3d.utility.Vector3dVector(points[indices])
    point_cloud.colors = o3d.utility.Vector3dVector(colors[indices])

    return point_cloud

def process_pointclouds():
    """
    1. Loop through all .ply files in 'Dataset'
    2. Remove statistical outliers and save with '_filtered.ply' in new directory
    4. Crop point cloud based on defined sphere and save with '_cropped.ply' in new directory
    """
    trajectory_fpath = 'Dataset'
    for traj in os.listdir(trajectory_fpath): # loops through all trajectory folders
        for state in os.listdir(trajectory_fpath + '/' + traj): # loops through all states in those trajectory folders
            if state.startswith ('S'): # ignores .npy action files
                for f in os.listdir(trajectory_fpath + '/' + traj + '/' + state): # loops through all .ply files
                    if f.endswith('.ply'):
                        print(f'\n\nTrajectory: {traj}\nState: {state}\nFile: {f}') # print to ensure proper indexing of folders

                        # load raw point cloud
                        point_cloud = load_data(trajectory_fpath + '/' + traj + '/' + state + '/' + f)

                        # statistical outlier removal
                        cl = remove_outliers(point_cloud)

                        # save point cloud with statistical outliers removed with '_filtered.ply'
                        new_fn = f[:-4] + '_filtered.ply' # edit filename
                        filtered_fpath = trajectory_fpath + '/' + traj + '/' + state + '/outliers_removed'
                        
                        try: 
                            os.mkdir(filtered_fpath) # attempt to make new directory for filtered point clouds
                            print('Saving filtered .ply to new directory...') 
                            o3d.io.write_point_cloud(filtered_fpath + '/' + new_fn, cl) 
                        
                        except OSError as error: 
                            print('Directory already exists, saving filtered .ply to directory...')
                            o3d.io.write_point_cloud(filtered_fpath + '/' + new_fn, cl) # if folder exists, save to existing folder
                        
                        # crop points outside sphere
                        cropped_pc = sphere_crop(point_cloud)

                        # save cropped point cloud with '_cropped.ply'
                        new_fn = f[:-4] + '_cropped.ply' # edit filename
                        cropped_fpath = trajectory_fpath + '/' + traj + '/' + state + '/cropped'

                        try: 
                            os.mkdir(cropped_fpath) # attempt to make new directory for filtered point clouds
                            print('Saving cropped .ply to new directory...') 
                            o3d.io.write_point_cloud(cropped_fpath + '/' + new_fn, cropped_pc) 
                        
                        except OSError as error: 
                            print('Directory already exists, saving cropped .ply to directory...')
                            o3d.io.write_point_cloud(cropped_fpath + '/' + new_fn, cropped_pc) # if folder exists, save to existing folder

if __name__ == "__main__":
    
    prompt = input('\n\nThis script will process all .ply files in the Dataset directory and save new filtered and cropped point clouds.\nWould you like to proceed? [y/n]\n>')

    if prompt == 'y':
        process_pointclouds()


    # load and visualize individual point clouds
    """filtered_fpath = 'Dataset/Trajectory0/State0/outliers_removed/'
    cropped_fpath = 'Dataset/Trajectory0/State0/cropped/'
    pc2_path = 'Dataset/Trajectory3/State0/'
    pc2_f = 'pc2.ply'
    pc3_path = 'Dataset/Trajectory0/State0/pc3.ply'
    pc4_path = 'Dataset/Trajectory0/State0/pc4.ply'
    pc5_path = 'Dataset/Trajectory0/State0/pc5.ply'

    point_cloud = load_data(filtered_fpath + 'pc5_filtered.ply')
    print('Displaying filtered point cloud...')
    o3d.visualization.draw_geometries([point_cloud])

    point_cloud = load_data(cropped_fpath + 'pc5_cropped.ply')
    print('Displaying cropped point cloud...')
    o3d.visualization.draw_geometries([point_cloud])

    cl = remove_outliers(point_cloud)
    o3d.visualization.draw_geometries([cl])
    new_fn = pc2_f[:-4] + '_filtered.ply' # edit filename
    ply_filtered = 'Dataset' + '/' + 'Trajectory0' + '/' + 'State0' + '/' + new_fn
    o3d.io.write_point_cloud(ply_filtered, cl)

    # make sure that worked
    point_cloud = load_data(ply_filtered)
    o3d.visualization.draw_geometries([point_cloud])"""
    
    
    # downsampling
    #down_pointcloud = pointcloud.voxel_down_sample(voxel_size=0.005)
    #o3d.visualization.draw_geometries([down_pointcloud])

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