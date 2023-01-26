import yaml
import numpy as np
import open3d as o3d

cam_a = 2
cam_b = 4

filename = "Calibration_Dataset/calibration_cams" + str(cam_a) + str(cam_b) + ".yaml" # "23.yaml"
with open(filename, "r") as f:
    calibration_params = yaml.load(f, Loader=yaml.FullLoader)

axis_angle = calibration_params['sensors']['cam' + str(cam_b)]['extrinsics']['axis_angle']
translation = calibration_params['sensors']['cam' + str(cam_b)]['extrinsics']['translation']
print("\nAxis Angle: ", axis_angle)
print("Translation: ", translation)

transformation = np.eye(4)
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
transformation[:3, :3] = mesh.get_rotation_matrix_from_xyz(axis_angle)
transformation[:3,3] = -np.array(translation)

print("\nTransform: ", transformation)
np.save("transform_cam" + str(cam_b) + "_to_cam" + str(cam_a) + ".npy", transformation)


pca_path = 'Dataset/Trajectory0/State0/pc' + str(cam_a) + '.ply'
pcb_path = 'Dataset/Trajectory0/State0/pc' + str(cam_b) + '.ply'

# visualize
print("\nloading point cloud and render...")
pca = o3d.io.read_point_cloud(pca_path)

# statistical outlier removal
cla, ind = pca.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
o3d.visualization.draw_geometries([cla])

# visualize
print("\nloading point cloud and render...")
pcb = o3d.io.read_point_cloud(pcb_path)
pcb.transform(transformation)
# translation = -2*np.array(translation)
# print("\nTranslation: ", translation)
# pc3.translate(translation)

# statistical outlier removal
clb, ind = pcb.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
o3d.visualization.draw_geometries([clb])

# combine point clouds
# pcd_combined = o3d.geometry.PointCloud()
pointclouds = [cla, clb]
o3d.visualization.draw_geometries(pointclouds)



# import yaml files 
# get the axis angle and translation info
# create transformation umpy matrices
# save the numpy transformation matrices
# test one of the transformation matrices by applying it to a point cloud??