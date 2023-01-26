import numpy as np
import open3d as o3d
import pyrealsense2 as rs

def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
    """
    INSERT FUNCTION DESCRIPTION
    """
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    """
    INSERT FUNCTION DESCRIPTION
    """
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],
                max_correspondence_distance_coarse,
                max_correspondence_distance_fine)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    """
    INSERT FUNCTION DESCRIPTION
    """
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    """
    INSERT FUNCTION DESCRIPTION
    """
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result

def preprocess_point_cloud(pcd, voxel_size):
    """
    INSERT FUNCTION DESCRIPTION
    """
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh




if __name__ == "__main__":
    pc1_path = 'Dataset/Trajectory0/State0/pc1.ply'
    pc2_path = 'Dataset/Trajectory0/State0/pc2.ply'
    pc3_path = 'Dataset/Trajectory0/State0/pc3.ply'
    pc4_path = 'Dataset/Trajectory0/State0/pc4.ply'
    pc5_path = 'Dataset/Trajectory0/State0/pc5.ply'
    # file_path = 'Dataset/Test_Save_Formats/test9.ply'

    pointclouds = []
    # down = []
    # fpfh = []

    # # load in pointclouds
    # for i in range(2,4):
    #     pcd = o3d.io.read_point_cloud(f'Dataset/Trajectory0/State0/pc{i}.ply')

    #     # preprocessing cropping step
    #     center = np.array([0, 0, 0])
    #     radius = 0.9
    #     points = np.asarray(pcd.points)
    #     colors = np.asarray(pcd.colors)
    #     distances = np.linalg.norm(points - center, axis=1)
    #     print("\nDistances Length: ", distances.shape)
    #     print("\nPointcloud Length: ", len(np.asarray(pcd.points)))
    #     indices = np.where(distances <= radius)
    #     pcd.points = o3d.utility.Vector3dVector(points[indices])
    #     pcd.colors = o3d.utility.Vector3dVector(colors[indices])

    #     pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, 0.05)
    #     pointclouds.append(pcd)
    #     down.append(pcd_down)
    #     fpfh.append(pcd_fpfh)


    # load in pointclouds
    for i in range(2,5):
        pcd = o3d.io.read_point_cloud(f'Dataset/Trajectory0/State0/pc{i}.ply')

        # # preprocessing cropping step
        # center = np.array([0, 0, 0])
        # radius = 0.9
        # points = np.asarray(pcd.points)
        # colors = np.asarray(pcd.colors)
        # distances = np.linalg.norm(points - center, axis=1)
        # print("\nDistances Length: ", distances.shape)
        # print("\nPointcloud Length: ", len(np.asarray(pcd.points)))
        # indices = np.where(distances <= radius)
        # pcd.points = o3d.utility.Vector3dVector(points[indices])
        # pcd.colors = o3d.utility.Vector3dVector(colors[indices])

        # downsample
        pcd_down = pcd.voxel_down_sample(voxel_size=0.02)
        pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pointclouds.append(pcd_down)


        # # downsampling with statistical outliers
        # # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # pointclouds.append(pcd)

        
        # # downsample based on statistical outliers
        # cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # pointclouds.append(cl)


        # # no downsmapling
        # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # pointclouds.append(pcd)

    o3d.visualization.draw_geometries(pointclouds)

    print("\nFull registration ...")
    voxel_size = 0.02
    max_correspondence_distance_coarse = voxel_size * 15
    max_correspondence_distance_fine = voxel_size * 1.5
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph = full_registration(pointclouds,
                                    max_correspondence_distance_coarse,
                                    max_correspondence_distance_fine)

    print("\nOptimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)

    # print("Transform points and display")
    # for point_id in range(len(pointclouds)):
    #     print(pose_graph.nodes[point_id].pose)
    #     pointclouds[point_id].transform(pose_graph.nodes[point_id].pose)
    # o3d.visualization.draw_geometries(pointclouds,
    #                                 zoom=0.3412,
    #                                 front=[0.4257, -0.2125, -0.8795],
    #                                 lookat=[2.6172, 2.0475, 1.532],
    #                                 up=[-0.0694, -0.9768, 0.2024])

    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pointclouds)):
        pointclouds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pointclouds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration.pcd", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined_down],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

    
    # voxel_size = 0.05
    # # result_fast = execute_fast_global_registration(down[0], down[1], fpfh[0], fpfh[1], voxel_size)
    # result_ransac = execute_global_registration(down[0], down[1], fpfh[0], fpfh[1], voxel_size)
    # o3d.visualization.draw_geometries([down[0].transform(result_ransac.transformation), down[1]])

    # TODOS:
        # 1) try fusing two point clouds together
        # 2) try fusing all point clouds together
        # 3) pre-process the point clouds (remove background clouds except for surface & table), then try fusing 2 & 5 point clouds together