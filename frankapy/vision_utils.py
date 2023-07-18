import os
import subprocess
import numpy as np
import rospy
from sensor_msgs.msg import Image
from perception import CameraIntrinsics
import cv2
from cv_bridge import CvBridge, CvBridgeError

from frankapy import FrankaArm

from autolab_core import RigidTransform, Point, transformations

ALLOW_PRINT = False

def get_azure_kinect_rgb_image(cv_bridge, topic='/rgb/image_raw'):
    """
    Grabs an RGB image for the topic as argument
    """
    rgb_image_msg = rospy.wait_for_message(topic, Image)
    try:
        rgb_cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return rgb_cv_image

def get_azure_kinect_depth_image(cv_bridge, topic='/depth_to_rgb/image_raw'):
    """
    Grabs an Depth image for the topic as argument
    """
    depth_image_msg = rospy.wait_for_message(topic, Image)
    try:
        depth_cv_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return depth_cv_image

def get_realsense_rgb_image(cv_bridge, topic='/camera/color/image_raw'):
    """
    Grabs an RGB image for the topic as argument
    """
    rgb_image_msg = rospy.wait_for_message(topic, Image)
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_msg)
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
        print(e)
    
    return rgb_cv_image

def get_realsense_depth_image(cv_bridge, topic='/camera/aligned_depth_to_color/image_raw'):
    """
    Grabs an Depth image for the topic as argument
    """
    depth_image_msg = rospy.wait_for_message(topic, Image)
    try:
        depth_cv_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return depth_cv_image

def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print ('Error: Creating directory. ' +  directory)

def get_object_center_point_in_world(object_image_center_x, object_image_center_y, depth_image, intrinsics, transform):    
    
    object_center = Point(np.array([object_image_center_x, object_image_center_y]), 'azure_kinect_overhead')
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    if ALLOW_PRINT:
        print("x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
        object_image_center_x, object_image_center_y, object_depth))
    
    object_center_point_in_world = transform * intrinsics.deproject_pixel(object_depth, object_center)    
    if ALLOW_PRINT:
        print(object_center_point_in_world)

    return object_center_point_in_world 


def get_object_center_point_in_world_realsense(
    object_image_center_x,
    object_image_center_y,
    depth_image,
    intrinsics,
    transform,
    current_pose,):

    object_center = Point(
        np.array([object_image_center_x, object_image_center_y]),
        "realsense_ee",
    )
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    object_center_point_in_world = current_pose * transform * intrinsics.deproject_pixel(
        object_depth, object_center
    )
    return object_center_point_in_world

# def get_object_center_point_in_camera_pixels(
#     NOT FUNCTIONAL
#     object_world_location,
#     intrinsics,
#     transform,
#     current_pose):
    
#     np_point = np.ones((4,1))
#     np_point[0:3, 0] = object_world_location
#     transform_matrix = ((current_pose*transform).inverse()).matrix
#     realsense_location = transform_matrix@np_point
#     # realsense_point[2] = 0
#     realsense_point = Point(realsense_location[0:3, 0], "realsense_ee")
#     return intrinsics.project(realsense_point)

def _reject_outliers(data, m=2):
    """
    Helper function to reject outliers from numpy array.
    """
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)
    return data[s < m]

def get_object_center_point_in_world_realsense_3D_camera_point(
    object_camera_point,
    intrinsics,
    transform,
    current_pose):

    if ALLOW_PRINT:
        print("\nUtils Transform: ", transform)

    object_camera_point = Point(object_camera_point, "realsense_ee")
    object_center_point_in_world = current_pose * transform * object_camera_point
    # object_center_point_in_world = transform * object_camera_point # NOTE: for second stationary camera
    return object_center_point_in_world

def get_object_center_point_in_world_realsense_static_camera(
    object_camera_point,
    intrinsics,
    transform):
    
    object_camera_point = Point(object_camera_point, "realsense_ee")
    object_center_point_in_world = transform * object_camera_point # NOTE: for second stationary camera
    return object_center_point_in_world

def get_object_center_point_in_world_realsense_3D_camera_point_multicam(
    object_camera_point,
    intrinsics,
    transform,
    current_pose,
    ee_cam):

    # if camera is mounted on end effector
    if ee_cam:
        object_camera_point = Point(object_camera_point, "realsense_ee")
        object_center_point_in_world = current_pose * transform * object_camera_point
    # camera mounted statically - do not care about the end effector current pose
    else:
        object_camera_point = Point(object_camera_point, "realsense_ee")
        object_center_point_in_world = transform * object_camera_point # NOTE: for second stationary camera
    return object_center_point_in_world

def get_object_center_point_in_world_realsense_robust(
    object_image_center_x,
    object_image_center_y,
    object_surface_pixel_pairs,
    depth_image,
    intrinsics,
    transform,
    current_pose,):

    object_center = Point(
        np.array([object_image_center_x, object_image_center_y]),
        "realsense_ee",
    )
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    object_center_point_in_world = current_pose * transform * intrinsics.deproject_pixel(
        object_depth, object_center
    )

    # print("Center Deproject: ", intrinsics.deproject_pixel(object_depth, object_center))
    # print("Center in World: ", object_center_point_in_world)

    # NOTE: the depth is the same between the center and rest of the points, so the discrepency is in the deproject section

    object_points = np.zeros((3,len(object_surface_pixel_pairs)))
    # iterate through the different surface pairs
    for i in range(len(object_surface_pixel_pairs)):
        object_point = Point(
            np.array([object_surface_pixel_pairs[i][0], object_surface_pixel_pairs[i][1]]), 
            "realsense_ee",)

        # print("Object Point: ", object_point)

        object_depth = depth_image[object_surface_pixel_pairs[i][0], object_surface_pixel_pairs[i][1]] * 0.001
        object_point_in_world = current_pose * transform * intrinsics.deproject_pixel(
            object_depth, object_point
        )

        # print("Point Deproject: ", intrinsics.deproject_pixel(object_depth, object_point))
        # print("Point in World: ", object_point_in_world)

        object_points[:,i] = np.array([object_point_in_world[0], object_point_in_world[1], object_point_in_world[2]])

    # print the object points calculated
    # print("\nAll Object Points: ", object_points)

    # original method
    if ALLOW_PRINT:
        print("\nOriginal Based on Center Point: ", np.array([object_center_point_in_world[0], object_center_point_in_world[1], object_center_point_in_world[2]]))
    
    # naiively averaging the x,y,z to get the final point
    center_point = np.mean(object_points, axis=1)
    # print("Average All Points: ", center_point)

    # removing outliers in x,y,z prediction then averaging to get final point
    x = np.mean(_reject_outliers(object_points[0,:]))
    y = np.mean(_reject_outliers(object_points[1,:]))
    z = np.mean(_reject_outliers(object_points[2,:]))
    center_point = np.array([x,y,z])
    object_center_point_in_world = center_point
    # print("Remove Outliers and Average All Points: ", center_point)

    # only averaging z to get final point (using object center for x,y prediction)
    z = np.mean(object_points[2,:])
    center_point = np.array([object_center_point_in_world[0], object_center_point_in_world[1], z])
    # print("X, Y from Center Point and Average Z Points: ", center_point)

    # removing z oultiers and then use object center for x,y prediction
    z = np.mean(_reject_outliers(object_points[2,:]))
    center_point = np.array([object_center_point_in_world[0], object_center_point_in_world[1], z])
    # print("X,Y from Center Point and Remove Outliers and Average Z Points: ", center_point)

    # check for z variance and if too high return something else to flag that
    # print("Z variance with outliers: ", np.var(object_points[2,:]))
    # print("Z variance no outliers: ", np.var(_reject_outliers(object_points[2,:])))
    variance = np.var(_reject_outliers(object_points[2,:]))

    # original method
    return object_center_point_in_world, variance

def rotation_matrix_weighted_average(rotation_matrices, weights):
    """
    This function takes two rotation matricies and finds their weighted average. 

    Parameters
    ----------
    rotation_matricies: rotation matricies to be averaged, as numpy arrays
    weights: relative weights of each matrix.

    returns: the weighted average rotation matrix (numpy array) 
    """
    M = np.zeros((4,4))
    for rotation_matrix, weight in zip(rotation_matrices, weights):
        # Given the rotation matrix, calcuate the quaternion in w,x,y,z form.
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        q_xyzw = transformations.quaternion_from_matrix(transform_matrix)
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])

        # Create M = sum(weight*q@q.T)
        M += weight*np.outer(q_wxyz, q_wxyz)

    eig_vals, eig_vecs = np.linalg.eig(M)
    return eig_vecs[:, np.argmax(eig_vals)]

