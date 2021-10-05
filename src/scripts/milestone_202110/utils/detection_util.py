import os
import sys
import open3d as o3d
import numpy as np
import cv2
import copy
import matplotlib.pyplot as plt

from milestone_config import *
from streaming import *

import SharedArray as sa
import time
import random
from pkg.geometry.geotype import GEOTYPE


IMG_URI = "shm://color_img"
MASK_URI = "shm://mask_img"
REQ_URI = "shm://request"
RESP_URI = "shm://response"

color_img_p = None
return_img_p = None
request_p = None
resp_p = None


def attacth_to_server():
    global color_img_p, return_img_p, request_p, resp_p
    try:
        color_img_p = sa.attach(IMG_URI)
        return_img_p = sa.attach(MASK_URI)
        request_p = sa.attach(REQ_URI)
        resp_p = sa.attach(RESP_URI)
    except Exception as e:
        print(e)


def detect_from_server(image):
    if color_img_p is not None:
        color_img_p[:] = image[:]
        request_p[:] = 1
        while not resp_p[0]:
            time.sleep(0.01)
        resp_p[:] = 0
        return np.copy(return_img_p.astype(np.bool))
    else:
        print("Detect server not attached - call attach_to_server")


SAVE_DIR = os.path.join(MILESTONE_DIR, "save_img")
CROP_DIR = os.path.join(MILESTONE_DIR, "crop_img")
EXP_IMG_DIR = os.path.join(MILESTONE_DIR, "exp_dataset")
MODEL_DIR = os.path.join(MILESTONE_DIR, "model_CAD")
COLOR_PATH = os.path.join(MILESTONE_DIR, "save_img/top_table/color")
DEPTH_PATH = os.path.join(MILESTONE_DIR, "save_img/top_table/depth")
INTRINSIC_PATH = os.path.join(MILESTONE_DIR, "save_img/top_table")

DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (720, 1280)

cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy = [None]*6
__d_scale = None


def set_cam_params(cam_intrins_, d_scale):
    global cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy, __d_scale
    cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy = cam_intrins_
    __d_scale = d_scale


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    FOR_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
#     FOR_model = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
#     FOR_model.transform(transformation)
#     FOR_model.translate(source_temp.get_center() - FOR_model.get_center())
#     FOR_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=target.get_center())
    o3d.visualization.draw_geometries([source_temp, target_temp, FOR_origin])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 6
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=200))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, model_mesh, target):
    source = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.9))

    print(":: Load two point clouds and disturb initial pose.")
    trans_init = np.identity(4)
    # trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
    #                              [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(True), 3, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


# def compute_color_icp(model_mesh, target, initial_guess):
#     source = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.9))
#
#     #     voxel_radius = [0.03, 0.01, 0.003]
#     #     max_iter = [60, 40, 30]
#     voxel_radius = [0.04, 0.02, 0.01]
#     max_iter = [1000, 400, 300]
#
#     current_transformation = initial_guess
#     draw_registration_result(source, target, current_transformation)
#
#     for scale in range(3):
#         iter = max_iter[scale]
#         radius = voxel_radius[scale]
#         #         print([iter, radius, scale])
#         #         print("3-1. Downsample with a voxel size %.4f" % radius)
#         source_down = source.voxel_down_sample(radius)
#         target_down = target.voxel_down_sample(radius)
#
#         #         source_down = source
#         #         target_down = target
#         #         print("3-2. Estimate normal.")
#         source_down.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
#         target_down.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
#
#         #         print("3-3. Applying colored point cloud registration")
#         result_icp = o3d.registration.registration_colored_icp(
#             source_down, target_down, radius, current_transformation,
#             o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-8,
#                                                     relative_rmse=1e-8,
#                                                     max_iteration=iter))
#         current_transformation = result_icp.transformation
#         print(result_icp)
#     draw_registration_result(source, target, result_icp.transformation)
#     print("Total result ICP model fitting")
#     print(result_icp.transformation)
#     return result_icp.transformation


def compute_ICP(model_mesh, pcd, initial_guess):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.9))
    source = copy.deepcopy(model_pcd)


    # Guess Initial Transformation
    trans_init = initial_guess
    # draw_registration_result(source, target, trans_init)

    print("Apply point-to-point ICP")
    threshold = 0.07
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-9,
                                                                                        relative_rmse=1e-9,
                                                                                        max_iteration=500000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    return ICP_result

def process_bed_detection():
    # Load CAD model of bed
    bed_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed.STL')
    # bed_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed_floor_centered_m_scale.STL')
    bed_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(bed_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # Load PCD of bed
    color = o3d.io.read_image(CROP_DIR + '/bed_crop.jpg')
    depth = o3d.io.read_image(CROP_DIR + '/bed_crop.png')
    # color = o3d.io.read_image(EXP_IMG_DIR + '/526.jpg')
    # depth = o3d.io.read_image(EXP_IMG_DIR + '/526.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
                                                        depth_trunc = 10.0, convert_rgb_to_intensity = False)
    pcd_bed = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))
    # cl, ind = pcd_bed.remove_radius_outlier(nb_points=20, radius=0.07)
    # pcd_bed = cl
    o3d.visualization.draw_geometries([pcd_bed])

    voxel_size = 0.05
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, bed_model, pcd_bed)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    print(result_ransac.transformation)
    draw_registration_result(source_down, target_down,
                             result_ransac.transformation)

    ICP_result= compute_ICP(bed_model, pcd_bed, result_ransac.transformation)
    # ICP_result= compute_color_icp(bed_model, pcd_bed, result_ransac.transformation)

    return ICP_result


def extract_outliers(pcd_points, inliers):
    temp = []

    for i in range(len(pcd_points)):
        idx = i
        if not idx in inliers:
            temp.append(pcd_points[idx])

    p_outliers = np.zeros((len(temp), 3))
    for i in range(len(temp)):
        p_outliers[i] = temp[i]
    return p_outliers


def remove_background(pcd_points, thres):
    plane_model, inliers = pcd_points.segment_plane(distance_threshold=thres,
                                                   ransac_n=5,
                                                   num_iterations=1400)
    pcd_outliers = o3d.geometry.PointCloud()
    pcd_outliers.points = o3d.utility.Vector3dVector(extract_outliers(np.asarray(pcd_points.points), inliers))
    o3d.visualization.draw_geometries([pcd_outliers])
    return pcd_outliers


def remove_bed(pcd_original, pcd_bed):
    dists = pcd_original.compute_point_cloud_distance(pcd_bed)
    dists = np.asarray(dists)

    idx = np.where(dists > 0.07)[0]
    p_inliers = []
    for i in range(len(idx)):
        p_inliers.append(pcd_original.points[idx[i]])

    return p_inliers


def check_location_top_table(color_path, depth_path, T_bc, T_bo):
    # Load total PCD of first view
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / __d_scale,
                                                            depth_trunc=10.0, convert_rgb_to_intensity = False)
    pcd_total = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))

    pcd_total = pcd_total.uniform_down_sample(every_k_points=11)
    o3d.visualization.draw_geometries([pcd_total])

    # Remove background(wall & ground plane)
    # First, Remove wall
    pcd_outliers = remove_background(pcd_total, thres=0.067)

    # # Second, Remove ground
    # pcd_outliers_2 = remove_background(pcd_outliers)

    # Load PCD of bed
    color = o3d.io.read_image(CROP_DIR + '/bed_crop.jpg')
    depth = o3d.io.read_image(CROP_DIR + '/bed_crop.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
                                                                depth_trunc = 10.0, convert_rgb_to_intensity = False)
    pcd_bed = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))

    # Remove bed
    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(remove_bed(pcd_outliers, pcd_bed))
    o3d.visualization.draw_geometries([pcd_top_table])

    # Remove other noise
    cl, ind = pcd_top_table.remove_radius_outlier(nb_points=25, radius=0.3)
    pcd_top_table = cl
    # o3d.visualization.draw_geometries([pcd_top_table])


    # Determine rough location of top_table
    points = np.asarray(pcd_top_table.points)
    # points_4d = np.zeros((len(points)),4)
    points_4d = []
    points_temp = []
    points_transformed = []
    for i in range(len(points)):
        points_4d.append(np.hstack([points[i], [1]]))

    for i in range(len(points_4d)):
        points_temp.append(np.matmul(T_bc, points_4d[i]))

    for i in range(len(points_temp)):
        points_transformed.append(np.matmul(np.linalg.inv(T_bo), points_temp[i]))


    check_left = 0
    check_right = 0
    for i in range(len(points_transformed)):
        if points_transformed[i][1] > 0:
            check_right += 1
        elif points_transformed[i][1] < 0:
            check_left += 1

    TOP_TABLE_MODE = "LEFT"
    if check_left > check_right:
        TOP_TABLE_MODE = "LEFT"
    else:
        TOP_TABLE_MODE = "RIGHT"
    # top_table_center = pcd_top_table.get_center()
    # bed_center = pcd_bed.get_center()
    #
    # if ROBOT_LOCATION == "RIGHT":
    #     if np.linalg.norm(top_table_center) > np.linalg.norm(bed_center):
    #         TOP_TABLE_MODE = "LEFT"
    #     else:
    #         TOP_TABLE_MODE = "RIGHT"
    # elif ROBOT_LOCATION == "LEFT":
    #     if np.linalg.norm(top_table_center) > np.linalg.norm(bed_center):
    #         TOP_TABLE_MODE = "RIGHT"
    #     else:
    #         TOP_TABLE_MODE = "LEFT"

    return TOP_TABLE_MODE


def save_intrinsic_as_json(filename):
    # intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'color_format':
                    "RGB8",
                'depth_format':
                    "Z16",
                'depth_scale':
                    1/__d_scale,
                'device_name':
                    "Intel RealSense L515",
                'fps':
                    30.0,
                'width':
                    cam_width,
                'height':
                    cam_height,
                'intrinsic_matrix': [
                    cam_fx, 0.0, 0.0, 0.0, cam_fy, 0.0, cam_ppx,
                    cam_ppy, 1.0
                ]
                # 'serial_number':
                #     "f0271852"
            },
            outfile,
            indent=8)


# def check_top_table_exist(check_color_path, check_depth_path):
#     # Load PCD of top table check image
#     color = o3d.io.read_image(check_color_path)
#     depth = o3d.io.read_image(check_depth_path)
#     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
#                                                                 depth_trunc = 10.0, convert_rgb_to_intensity = False)
#     pcd_top_table = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
#                                                                 o3d.camera.PinholeCameraIntrinsic(cam_width,
#                                                                                                   cam_height, cam_fx,
#                                                                                                   cam_fy,
#                                                                                                   cam_ppx, cam_ppy))
#     pcd_top_table = pcd_top_table.uniform_down_sample(every_k_points=11)
#     pcd_remove = remove_background(pcd_top_table, thres=0.03)
#
#     n = len(np.asarray(pcd_remove.points))
#     if n < int(len(np.asarray(pcd_top_table.points)) * 0.45):
#         return False
#     else:
#         return True



def process_top_table_detection():
    # Load CAD model of top table
    top_table_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/top_table/top_table.STL')
    top_table_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(top_table_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # Load PCD of top table (obtained from reconstruction)
    pcd_top_table = o3d.io.read_point_cloud(MILESTONE_DIR + "/pcd.ply")
    pcd_top_table = pcd_top_table.uniform_down_sample(every_k_points=11)
    o3d.visualization.draw_geometries([pcd_top_table])

    pcd_top_table = remove_background(pcd_top_table, thres=0.03)

    voxel_size = 0.05  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, top_table_model,
                                                                                         pcd_top_table)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    print(result_ransac.transformation)
    draw_registration_result(source_down, target_down,
                             result_ransac.transformation)

    ICP_result = compute_ICP(top_table_model, pcd_top_table, result_ransac.transformation)
    return ICP_result


