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
from pkg.utils.rotation_utils import *


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

    FOR_model = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    FOR_model.transform(transformation)
    FOR_model.translate(source_temp.get_center() - FOR_model.get_center())

    FOR_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=target.get_center())

    o3d.visualization.draw_geometries([source_temp, target_temp,
                                       FOR_origin, FOR_model, FOR_target])

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
    source = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.7))

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
    distance_threshold = voxel_size * 1.4
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 3, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result



def compute_ICP(model_mesh, pcd, initial_guess, ratio, thres, visualize=False):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * ratio))
    source = copy.deepcopy(model_pcd)

    # Guess Initial Transformation
    trans_init = initial_guess

    print("Apply point-to-point ICP")
    threshold = thres
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-17,
                                                                                        relative_rmse=1e-17,
                                                                                        max_iteration=500000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    if visualize:
        draw_registration_result(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    return ICP_result, reg_p2p.fitness



# def compute_ICP_plane(model_mesh, pcd, initial_guess, ratio, thres, visualize=False):
#     # Compute ICP to align model(source) to obtained point clouds(target)
#     target = copy.deepcopy(pcd)
#     model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * ratio))
#     source = copy.deepcopy(model_pcd)
#
#     radius_normal = 0.05
#     target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
#     source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
#
#     # Guess Initial Transformation
#     trans_init = initial_guess
#
#     print("Apply point-to-plane ICP")
#     threshold = thres
#     reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
#                                                 o3d.registration.TransformationEstimationPointToPlane(),
#                                                 o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-15,
#                                                                                         relative_rmse=1e-15,
#                                                                                         max_iteration=500000))
#     print(reg_p2p)
#     print("Transformation is:")
#     print(reg_p2p.transformation)
#     if visualize:
#         draw_registration_result(source, target, reg_p2p.transformation)
#     ICP_result = reg_p2p.transformation
#
#     return ICP_result, reg_p2p.fitness


def compute_close_ICP(model_mesh, pcd, initial_guess, thres, visualize=False,
                      relative_fitness=1e-13, relative_rmse=1e-13, max_iteration=600000):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.3))
    source = copy.deepcopy(model_pcd)

    # Guess Initial Transformation
    trans_init = initial_guess

    print("Apply point-to-point ICP")
    threshold = thres
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(relative_fitness=relative_fitness,
                                                                                        relative_rmse=relative_rmse,
                                                                                        max_iteration=max_iteration))
    compute_close_ICP.reg_p2p = reg_p2p
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    if visualize:
        draw_registration_result(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    return ICP_result


def compute_front_ICP(model_type, pcd, initial_guess, T_off, thres, visualize=False,
                      relative_fitness=1e-13, relative_rmse=1e-13, max_iteration=600000):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    if model_type=="bed":
        model_mesh = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed.STL')
        model_mesh.vertices = o3d.utility.Vector3dVector(
            np.asarray(model_mesh.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))
        model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.7))
        max_dist = model_pcd.get_center()[2] * 1.6
    elif model_type=="closet":
        model_mesh = o3d.io.read_triangle_mesh(MODEL_DIR + '/top_table/top_table.STL')
        model_mesh.vertices = o3d.utility.Vector3dVector(
            np.asarray(model_mesh.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))
        model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points)) * 0.7))
        max_dist = model_pcd.get_center()[0] * 1.5

    model_pcd_bed = np.asarray(model_pcd.points)
    idx = []
    for i in range(len(model_pcd_bed)):
        if model_pcd_bed[i, 2] > max_dist:
            idx.append(i)

    pts = np.zeros((len(idx), 3))
    for i in range(len(idx)):
        pts[i] = model_pcd_bed[idx[i]]

    front_pcd = o3d.geometry.PointCloud()
    front_pcd.points = o3d.utility.Vector3dVector(pts)
    source = copy.deepcopy(front_pcd)
    if visualize:
        vis_pointcloud(source)

    # Guess Initial Transformation
    trans_init = initial_guess

    print("Apply point-to-point ICP")
    threshold = thres
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(relative_fitness=relative_fitness,
                                                                                        relative_rmse=relative_rmse,
                                                                                        max_iteration=max_iteration))
    compute_close_ICP.reg_p2p = reg_p2p
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    if visualize:
        draw_registration_result(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    return np.matmul(ICP_result, T_off)


def process_bed_detection_front(T_bc, visualize=False):
    # Load CAD model of bed
    bed_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed.STL')
    bed_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(bed_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))

    # Load PCD of bed
    color = o3d.io.read_image(CROP_DIR + '/bed_crop.jpg')
    depth = o3d.io.read_image(CROP_DIR + '/bed_crop.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / __d_scale,
                                                                    depth_trunc=8.0, convert_rgb_to_intensity=False)
    pcd_bed = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                             o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                               cam_height, cam_fx,
                                                                                               cam_fy,
                                                                                               cam_ppx, cam_ppy))
    # Remove other noise
    cl, ind = pcd_bed.remove_radius_outlier(nb_points=20, radius=0.07)
    pcd_bed = cl
    if visualize:
        o3d.visualization.draw_geometries([pcd_bed])

    # Get distance of pcd
    center_p = pcd_bed.get_center()
    distance = np.linalg.norm(center_p)

    z = T_bc[2, 3]
    T_cb = SE3_inv(T_bc)
    T = T_cb
    T1 = np.identity(4)
    T1[:3, :3] = Rot_axis(2, -np.pi / 2)
    T2 = np.identity(4)
    T2[:3, :3] = Rot_axis(3, -np.pi / 2)
    T_ = np.matmul(T1, T2)
    length = np.sqrt(distance * distance - z * z)
    T_[:3, 3] += (length + 2.02, 0.455, 0)

    #     T = np.matmul(np.matmul(T_cb, T1), T2)
    T = np.matmul(T_cb, T_)

    # Calculate intial guess
    initial_guess = np.identity(4)
    #     initial_guess[:3,:3] = Rot_axis(1, np.pi)
    initial_guess[:3, :3] = T[:3, :3]
    initial_guess[:3, 3] = T[:3, 3]

    voxel_size = 0.038
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, bed_model, pcd_bed)

    #     result_ransac = execute_global_registration(source_down, target_down,
    #                                                 source_fpfh, target_fpfh,
    #                                                 voxel_size)

    #     print(result_ransac.transformation)
    if visualize:
        draw_registration_result(source_down, target_down, initial_guess)

    ICP_result, fitness = compute_ICP(bed_model, pcd_bed, initial_guess, ratio=0.7, thres=0.16, visualize=visualize)

    return ICP_result, fitness



def process_bed_detection(visualize=False):
    # Load CAD model of bed
    bed_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed.STL')
    bed_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(bed_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # Load PCD of bed
    color = o3d.io.read_image(CROP_DIR + '/bed_crop.jpg')
    depth = o3d.io.read_image(CROP_DIR + '/bed_crop.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
                                                        depth_trunc = 8.0, convert_rgb_to_intensity = False)
    pcd_bed = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))
    # Remove other noise
    cl, ind = pcd_bed.remove_radius_outlier(nb_points=20, radius=0.07)
    pcd_bed = cl
    if visualize:
        o3d.visualization.draw_geometries([pcd_bed])

    voxel_size = 0.038
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, bed_model, pcd_bed)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    print(result_ransac.transformation)
    if visualize:
        draw_registration_result(source_down, target_down,
                                 result_ransac.transformation)

    ICP_result, fitness = compute_ICP(bed_model, pcd_bed, result_ransac.transformation, ratio=0.7, thres=0.16, visualize=visualize)

    return ICP_result, fitness



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


def get_inliers(pcd_points, inliers):
    temp = []

    for i in range(len(pcd_points)):
        idx = i
        if idx in inliers:
            temp.append(pcd_points[idx])

    p_inliers = np.zeros((len(temp), 3))
    for i in range(len(temp)):
        p_inliers[i] = temp[i]
    return p_inliers


def remove_background(pcd_points, thres, visualize=False):
    plane_model, inliers = pcd_points.segment_plane(distance_threshold=thres,
                                                   ransac_n=5,
                                                   num_iterations=1400)
    pcd_outliers = o3d.geometry.PointCloud()
    pcd_outliers.points = o3d.utility.Vector3dVector(extract_outliers(np.asarray(pcd_points.points), inliers))
    if visualize:
        vis_pointcloud(pcd_outliers)

    return pcd_outliers


def get_plane(pcd_points, thres, visualize=False):
    plane_model, inliers = pcd_points.segment_plane(distance_threshold=thres,
                                                   ransac_n=5,
                                                   num_iterations=1400)

    pcd_inliers = o3d.geometry.PointCloud()
    pcd_inliers.points = o3d.utility.Vector3dVector(get_inliers(np.asarray(pcd_points.points), inliers))
    if visualize:
        vis_pointcloud(pcd_inliers)

    return pcd_inliers


def remove_bed(pcd_original, pcd_bed):
    dists = pcd_original.compute_point_cloud_distance(pcd_bed)
    dists = np.asarray(dists)

    idx = np.where(dists > 0.07)[0]
    p_inliers = []
    for i in range(len(idx)):
        p_inliers.append(pcd_original.points[idx[i]])

    return p_inliers


def check_location_top_table(pcd_total, pcd_bed, T_bc, T_bo, bed_dims, floor_margin=0.1, visualize=False):
    pcd_total = pcd_total.uniform_down_sample(every_k_points=11)
    if visualize:
        o3d.visualization.draw_geometries([pcd_total])

    # Remove bed
    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(remove_bed(pcd_total, pcd_bed))
    if visualize:
        o3d.visualization.draw_geometries([pcd_top_table])

    # Remove other noise
    cl, ind = pcd_top_table.remove_radius_outlier(nb_points=25, radius=0.15)
    pcd_top_table = cl
    if visualize:
        o3d.visualization.draw_geometries([pcd_top_table])

    # Determine rough location of top_table
    points = np.asarray(pcd_top_table.points)
    points_4d = []
    points_temp = []
    points_transformed = []
    for i in range(len(points)):
        points_4d.append(np.hstack([points[i], [1]]))

    for i in range(len(points_4d)):
        points_temp.append(np.matmul(T_bc, points_4d[i]))

    for i in range(len(points_temp)):
        points_transformed.append(np.matmul(np.linalg.inv(T_bo), points_temp[i]))

    points_transformed_np = np.array(points_transformed)[:,:3]

    # Remove background based on bed_vis coord
    out_x = np.where(np.abs(points_transformed_np[:,0])>bed_dims[0]/2)[0]
    out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1])[0]
    out_z = np.where(points_transformed_np[:,2]<floor_margin)[0]
    out_all = sorted(set(out_x).union(out_y).union(out_z))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = np.array(points_transformed)[in_all, :3]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Determine closet location by checking num of points
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



def process_top_table_detection(color_path, depth_path, T_sc, bed_dims, z_ceiling = 2.3,
                                initial_offset=[0.28,1.1,0.6], floor_margin=0.1, bed_margin=0.1, visualize=False):

    # Load CAD model of top table
    top_table_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/top_table/top_table.STL')
    top_table_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(top_table_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))

    #
    # # Load PCD of top table (obtained from reconstruction)
    # pcd_top_table = o3d.io.read_point_cloud(MILESTONE_DIR + "/pcd.ply")
    # pcd_top_table = pcd_top_table.uniform_down_sample(every_k_points=11)

    # Load PCD of bed
    color = o3d.io.read_image(color_path)
    depth = o3d.io.read_image(depth_path)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / __d_scale,
                                                                    depth_trunc=5.0, convert_rgb_to_intensity=False)
    pcd_top_table = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                             o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                               cam_height, cam_fx,
                                                                                               cam_fy,
                                                                                               cam_ppx, cam_ppy))

    pcd_top_table = pcd_top_table.uniform_down_sample(every_k_points=5)
    # pcd_top_table = remove_background(pcd_top_table, thres=0.04)

    if visualize:
        vis_pointcloud(pcd_top_table)


    # Transform points w.r.t bed_vis coord
    points = np.asarray(pcd_top_table.points)
    points_transformed_np = np.matmul(T_sc[:3,:3], points.transpose()).transpose() + T_sc[:3,3]

    # Remove background based on bed_vis coord
    out_x = np.where(np.abs(points_transformed_np[:,0])>bed_dims[0]/2)[0]
    out_x = np.where(points_transformed_np[:,0]>bed_dims[0]/2)[0]
    out_x2 = np.where(points_transformed_np[:,0]<-bed_dims[0]/2)[0]
    out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1]*1.5)[0]
    # out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1]*1.7)[0]
    in_y = np.where(np.abs(points_transformed_np[:,1])<bed_dims[1]/2+bed_margin)[0]
    out_z = np.where(points_transformed_np[:,2]<floor_margin)[0]
    out_z2 = np.where(points_transformed_np[:,2]>z_ceiling)[0]
    out_all = sorted(set(out_x).union(out_x2).union(out_y).union(in_y).union(out_z).union(out_z2))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = points_transformed_np[in_all, :]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Reconvert points w.r.t camera coord
    T_cs = SE3_inv(T_sc)
    points_recovered = np.matmul(T_cs[:3,:3], points_transformed.transpose()).transpose() + T_cs[:3,3]

    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(points_recovered)

    # Remove other noise
    cl, ind = pcd_top_table.remove_radius_outlier(nb_points=20, radius=0.06)
    pcd_top_table = cl

    # pcd_top_table = pcd_top_table.uniform_down_sample(every_k_points=5)
    # radius_normal = 0.07
    # pcd_top_table_plane = get_plane(pcd_top_table, 0.07, visualize=visualize)
    # pcd_top_table_plane.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # n = np.asarray(pcd_top_table_plane.normals)
    # normal_vec = np.sum(n, axis=0) / np.linalg.norm(np.sum(n, axis=0))

    # Initial guess of ICP
    # q = np.arccos(np.dot(np.array([0,0,1]), normal_vec))
    # R_cc = Rot_axis_series([2, 3], [-np.pi/2, np.pi])
    R_cc = Rot_axis_series([2, 3], [-np.pi/2, np.pi])
    P = np.median(points_recovered, axis=0)
    P += initial_offset
    T_cc = SE3(R_cc, P)
    initial_guess = T_cc


    voxel_size = 0.03
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, top_table_model, pcd_top_table)

    # result_fgr = execute_fast_global_registration(source_down, target_down,
    #                                  source_fpfh, target_fpfh,
    #                                  voxel_size)

    # result_ransac = execute_global_registration(source_down, target_down,
    #                                             source_fpfh, target_fpfh,
    #                                             voxel_size)
    # if visualize:
    #     draw_registration_result(source_down, target_down,
    #                              result_ransac.transformation)
    #
    # initial_guess = np.identity(4)
    # initial_guess[:3,:3] = result_ransac.transformation[:3,:3]
    # initial_guess[:3,3] = result_ransac.transformation[:3,3]

    # q = np.arccos(np.dot(result_ransac.transformation[:3,0], normal_vec))
    # print(q)
    # initial_guess[:3,:3] = np.matmul(result_ransac.transformation[:3,:3], Rot_axis(2, q))

    # if np.abs(np.dot(result_ransac.transformation[:3,2], np.array([0,0,1]))) > 0.5:
    #     initial_guess[:3,:3] = np.matmul(result_ransac.transformation[:3,:3], Rot_axis(2, np.pi/2))
    # if np.dot(result_ransac.transformation[:3,0], np.array([0,0,1])) > 0:
    #     R_tmp = initial_guess[:3,:3]
    #     initial_guess[:3,:3] = np.matmul(Rot_axis(2, np.pi), R_tmp)
    #
    # initial_guess = result_fgr.transformation
    if visualize:
        draw_registration_result(source_down, target_down,
                                     initial_guess)
    # if visualize:
    #     draw_registration_result(source_down, target_down, T_cc)

    ICP_result, fitness = compute_ICP(top_table_model, pcd_top_table, initial_guess,  ratio=0.5, thres=0.05, visualize=visualize)
    return ICP_result


def process_pillow_detection(T_sc, bed_dims, pcd_input, floor_margin=0.1, visualize=False):
    # pillow_dims = (0.57, 0.4, 0.135)
    # # Create cuboid mesh represent pillow shape
    # pillow_model = o3d.geometry.TriangleMesh

    # Load CAD model of pillow
    pillow_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/pillow/pillow.STL')
    pillow_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(pillow_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # # Load PCD of close view of bed for pillow detection
    # color = o3d.io.read_image(SAVE_DIR + '/bed_close.jpg')
    # depth = o3d.io.read_image(SAVE_DIR + '/bed_close.png')
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
    #                                                             depth_trunc = 8.0, convert_rgb_to_intensity = False)
    # pcd_input = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
    #                                                             o3d.camera.PinholeCameraIntrinsic(cam_width,
    #                                                                                               cam_height, cam_fx,
    #                                                                                               cam_fy,
    #                                                                                               cam_ppx, cam_ppy))

    # Remove background based on bed_vis coord
    points = np.asarray(pcd_input.points)
    points_transformed_np = np.matmul(T_sc[:3, :3], points.transpose()).transpose() + T_sc[:3, 3]

    out_x = np.where(np.abs(points_transformed_np[:, 0]) > bed_dims[0] / 2)[0]
    out_x2 = np.where(np.abs(points_transformed_np[:, 0]) < -bed_dims[0] / 2)[0]
    out_y = np.where(np.abs(points_transformed_np[:, 1]) > bed_dims[1] / 2 + bed_dims[1])[0]
    out_z = np.where(points_transformed_np[:, 2] < floor_margin)[0]
    out_all = sorted(set(out_x).union(out_y).union(out_z).union(out_x2))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = points_transformed_np[in_all, :]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Remove bed plane
    pcd_proc = o3d.geometry.PointCloud()
    pcd_proc.points = o3d.utility.Vector3dVector(points_transformed)
    pcd_proc = pcd_proc.uniform_down_sample(every_k_points=9)
    pcd_proc = remove_background(pcd_proc, thres=0.04)

    # Reconvert points w.r.t camera coord
    T_cs = SE3_inv(T_sc)
    points_pillow = np.matmul(T_cs[:3,:3], np.asarray(pcd_proc.points).transpose()).transpose() + T_cs[:3,3]
    pcd_pillow = make_pcd_np(points_pillow)


    voxel_size = 0.03
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, pillow_model, pcd_pillow)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac.transformation)
    if visualize:
        draw_registration_result(source_down, target_down,
                                 result_ransac.transformation)

    ICP_result, fitness = compute_ICP(pillow_model, pcd_pillow, result_ransac.transformation, thres=0.07, visualize=visualize)

    return ICP_result



def reprocess_bed_detection(T_sc, bed_dims, floor_margin, T_toff_bed, visualize=False,
                            relative_fitness=1e-13, relative_rmse=1e-13, max_iteration=600000):
    # Load CAD model of bed
    bed_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/bed/bed.STL')
    bed_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(bed_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))

    # Load PCD of close view of bed for redetection
    # color = o3d.io.read_image(SAVE_DIR + '/bed_close.jpg')
    # depth = o3d.io.read_image(SAVE_DIR + '/bed_close.png')
    # color = o3d.io.read_image(SAVE_DIR + '/full_view.jpg')
    # depth = o3d.io.read_image(SAVE_DIR + '/full_view.png')
    color = o3d.io.read_image(CROP_DIR + '/full_view.jpg')
    depth = o3d.io.read_image(CROP_DIR + '/full_view.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
                                                                depth_trunc = 5.0, convert_rgb_to_intensity = False)
    pcd_input = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))
    # Remove other noise
    cl, ind = pcd_input.remove_radius_outlier(nb_points=20, radius=0.07)
    pcd_input = cl

    # Remove background based on bed_vis coord
    points = np.asarray(pcd_input.points)
    points_transformed_np = np.matmul(T_sc[:3, :3], points.transpose()).transpose() + T_sc[:3, 3]

    out_y = np.where(np.abs(points_transformed_np[:, 1]) > bed_dims[1] / 2 + bed_dims[1])[0]
    out_z = np.where(points_transformed_np[:, 2] < floor_margin)[0]
    out_all = sorted(set(out_y).union(out_z))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = points_transformed_np[in_all, :]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Reconvert points w.r.t camera coord
    T_cs = SE3_inv(T_sc)
    points_recovered = np.matmul(T_cs[:3, :3], points_transformed.transpose()).transpose() + T_cs[:3, 3]
    pcd_bed_full_view = make_pcd_np(points_recovered).uniform_down_sample(every_k_points=3)


    # bed_initial = np.matmul(T_cs, SE3_inv(T_toff_bed))
    bed_initial = np.matmul(T_cs, SE3_inv(T_toff_bed))

    voxel_size = 0.03
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, bed_model,
                                                                                         pcd_bed_full_view)
    if visualize:
        draw_registration_result(source_down, target_down, bed_initial)

    ICP_result, fitness = compute_ICP(bed_model, pcd_bed_full_view, bed_initial, ratio=0.7, thres=0.14, visualize=visualize)
    # ICP_result= compute_close_ICP(bed_model, pcd_bed_close, bed_initial, thres=0.08, visualize=visualize,
    #                               relative_fitness=relative_fitness, relative_rmse=relative_rmse,
    #                               max_iteration=max_iteration)

    return ICP_result


def reprocess_top_table_detection(T_sc, T_cs_closet, bed_dims, T_toff_closet,
                                initial_offset=[0.3,1.1,0.6], floor_margin=0.1, visualize=False, bed_height=1.5):

    # Load CAD model of top table
    top_table_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/top_table/top_table.STL')
    top_table_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(top_table_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # Load PCD of closet
    color = o3d.io.read_image(SAVE_DIR + '/top_table_close.jpg')
    depth = o3d.io.read_image(SAVE_DIR + '/top_table_close.png')
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / __d_scale,
                                                                    depth_trunc=5.0, convert_rgb_to_intensity=False)
    pcd_input = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                             o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                               cam_height, cam_fx,
                                                                                               cam_fy,
                                                                                               cam_ppx, cam_ppy))

    if visualize:
        o3d.visualization.draw_geometries([pcd_input])

    # Transform points w.r.t bed_vis coord
    points = np.asarray(pcd_input.points)
    points_transformed_np = np.matmul(T_sc[:3,:3], points.transpose()).transpose() + T_sc[:3,3]

    # Remove background based on bed_vis coord
    out_x = np.where(np.abs(points_transformed_np[:,0])>bed_dims[0]/2)[0]
    # out_x2 = np.where(np.abs(points_transformed_np[:,0])<-bed_dims[0]/2)[0]
    out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1])[0]
    in_y = np.where(np.abs(points_transformed_np[:,1])<bed_dims[1]/2)[0]
    # over_bed = np.where(np.abs(points_transformed_np[:,2])>bed_height)[0]
    # in_y = set(in_y) - set(over_bed)
    out_z = np.where(points_transformed_np[:,2]<floor_margin)[0]
    out_all = sorted(set(out_x).union(out_y).union(out_z).union(in_y))
    # out_all = sorted(set(out_y).union(out_z).union(in_y))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = points_transformed_np[in_all, :]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Reconvert points w.r.t camera coord
    T_cs = SE3_inv(T_sc)
    points_recovered = np.matmul(T_cs[:3,:3], points_transformed.transpose()).transpose() + T_cs[:3,3]

    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(points_recovered)

    # Initial guess of ICP
    R_cc = Rot_axis_series([2, 3], [-np.pi/2, np.pi])
    P = np.median(points_recovered, axis=0)
    P += initial_offset
    T_cc = SE3(R_cc, P)

    closet_initial = np.matmul(T_cs_closet, SE3_inv(T_toff_closet))


    voxel_size = 0.03
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, top_table_model, pcd_top_table)
    if visualize:
        draw_registration_result(source_down, target_down, closet_initial)

    ICP_result = compute_close_ICP(top_table_model, pcd_top_table, closet_initial, thres=0.05, visualize=visualize)

    return ICP_result, pcd_top_table



def reprocess_top_table_detection_scan(T_sc, T_cs_closet, bed_dims, T_toff_closet,
                                initial_offset=[0.3,1.1,0.6], floor_margin=0.1, visualize=False):

    # Load CAD model of top table
    top_table_model = o3d.io.read_triangle_mesh(MODEL_DIR + '/top_table/top_table.STL')
    top_table_model.vertices = o3d.utility.Vector3dVector(
        np.asarray(top_table_model.vertices) * np.array([1 / 1000.0, 1 / 1000.0, 1 / 1000.0]))


    # Load PCD of top table (obtained from reconstruction)
    pcd_input = o3d.io.read_point_cloud(MILESTONE_DIR + "/pcd.ply")
    pcd_input = pcd_input.uniform_down_sample(every_k_points=11)


    # # Load PCD of closet
    # color = o3d.io.read_image(SAVE_DIR + '/top_table_close.jpg')
    # depth = o3d.io.read_image(SAVE_DIR + '/top_table_close.png')
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / __d_scale,
    #                                                                 depth_trunc=5.0, convert_rgb_to_intensity=False)
    # pcd_input = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
    #                                                          o3d.camera.PinholeCameraIntrinsic(cam_width,
    #                                                                                            cam_height, cam_fx,
    #                                                                                            cam_fy,
    #                                                                                            cam_ppx, cam_ppy))

    if visualize:
        o3d.visualization.draw_geometries([pcd_input])

    # Transform points w.r.t bed_vis coord
    points = np.asarray(pcd_input.points)
    points_transformed_np = np.matmul(T_sc[:3,:3], points.transpose()).transpose() + T_sc[:3,3]

    # Remove background based on bed_vis coord
    out_x = np.where(np.abs(points_transformed_np[:,0])>bed_dims[0]/2)[0]
    out_x2 = np.where(np.abs(points_transformed_np[:,0])<-bed_dims[0]/2)[0]
    out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1])[0]
    in_y = np.where(np.abs(points_transformed_np[:,1])<bed_dims[1]/2+0.3)[0]
    out_z = np.where(points_transformed_np[:,2]<floor_margin)[0]
    out_all = sorted(set(out_x).union(out_y).union(out_z).union(in_y))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = points_transformed_np[in_all, :]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Reconvert points w.r.t camera coord
    T_cs = SE3_inv(T_sc)
    points_recovered = np.matmul(T_cs[:3,:3], points_transformed.transpose()).transpose() + T_cs[:3,3]

    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(points_recovered)

    # Initial guess of ICP
    R_cc = Rot_axis_series([2, 3], [-np.pi/2, np.pi])
    P = np.median(points_recovered, axis=0)
    P += initial_offset
    T_cc = SE3(R_cc, P)

    closet_initial = np.matmul(T_cs_closet, SE3_inv(T_toff_closet))


    voxel_size = 0.03
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, top_table_model, pcd_top_table)
    if visualize:
        draw_registration_result(source_down, target_down, closet_initial)

    ICP_result = compute_close_ICP(top_table_model, pcd_top_table, closet_initial, thres=0.05, visualize=visualize)

    return ICP_result



def vis_pointcloud(pcd):
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2,origin=(0,0,0))
    o3d.visualization.draw_geometries([pcd, origin])


def vis_pointcloud_np(points):
    vis = o3d.geometry.PointCloud()
    vis.points = o3d.utility.Vector3dVector(points)
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=(0, 0, 0))
    o3d.visualization.draw_geometries([vis, origin])


def make_pcd_from_rgbd(color_instance, depth_instance):
    color = o3d.geometry.Image(color_instance)
    depth = o3d.geometry.Image(depth_instance)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/__d_scale,
                                                                depth_trunc = 8.0, convert_rgb_to_intensity = False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy))
    return pcd


def make_pcd_np(pcd_points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_points)
    return pcd


def skew_symmetric_mat(axis):
    ax = axis[0]
    ay = axis[1]
    az = axis[2]
    A = np.asarray([[0, -az, ay],
                    [az, 0, -ax],
                    [-ay, ax, 0]])
    return A


def Rodrigues_Rot_mat(axis, q):
    A = skew_symmetric_mat(axis)
    R = np.identity(3) + sin(q) * A + (1 - cos(q)) * np.matmul(A, A)

    return R


import os
import sys

sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/milestone_202110'))
from pkg.utils.rotation_utils import *
from collections import namedtuple
import open3d as o3d

CamIntrins = namedtuple('CamIntrins', ['cam_width', 'cam_height', 'cam_fx', 'cam_fy', 'cam_ppx', 'cam_ppy'])

##
# @class ColorDepthMap
# @param color numpy 8 bit array
# @param depth numpy 16 bit array
# @param intrins CamIntrins
# @param depth_scale multiplier for depthymap
ColorDepthMap = namedtuple('ColorDepthMap', ['color', 'depth', 'intrins', 'depth_scale'])


##
# @# param fname file apth except extension
def load_rdict(obj_type,
               intrins=[1280, 720,
                        909.957763671875, 909.90283203125,
                        638.3824462890625, 380.0085144042969],
               depth_scale=1 / 3999.999810010204):
    rdict = {}
    rdict['color'] = cv2.imread(
        os.path.join(SAVE_DIR, obj_type + '.jpg'), flags=cv2.IMREAD_UNCHANGED)
    rdict['depth'] = cv2.imread(
        os.path.join(SAVE_DIR, obj_type + '.png'), flags=cv2.IMREAD_UNCHANGED)
    rdict['intrins'], rdict['depth_scale'] = intrins, depth_scale
    Q = np.loadtxt(os.path.join(SAVE_DIR, obj_type + '.csv'), delimiter=",")
    return rdict, np.array(Q)


def rdict2cdp(rdict):
    return ColorDepthMap(**{k: rdict[k] for k in ColorDepthMap._fields})


def apply_mask(cdp, mask):
    mask_u8 = np.zeros_like(mask).astype(np.uint8)
    mask_u8[np.where(mask)] = 255
    color_masked = cv2.bitwise_and(
        cdp.color, cdp.color, mask=mask_u8
    ).astype(np.uint8)
    depth_masked = cv2.bitwise_and(
        cdp.depth, cdp.depth, mask=mask_u8
    ).astype(np.uint16)
    return ColorDepthMap(color_masked, depth_masked, cdp.intrins, cdp.depth_scale)

def cdp2pcd(cdp, Tc=None, depth_trunc=5.0):
    if Tc is None:
        Tc = np.identity(4)
    color = o3d.geometry.Image(cdp.color)
    depth = o3d.geometry.Image(cdp.depth)
    d_scale = cdp.depth_scale
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / d_scale,
                                                                    depth_trunc=depth_trunc,
                                                                    convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
                                                         o3d.camera.PinholeCameraIntrinsic(
                                                             *cdp.intrins), SE3_inv(Tc))
    return pcd

class MultiICP:

    ##
    # @param model open3d.geometry.TriangleMesh or file path
    # @param Toff  GeometryItem coordinate in TriangleMesh coordinate. externally, we use geometry coordinate for all TFs
    # @param scale scale multiplier if model is given as file path
    def __init__(self, model, Toff, scale=1e-3):
        if isinstance(model, o3d.geometry.TriangleMesh):
            self.model = model
        elif isinstance(model, str):
            self.model = o3d.io.read_triangle_mesh(model)
            self.model.vertices = o3d.utility.Vector3dVector(
                np.asarray(self.model.vertices) * np.array([scale] * 3))
        else:
            raise (NotImplementedError("non available input for model : \n".format(model)))
        self.set_Toff(Toff)
        self.depth_trunc = 5.0
        self.pcd = None
        self.pcd_Tc_stack = []

    ##
    # @param Toff  GeometryItem coordinate in TriangleMesh coordinate. externally, we use geometry coordinate for all TFs
    def set_Toff(self, Toff):
        self.Toff = Toff
        self.Toff_inv = SE3_inv(Toff)

    ##
    # @param cdp open3d.geometry.PointCloud
    # @param Tc camera transformation matrix
    def add_pointcloud(self, pcd, Tc=None):
        if Tc is None:
            pcd_cam = copy.deepcopy(pcd)
            pcd = copy.deepcopy(pcd)
            Tc= np.identity(4)
        else:
            pcd_cam = copy.deepcopy(pcd)
            pcd = copy.deepcopy(pcd)

            points = np.asarray(pcd_cam.points)
            points4d = np.pad(points, ((0, 0), (0, 1)), 'constant', constant_values=1)
            Tc_inv = SE3_inv(Tc)
            points_c = np.matmul(points4d, Tc_inv.transpose())[:, :3]
            pcd_cam.points = o3d.utility.Vector3dVector(points_c)

        self.pcd_Tc_stack.append((pcd_cam, Tc))
        if self.pcd is None:
            self.pcd = pcd
        else:
            pass  # add
            self.pcd += pcd
            self.pcd = self.pcd.uniform_down_sample(every_k_points=10)
        self.model.compute_vertex_normals()
        # self.model_sampled = self.model.sample_points_uniformly(number_of_points=int(len(np.array(self.pcd.points)*0.6)))
        self.model_sampled = self.model.sample_points_poisson_disk(
                                                    number_of_points=int(len(np.array(self.pcd.points) * 0.4)))
        return self.pcd

    ##
    # @param cdp ColorDepthMap
    # @param Tc camera transformation matrix
    def add_image(self, cdp, Tc=None):
        if Tc is None:
            Tc= np.identity(4)
        pcd_cam = cdp2pcd(cdp, depth_trunc=self.depth_trunc)
        pcd = cdp2pcd(cdp, Tc=Tc, depth_trunc=self.depth_trunc)
        self.pcd_Tc_stack.append((pcd_cam, Tc))
        if self.pcd is None:
            self.pcd = pcd
        else:
            pass  # add
            self.pcd += pcd
            self.pcd = self.pcd.uniform_down_sample(every_k_points=10)
        self.model.compute_vertex_normals()
        # self.model_sampled = self.model.sample_points_uniformly(number_of_points=int(len(np.array(self.pcd.points)*0.6)))
        self.model_sampled = self.model.sample_points_poisson_disk(
                                                    number_of_points=int(len(np.array(self.pcd.points) * 0.4)))
        return self.pcd

    ##
    # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
    # @param thres max distance between corresponding points
    def compute_ICP(self, To=None, thres=0.1,
                    relative_fitness=1e-15, relative_rmse=1e-15, max_iteration=500000,
                    voxel_size=0.04, visualize=False
                    ):
        if To is None:
            To, fitness = self.auto_init(0, voxel_size)
        target = copy.deepcopy(self.pcd)
        source = copy.deepcopy(self.model_sampled)

        # To Be Done - add front only option and cut backward surface here based on To
        if visualize:
            self.draw(To)

        To = np.matmul(To, self.Toff_inv)

        # Guess Initial Transformation
        trans_init = To

        print("Apply point-to-point ICP")
        threshold = thres
        reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                    o3d.registration.TransformationEstimationPointToPoint(),
                                                    o3d.registration.ICPConvergenceCriteria(
                                                        relative_fitness=relative_fitness,
                                                        relative_rmse=relative_rmse,
                                                        max_iteration=max_iteration))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        ICP_result = reg_p2p.transformation

        ICP_result = np.matmul(ICP_result, self.Toff)
        if visualize:
            self.draw(ICP_result)

        return ICP_result, reg_p2p.fitness


        ##
        # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
        # @param thres max distance between corresponding points
    def compute_front_ICP(self, model_type, T_bc=None, To=None, thres=0.1,
                    relative_fitness=1e-15, relative_rmse=1e-15, max_iteration=500000,
                    voxel_size=0.04, visualize=False
                    ):
        if To is None:
            To, fitness = self.auto_init(0, voxel_size)

        if T_bc is None:
            T_bc = SE3(np.identity(3), (0, 0, 0))

        target = copy.deepcopy(self.pcd)

        T_cb = SE3_inv(T_bc)
        T_co = np.matmul(np.matmul(T_cb, To), self.Toff_inv)
        # model_mesh = self.model.compute_vertex_normals()
        model_pcd = self.model_sampled

        normals = np.asarray(model_pcd.normals)
        points = np.asarray(model_pcd.points)
        # point_normals = normals
        # view_vec = SE3_inv(Tguess)[:3,2]
        point_normals = np.matmul(T_co[:3, :3], normals.T).T
        view_vec = (0, 0, 1)
        idx = []
        for i in range(len(point_normals)):
            if np.dot(view_vec, point_normals[i]) < 0:
                idx.append(i)

        pts = np.zeros((len(idx), 3))
        for i in range(len(idx)):
            pts[i] = points[idx[i]]

        front_pcd = o3d.geometry.PointCloud()
        front_pcd.points = o3d.utility.Vector3dVector(pts)
        if visualize:
            vis_pointcloud(front_pcd)
        source = copy.deepcopy(front_pcd)

        # To Be Done - add front only option and cut backward surface here based on To
        if model_type=="bed":
            To = np.matmul(T_cb, To)

        if visualize:
            self.draw(To, source, target)

        To = np.matmul(To, self.Toff_inv)

        # Guess Initial Transformation
        trans_init = To

        print("Apply point-to-point ICP")
        threshold = thres
        reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                    o3d.registration.TransformationEstimationPointToPoint(),
                                                    o3d.registration.ICPConvergenceCriteria(
                                                        relative_fitness=relative_fitness,
                                                        relative_rmse=relative_rmse,
                                                        max_iteration=max_iteration))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        ICP_result = reg_p2p.transformation

        ICP_result = np.matmul(ICP_result, self.Toff)
        if visualize:
            self.draw(ICP_result, source, target)

        return ICP_result, reg_p2p.fitness


        ##
        # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
        # @param thres max distance between corresponding points
    def compute_front_cut_ICP(self, model_type, To=None, thres=0.1,
                    relative_fitness=1e-15, relative_rmse=1e-15, max_iteration=500000,
                    voxel_size=0.04, visualize=False
                    ):
        if To is None:
            To, fitness = self.auto_init(0, voxel_size)
        target = copy.deepcopy(self.pcd)

        idx = []
        if model_type == "bed":
            max_dist = self.model_sampled.get_center()[2] * 1.6
            front_model = np.asarray(self.model_sampled.points)
            for i in range(len(front_model)):
                if front_model[i, 2] > max_dist:
                    idx.append(i)
        elif model_type == "closet":
            max_dist = self.model_sampled.get_center()[0] * 1.4
            front_model = np.asarray(self.model_sampled.points)
            for i in range(len(front_model)):
                if front_model[i, 0] > max_dist:
                    idx.append(i)

        pts = np.zeros((len(idx), 3))
        for i in range(len(idx)):
            pts[i] = front_model[idx[i]]

        front_pcd = o3d.geometry.PointCloud()
        front_pcd.points = o3d.utility.Vector3dVector(pts)
        source = copy.deepcopy(front_pcd)

        # To Be Done - add front only option and cut backward surface here based on To
        if visualize:
            self.draw(To, source, target)

        To = np.matmul(To, self.Toff_inv)

        # Guess Initial Transformation
        trans_init = To

        print("Apply point-to-point ICP")
        threshold = thres
        reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                    o3d.registration.TransformationEstimationPointToPoint(),
                                                    o3d.registration.ICPConvergenceCriteria(
                                                        relative_fitness=relative_fitness,
                                                        relative_rmse=relative_rmse,
                                                        max_iteration=max_iteration))
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        ICP_result = reg_p2p.transformation

        ICP_result = np.matmul(ICP_result, self.Toff)
        if visualize:
            self.draw(ICP_result, source, target)

        return ICP_result, reg_p2p.fitness


    def draw(self, To, source=None, target=None):
        if source is None: source = self.model_sampled
        if target is None: target = self.pcd
        To = np.matmul(To, self.Toff_inv)
        draw_registration_result(source, target, To)

    def auto_init(self, init_idx=0, voxel_size=0.04):
        pcd_cam, Tc = self.pcd_Tc_stack[init_idx]
        Tc_inv = SE3_inv(Tc)
        source_down, source_fpfh = preprocess_point_cloud(pcd_cam, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(self.model_sampled, voxel_size)

        distance_threshold = voxel_size * 1.4
        result = o3d.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(False), 3, [
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))

        To = matmul_series(Tc, result.transformation, self.Toff)
        return To, result.fitness

    ##
    # @param R orientation guess for geometry coord
    # @param offset offset to add to the pcd center, in icp origin coordinate
    def get_initial_by_center(self, R, offset):
        # Get distance of pcd
        center_p = self.pcd.get_center()
        return SE3(R, center_p + offset)

    ##
    # @param R orientation guess for geometry coord
    # @param offset offset to add to the pcd center, in icp origin coordinate
    def get_initial_by_median(self, R, offset):
        # Get distance of pcd
        center_p = np.median(self.pcd.points, axis=0)
        return SE3(R, center_p + offset)

    def clear(self):
        self.pcd = None
        self.pcd_Tc_stack = []

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    FOR_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])

    FOR_model = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    FOR_model.transform(transformation)
    FOR_model.translate(source_temp.get_center() - FOR_model.get_center())

    FOR_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=target.get_center())
    o3d.visualization.draw_geometries([source_temp, target, FOR_origin, FOR_model, FOR_target])

##
# @param pcd        o3d.geometry.PointCloud
# @param inside     if True, return points inside. if False, return points outside
# @param merge_rule np.any or np.all
def mask_boxes(pcd, boxes, Q, inside, merge_rule=np.all, link_ref="base_link"):
    pcd = copy.deepcopy(pcd)
    points = np.asarray(pcd.points)
    points4d = np.pad(points, ((0,0), (0,1)), 'constant', constant_values=1)
    mask_list = []
    for box in boxes:
        T_bx = box.get_tf(Q, from_link=link_ref)
        T_xb = SE3_inv(T_bx)
        abs_cuts = np.divide(box.dims, 2)
        points_x = np.matmul(points4d, T_xb.transpose())[:,:3]
        if inside:
            mask = np.all(np.abs(points_x) < abs_cuts, axis=-1)
        else:
            mask = np.any(np.abs(points_x) > abs_cuts, axis=-1)
        mask_list.append(mask)
    idc = np.where(merge_rule(mask_list, axis=0))[0]
    pcd.points = o3d.utility.Vector3dVector(points[idc])
    return pcd
