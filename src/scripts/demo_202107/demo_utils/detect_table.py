import open3d as o3d
import numpy as np
import cv2
import copy
import os
import sys
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from demo_config import *
from pkg.utils.rotation_utils import *
import SharedArray as sa
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt

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

CONFIG_DIR = os.path.join(DEMO_DIR, "configs")
SAVE_DIR = os.path.join(DEMO_DIR, "save_img")
CROP_DIR = os.path.join(DEMO_DIR, "crop_img")
MODEL_DIR = os.path.join(DEMO_DIR, "model_CAD")

DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (720, 1280)

# import subprocess
# subprocess.call(['python3', 'detection.py'])
cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy = [None]*6
__d_scale = None

def set_cam_params(cam_intrins_, d_scale):
    global cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy, __d_scale
    cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy = cam_intrins_
    __d_scale = d_scale

def preprocessing():
    # Load CAD model of table leg
    model_mesh = o3d.io.read_triangle_mesh(MODEL_DIR + '/table_leg_scaling.STL')
    # model_pcd = model_mesh.sample_points_uniformly(number_of_points=300)

    # Load Depth image to make point clouds
    depth = o3d.io.read_image(CROP_DIR + '/depth_crop.png')
    depth_pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy),
                                                                depth_scale=1 / __d_scale)

    # Remove noise points which put very far from camera
    thres = np.linalg.norm(depth_pcd.get_center())
    depth_pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy),
                                                                depth_scale=1 / __d_scale, depth_trunc=thres * 1.5)

    o3d.visualization.draw_geometries([depth_pcd])

    # Convert point clouds to numpy array
    xyz_points = np.array(depth_pcd.points)

    # Kmeans Clustering to classify front, back legs of table
    # Ideally, if noise does not exist, then number of cluster is 2
    kmeans = KMeans(n_clusters=2, random_state=0)
    kmeans.fit(xyz_points)

    # Re-convert numpy array to point clouds in type of o3d point clouds
    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()
    xyz_1 = np.vstack(
        [xyz_points[kmeans.labels_ == 0, 0], xyz_points[kmeans.labels_ == 0, 1], xyz_points[kmeans.labels_ == 0, 2]])
    xyz_2 = np.vstack(
        [xyz_points[kmeans.labels_ == 1, 0], xyz_points[kmeans.labels_ == 1, 1], xyz_points[kmeans.labels_ == 1, 2]])
    pcd1.points = o3d.utility.Vector3dVector(xyz_1.T)
    pcd2.points = o3d.utility.Vector3dVector(xyz_2.T)
    # FOR_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0,0,0])

    # o3d.visualization.draw_geometries([pcd1, FOR_origin])
    # o3d.visualization.draw_geometries([pcd2, FOR_origin])

    # Divide front and back legs of table
    dist_1 = np.linalg.norm(pcd1.get_center())
    dist_2 = np.linalg.norm(pcd2.get_center())

    if (abs(dist_1 - dist_2) < 0.8):
        # Original depth point clouds include front legs only
        # Not Do clustering
        pcd_out = depth_pcd

    else:
        # Original depth point clouds include front and back legs together, so clustering is reasonable
        # Do clustering
        if (dist_1 > dist_2):
            # The clustering result would be random for order of points
            # So, shortest distance point clouds set front legs
            pcd_out = pcd1
    #             tmp = pcd2
    #             pcd2 = pcd1
    #             pcd1 = tmp

    # o3d.visualization.draw_geometries([pcd1, FOR_origin])
    return model_mesh, pcd_out


def ransac_plane_fitting(img_path):
    # Find plane through ransac plane fitting
    # depth_raw = o3d.io.read_image("/home/jhkim/Projects/rnb-planning/src/scripts/demo_202107/save_img/depth.png")
    depth_raw = o3d.io.read_image(img_path)
    depth_pcd_raw = o3d.geometry.PointCloud.create_from_depth_image(depth_raw,
                                                                    o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                      cam_height,
                                                                                                      cam_fx, cam_fy,
                                                                                                      cam_ppx, cam_ppy),
                                                                    depth_scale=1 / __d_scale)
    # depth_pcd_raw = o3d.geometry.PointCloud.create_from_depth_image(depth_raw, o3d.camera.PinholeCameraIntrinsic(640, 480,
    #                                                                                                  461.734375, 462.06640625,
    #                                                                                                  350.4140625, 244.541015625), depth_scale = 4000.0)

    plane_model, inliers = depth_pcd_raw.segment_plane(distance_threshold=0.01,
                                                       ransac_n=8,
                                                       num_iterations=2000)
    [a, b, c, d] = plane_model
    print("Coeffs of eq of fitting plane are :")
    print(a, b, c, d)
    return plane_model
    # print(f"Plane equation: {a:.5f}x + {b:.5f}y + {c:.5f}z + {d:.5f} = 0")


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    FOR_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])

    FOR_model = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    FOR_model.transform(transformation)
    FOR_model.translate(source_temp.get_center() - FOR_model.get_center())

    FOR_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=target.get_center())
    o3d.visualization.draw_geometries([source_temp, target, FOR_origin])


def compute_ICP(model_mesh, pcd, model_center_offset):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points) * 0.8)))
    source = copy.deepcopy(model_pcd)
    # source.translate((-T_Height/2, -T_Depth/2, 0.0), relative=True)
    source_cpy = copy.deepcopy(model_pcd)
    source_cpy.translate(model_center_offset, relative=True)

    # Guess Initial Transformation
    center = target.get_center()
    trans_init = np.identity(4)
    trans_init[0:3, 3] = center.T
    source_cpy.transform(trans_init)
    trans_init[0:3, 3] = source_cpy.get_center().T
    draw_registration_result_original_color(source, target, trans_init)

    print("Apply point-to-point ICP")
    threshold = 0.50
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(max_iteration=600000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result_original_color(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    source.transform(ICP_result)
    # o3d.visualization.draw_geometries([source, target])
    return ICP_result


def get_inliers(img_path):
    depth_raw = o3d.io.read_image(img_path)
    depth_pcd_raw = o3d.geometry.PointCloud.create_from_depth_image(depth_raw,
                                                                    o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                      cam_height,
                                                                                                      cam_fx, cam_fy,
                                                                                                      cam_ppx, cam_ppy),
                                                                    depth_scale=1 / __d_scale, depth_trunc=0.8)

    plane_model, inliers = depth_pcd_raw.segment_plane(distance_threshold=0.004,
                                                       ransac_n=7,
                                                       num_iterations=1500)
    [a, b, c, d] = plane_model
    print("Coeffs of eq of fitting plane are :")
    print(a, b, c, d)

    # Inlier points from ransac plane fitting
    p_inliers = []
    for i in range(len(inliers)):
        p_inliers.append(depth_pcd_raw.points[i])

    pcd_inliers = o3d.geometry.PointCloud()
    pcd_inliers.points = o3d.utility.Vector3dVector(p_inliers)
    o3d.visualization.draw_geometries([depth_pcd_raw])
    o3d.visualization.draw_geometries([pcd_inliers])
    return p_inliers


def point_proj(T_bc, p_inliers):
    x_bo = []
    y_bo = []
    for i in range(len(p_inliers)):
        vec = np.hstack([p_inliers[i], [1]]).T
        xyz_point = np.matmul(T_bc, vec)
        # points_bo.append(xyz_point[0:2])
        x_bo.append(xyz_point[0])
        y_bo.append(xyz_point[1])
    plt.plot(x_bo, y_bo)
    return x_bo, y_bo




def left_corner(x_bo, y_bo):
    # for left corner
    TABLE_DIMS = np.array((0.785, 1.80, 0.735))
    TABLE_DIMS[[0, 1, 2]]
    OFF_DIR = np.array([1, -1, 1])
    idx_edge = np.argmax(np.subtract(y_bo, x_bo))
    idx_x_min = np.argmin(x_bo)
    idx_x_max = np.argmax(x_bo)
    idx_y_min = np.argmin(y_bo)
    idx_y_max = np.argmax(y_bo)

    # First, find edge in view of case 1
    # case 1
    edge_left = np.array((x_bo[idx_edge], y_bo[idx_edge], -0.439))
    p = np.array((x_bo[idx_x_min], y_bo[idx_x_min], -0.439))
    case = 1
    num = edge_left[0] - p[0]
    den = edge_left[1] - p[1]
    theta = np.arctan2(num, den)
    print(y_bo[idx_y_max])
    print(edge_left)
    print(p)

    if (np.linalg.norm(edge_left - p) < 0.05):
        # actually, it is case 2
        edge_left = np.array((x_bo[idx_x_min], y_bo[idx_x_min], -0.439))
        p = np.array((x_bo[idx_y_max], y_bo[idx_y_max], -0.439))
        case = 2
        num = edge_left[1] - p[1]
        den = edge_left[0] - p[0]
        theta = np.arctan2(num, den)
        print(y_bo[idx_y_max])
        print(edge_left)
        print(p)

    T_bo = np.identity(4)
    # orientation of table
    if (case == 1):
        T_bo[:3, :3] = Rot_axis(3, deg2rad(-theta))
    elif (case == 2):
        T_bo[:3, :3] = Rot_axis(3, deg2rad(theta))
    T_bo[:3, 3] = (edge_left.T + np.divide(TABLE_DIMS[[0, 1, 2]] * OFF_DIR, 2).T)
    return T_bo


def right_corner(x_bo, y_bo):
    # for right corner
    TABLE_DIMS = np.array((0.785, 1.80, 0.735))
    TABLE_DIMS[[0, 1, 2]]
    OFF_DIR = np.array([1, 1, 1])
    idx_edge = np.argmax(np.subtract(np.negative(y_bo), x_bo))
    idx_x_min = np.argmin(x_bo)
    idx_x_max = np.argmax(x_bo)
    idx_y_min = np.argmin(y_bo)
    idx_y_max = np.argmax(y_bo)

    # First, find edge in view of case 1
    # case 1
    edge_right = np.array((x_bo[idx_edge], y_bo[idx_edge], -0.439))
    p = np.array((x_bo[idx_y_max], y_bo[idx_y_max], -0.439))
    case = 1
    num = p[0] - edge_right[0]
    den = p[1] - edge_right[1]
    theta = np.arctan(num / den)
    print(x_bo[idx_x_min])
    print(edge_right[0])

    if (np.linalg.norm(edge_right - p) < 0.05):
        # actually, it is case 2
        # case 2
        edge_left = np.array((x_bo[idx_y_min], y_bo[idx_y_min], -0.439))
        p = np.array((x_bo[idx_x_max], y_bo[idx_x_max], -0.439))
        case = 2
        num = p[1] - edge_right[1]
        den = p[0] - edge_right[0]
        theta = np.arctan(num / den)
    print(x_bo[idx_x_min])
    print(edge_right[0])

    T_bo = np.identity(4)
    T_bo[:3, 3] = (edge_right.T + np.divide(TABLE_DIMS[[0, 1, 2]] * OFF_DIR, 2).T)
    # orientation of table
    if (case == 1):
        T_bo[:3, :3] = Rot_axis(3, deg2rad(-theta))
    elif (case == 2):
        T_bo[:3, :3] = Rot_axis(3, deg2rad(theta))
    return T_bo