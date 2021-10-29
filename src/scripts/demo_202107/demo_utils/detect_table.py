import open3d as o3d
import numpy as np
import cv2
import copy
import os
import sys
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from demo_config import *
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from streaming import *
import SharedArray as sa
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import random
from pkg.geometry.geotype import GEOTYPE
from demo_utils.environment import Corners

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
DATASET_CAM_DIR = os.path.join(DEMO_DIR, "exp_datasets")
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
    # FOR = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0,0,0])
    # FOR.translate(depth_pcd.get_center())
    # o3d.visualization.draw_geometries([depth_pcd, FOR])
    thres = np.linalg.norm(depth_pcd.get_center())
    print(thres)
    if (thres > 3.1) :
        depth_thres = thres * 1.95
    elif (thres > 2.7) :
        depth_thres = thres * 1.73
    elif (thres < 2.7 and thres > 2.5) :
        depth_thres = thres * 1.47
    elif (thres < 2.5 and thres > 2.3) :
        depth_thres = thres * 1.34
    elif (thres < 2.3 and thres > 2.08) :
        depth_thres = thres * 1.11
    elif (thres < 2.08 and thres > 1.9) :
        depth_thres = thres * 1.07
    elif (thres < 1.9 and thres > 1.78) :
        depth_thres = thres * 1.02
    else :
        depth_thres = thres * 0.97
    depth_pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,
                                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                  cam_height, cam_fx,
                                                                                                  cam_fy,
                                                                                                  cam_ppx, cam_ppy),
                                                                depth_scale=1 / __d_scale, depth_trunc=depth_thres)

    # o3d.visualization.draw_geometries([depth_pcd])

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

    pcd_out = pcd1
    if (abs(dist_1 - dist_2) < 0.9):
        # Original depth point clouds include front legs only
        # Not Do clustering
        pcd_out = depth_pcd
    else:
        # Original depth point clouds include front and back legs together, so clustering is reasonable
        # Do clustering
        if (dist_1 > dist_2):
            # The clustering result would be random for order of points
            # So, shortest distance point clouds set front legs
            pcd_out = pcd2
    #             tmp = pcd2
    #             pcd2 = pcd1
    #             pcd1 = tmp

    # o3d.visualization.draw_geometries([pcd_out])
    # Do ransac fitting
    plane_model, inliers = pcd_out.segment_plane(distance_threshold=0.04,
                                                       ransac_n=7,
                                                       num_iterations=1500)

    p_inliers = []
    for i in range(len(inliers)):
        p_inliers.append(pcd_out.points[inliers[i]])

    pcd_inliers = o3d.geometry.PointCloud()
    pcd_inliers.points = o3d.utility.Vector3dVector(p_inliers)
    # o3d.visualization.draw_geometries([pcd_inliers])

    # new_pcd_out = o3d.geometry.PointCloud()
    # new_p_inliers = []
    # for i in range(len(pcd_out.points)):
    #     if i not in inliers :
    #         new_p_inliers.append(pcd_out.points[i])
    #
    # new_pcd_out.points = o3d.utility.Vector3dVector(new_p_inliers)
    # o3d.visualization.draw_geometries([new_pcd_out])
    return model_mesh, pcd_inliers


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    FOR_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])

    FOR_model = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
    FOR_model.transform(transformation)
    FOR_model.translate(source_temp.get_center() - FOR_model.get_center())

    FOR_target = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=target.get_center())
    o3d.visualization.draw_geometries([source_temp, target, FOR_origin, FOR_model, FOR_target])


def compute_ICP(model_mesh, pcd, model_center_offset):
    # Compute ICP to align model(source) to obtained point clouds(target)
    target = copy.deepcopy(pcd)
    model_pcd = model_mesh.sample_points_uniformly(number_of_points=int(len(np.array(target.points) * 0.7)))
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
    # draw_registration_result_original_color(source, target, trans_init)

    print("Apply point-to-point ICP")
    threshold = 0.15
    reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
                                                o3d.registration.TransformationEstimationPointToPoint(),
                                                o3d.registration.ICPConvergenceCriteria(max_iteration=600000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    # draw_registration_result_original_color(source, target, reg_p2p.transformation)
    ICP_result = reg_p2p.transformation

    source.transform(ICP_result)
    return ICP_result


def get_inliers(img_path):
    gaze_dist = 0.5
    depth_raw = o3d.io.read_image(img_path)
    depth_pcd_raw = o3d.geometry.PointCloud.create_from_depth_image(depth_raw,
                                                                    o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                                                      cam_height,
                                                                                                      cam_fx, cam_fy,
                                                                                                      cam_ppx, cam_ppy),
                                                                    depth_scale=1 / __d_scale, depth_trunc= gaze_dist * 1.5)

    depth_pcd_raw = depth_pcd_raw.uniform_down_sample(every_k_points=3)
    plane_model, inliers = depth_pcd_raw.segment_plane(distance_threshold=0.003,
                                                       ransac_n=7,
                                                       num_iterations=1400)
    [a, b, c, d] = plane_model
    print("Coeffs of eq of fitting plane are :")
    print(a, b, c, d)

    # Inlier points from ransac plane fitting
    p_inliers = []
    for i in range(len(inliers)):
        p_inliers.append(depth_pcd_raw.points[inliers[i]])

    pcd_inliers = o3d.geometry.PointCloud()
    pcd_inliers.points = o3d.utility.Vector3dVector(p_inliers)
    #o3d.visualization.draw_geometries([depth_pcd_raw])
    #o3d.visualization.draw_geometries([pcd_inliers])
    print(len(p_inliers))
    print(p_inliers[0])
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
    x_bo = np.array(x_bo)
    y_bo = np.array(y_bo)
    xc = np.mean(x_bo)
    yc = np.mean(y_bo)
    # plt.plot(x_bo, y_bo)
    xp = x_bo[np.where(x_bo < xc)[0]]
    yp = y_bo[np.where(x_bo < xc)[0]]
    plt.plot(xp, yp)
    return xp, yp



def left_corner(x_bo, y_bo):
    # for left corner
    TABLE_DIMS = np.array((0.785, 1.80, 0.735))
    TABLE_DIMS[[0, 1, 2]]
    OFF_DIR = np.array([1, -1, 1])
    idx_edge = np.argmax(np.subtract(y_bo, x_bo))
    idx_p2 = np.argmax(np.subtract(np.negative(y_bo), x_bo))
    idx_p1 = np.argmin(np.subtract(np.negative(y_bo), x_bo))

    # corner points
    edge_left = np.array((x_bo[idx_edge], y_bo[idx_edge], -0.439))
    p1 = np.array((x_bo[idx_p1], y_bo[idx_p1], -0.439))
    p2 = np.array((x_bo[idx_p2], y_bo[idx_p2], -0.439))

    # First, find edge in view of case 1
    # case 1
    case = 1

    # rotation angle of table w.r.t z-axis
    num1 = abs(edge_left[1] - p1[1])
    den1 = abs(edge_left[0] - p1[0])
    num2 = abs(edge_left[0] - p2[0])
    den2 = abs(edge_left[1] - p2[1])
    theta1 = np.arctan2(num1, den1)
    theta2 = np.arctan2(num2, den2)
    # print(edge_left)
    # print(p1)
    # print(p2)

    # if (np.linalg.norm(edge_left - p) < 0.05):
    if (edge_left[0] < p2[0]):
        # actuaally, it is case 2
        case = 2
        num1 = abs(p1[1] - edge_left[1])
        den1 = abs(p1[0] - edge_left[0])
        num2 = abs(p2[0] - edge_left[0])
        den2 = abs(p2[1] - edge_left[1])
        theta1 = np.arctan2(num1, den1)
        theta2 = np.arctan2(num2, den2)
    print(edge_left)
    print(p1)
    print(p2)
    print(theta1, theta2)
    # print(abs(np.subtract(p1, edge_left)[1] / np.subtract(p1, edge_left)[0]))
    # print(abs(np.subtract(edge_left, p2)[0] / np.subtract(edge_left, p2)[1]))

    theta = (theta1 + theta2)/2
    T_bo = np.identity(4)
    # orientation of table
    if (case == 1):
        print("Detect table case 1")
        T_bo[:3, :3] = Rot_axis(3, -theta)
    elif (case == 2):
        print("Detect table case 2")
        T_bo[:3, :3] = Rot_axis(3, theta)
    T_bo[:3, 3] = (edge_left.T + np.divide(TABLE_DIMS[[0, 1, 2]] * OFF_DIR, 2).T)
    return T_bo


def right_corner(x_bo, y_bo):
    # for right corner
    TABLE_DIMS = np.array((0.785, 1.80, 0.735))
    TABLE_DIMS[[0, 1, 2]]
    OFF_DIR = np.array([1, 1, 1])
    idx_edge = np.argmax(np.subtract(np.negative(y_bo), x_bo))
    idx_p1 = np.argmax(np.subtract(y_bo, x_bo))
    idx_p2 = np.argmin(np.subtract(y_bo, x_bo))

    edge_right = np.array((x_bo[idx_edge], y_bo[idx_edge], -0.439))
    p1 = np.array((x_bo[idx_p1], y_bo[idx_p1], -0.439))
    p2 = np.array((x_bo[idx_p2], y_bo[idx_p2], -0.439))

    # First, find edge in view of case 1
    # case 1
    case = 1
    num1 = abs(p1[0] - edge_right[0])
    den1 = abs(p1[1] - edge_right[1])
    num2 = abs(p2[1] - edge_right[1])
    den2 = abs(p2[0] - edge_right[0])
    theta1 = np.arctan2(num1, den1)
    theta2 = np.arctan2(num2, den2)
    # print(edge_right)
    # print(p1)
    # print(p2)

    # if (np.linalg.norm(edge_right - p) < 0.05):
    if (edge_right[0] > p1[0]):
        # actually, it is case 2
        # case 2
        case = 2
        num = abs(p1[0] - edge_right[0])
        den = abs(p1[1] - edge_right[1])
        num2 = abs(p2[1] - edge_right[1])
        den2 = abs(p2[0] - edge_right[0])
        theta1 = np.arctan2(num1, den1)
        theta2 = np.arctan2(num2, den2)
    print(edge_right)
    print(p1)
    print(p2)
    print(theta1, theta2)
    # print(np.subtract(p1, edge_right))
    # print(np.subtract(edge_right, p2))
    # print(abs(np.subtract(p1, edge_right)[0] / np.subtract(p1, edge_right)[1]))
    # print(abs(np.subtract(edge_right, p2)[1] / np.subtract(edge_right, p2)[0]))

    theta = (theta1 + theta2) / 2
    # theta = max(theta1, theta2)
    T_bo = np.identity(4)
    T_bo[:3, 3] = (edge_right.T + np.divide(TABLE_DIMS[[0, 1, 2]] * OFF_DIR, 2).T)
    # orientation of table
    if (case == 1):
        print("Detect table case 1")
        T_bo[:3, :3] = Rot_axis(3, -theta)
    elif (case == 2):
        print("Detect table case 2")
        T_bo[:3, :3] = Rot_axis(3, theta)
    return T_bo

def refine_plane(gscene, track, viewpoint, T_ft, Qcur, TABLE_DIMS, collision,
                 cn_cur, CAM_HOST, CONNECT_CAM, CONNECT_INDY, ENABLE_DETECT):
    if CONNECT_CAM:
        rdict = stream_capture_image(ImageType.CloseView, host=CAM_HOST)
        set_cam_params(rdict['intrins'], rdict['depth_scale'])
        img_path = SAVE_DIR + '/table.png'
    else:
        img_path = DATASET_CAM_DIR + "/table_11.png"

    if ENABLE_DETECT:
        p_inliers = get_inliers(img_path)

        T_bc = viewpoint.get_tf(list2dict(Qcur, gscene.joint_names))
        viewpoint.draw_traj_coords([Qcur])
        x_bo, y_bo = point_proj(T_bc, p_inliers)

        if cn_cur == Corners.Left:
            T_bo = left_corner(x_bo, y_bo)
        elif cn_cur == Corners.Right:
            T_bo = right_corner(x_bo, y_bo)

        gscene.add_highlight_axis("table", "center", link_name="base_link", center=T_bo[:3, 3],
                                  orientation_mat=T_bo[:3, :3])

    else:
        T_bo = np.matmul(np.matmul(track.Toff, T_ft),
                         SE3(Rot_axis(3, random.uniform(-1, 1) * np.pi / 72),
                             np.concatenate([np.random.uniform(-0.1, 0.1, 2), [0]])
                             ))

    # geometry
    table_front = add_table(gscene, "table_front", TABLE_DIMS, T_bo, collision=collision)
    return table_front


def add_table(gscene, name, TABLE_DIMS, T_bo, collision):
    # geometry
    table = gscene.create_safe(gtype=GEOTYPE.BOX, name=name, link_name="base_link",
                                     dims=TABLE_DIMS, center=T_bo[:3, 3], rpy=Rot2rpy(T_bo[:3, :3]),
                                     color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True, collision=collision)
    RAIL_LENGTH = TABLE_DIMS[0]
    RAIL_THICKNESS = 0.02
    RAIL_MARGIN = 0.05
    RAIL_HEIGHT = 0.2
    RAIL_DIMS = (RAIL_LENGTH, RAIL_THICKNESS, RAIL_HEIGHT)
    RAIL_WIDTH_OFFSET = TABLE_DIMS[1]/2 + RAIL_MARGIN + RAIL_THICKNESS/2
    RAIL_VERTICAL_OFFSET = TABLE_DIMS[2]/2 + RAIL_HEIGHT/2
    gscene.create_safe(gtype=GEOTYPE.BOX, name=name+"_rail1", link_name="base_link",
                       dims=RAIL_DIMS,
                       center=(0, RAIL_WIDTH_OFFSET, RAIL_VERTICAL_OFFSET),
                       rpy=(0,0,0), color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True,
                       collision=collision, parent=name)
    gscene.create_safe(gtype=GEOTYPE.BOX, name=name+"_rail2", link_name="base_link",
                       dims=RAIL_DIMS,
                       center=(0, - RAIL_WIDTH_OFFSET, RAIL_VERTICAL_OFFSET),
                       rpy=(0,0,0), color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True,
                       collision=collision, parent=name)
    return table

