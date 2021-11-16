import os
import sys
import open3d as o3d
import numpy as np
import cv2
import copy
import matplotlib.pyplot as plt

import SharedArray as sa
import time
import random
from pkg.geometry.geotype import GEOTYPE
from pkg.utils.rotation_utils import *

sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/milestone_202110'))
from pkg.utils.rotation_utils import *
from collections import namedtuple
import open3d as o3d


def draw_registration_result(source, target, transformation, option_geos=[]):
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
                                       FOR_origin, FOR_model, FOR_target]+option_geos)

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=150))
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
def load_cdp(color_path, depth_path):
    intrins = [1280, 720,
               899.05322265625, 899.21044921875,
               654.8836669921875, 352.9295654296875]
    depth_scale = 0.000250000011874
    rdict = {}
    rdict['color'] = cv2.imread(
        os.path.join(color_path), flags=cv2.IMREAD_UNCHANGED)
    rdict['depth'] = cv2.imread(
        os.path.join(depth_path), flags=cv2.IMREAD_UNCHANGED)
    rdict['intrins'], rdict['depth_scale'] = intrins, depth_scale
    # Q = np.loadtxt(os.path.join(SAVE_DIR, obj_type + '.csv'), delimiter=",")
    return ColorDepthMap(**{k: rdict[k] for k in ColorDepthMap._fields})


# def rdict2cdp(rdict):
#     return ColorDepthMap(**{k: rdict[k] for k in ColorDepthMap._fields})


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
    def __init__(self, model, Toff, scale=[1e-3,1e-3,1e-3]):
        if isinstance(model, o3d.geometry.TriangleMesh):
            self.model = model
        elif isinstance(model, str):
            self.model = o3d.io.read_triangle_mesh(model)
            self.model.vertices = o3d.utility.Vector3dVector(
                np.asarray(self.model.vertices) * np.array([scale[0], scale[1], scale[2]]))
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
    def add_pointcloud(self, pcd, Tc=None, ratio=0.5):
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

        self.pcd_Tc_stack.append((pcd_cam, Tc, pcd))
        self.pcd = self.pcd_Tc_stack[0][2]
        for _pcd in self.pcd_Tc_stack[1:]:
            self.pcd += _pcd[2]
        if len(self.pcd_Tc_stack) > 1:
            self.pcd = self.pcd.uniform_down_sample(every_k_points=len(self.pcd_Tc_stack))
        self.model.compute_vertex_normals()
        self.model_sampled = self.model.sample_points_uniformly(
            number_of_points=int(len(np.array(self.pcd.points))*ratio))
        # self.model_sampled = self.model.sample_points_poisson_disk(
        #                                             number_of_points=int(len(np.array(self.pcd.points) * ratio)))
        return self.pcd

    ##
    # @param cdp ColorDepthMap
    # @param Tc camera transformation matrix
    def add_image(self, cdp, Tc=None, ratio=0.5):
        if Tc is None:
            Tc = np.identity(4)
        pcd_cam = cdp2pcd(cdp, depth_trunc=self.depth_trunc)
        pcd = cdp2pcd(cdp, Tc=Tc, depth_trunc=self.depth_trunc)

        self.pcd_Tc_stack.append((pcd_cam, Tc, pcd))
        self.pcd = self.pcd_Tc_stack[0][2]
        for _pcd in self.pcd_Tc_stack[1:]:
            self.pcd += _pcd[2]
        if len(self.pcd_Tc_stack) > 1:
            self.pcd = self.pcd.uniform_down_sample(every_k_points=len(self.pcd_Tc_stack))
        self.model.compute_vertex_normals()
        self.model_sampled = self.model.sample_points_uniformly(
            number_of_points=int(len(np.array(self.pcd.points)) * ratio))
        # self.model_sampled = self.model.sample_points_poisson_disk(
        #                                             number_of_points=int(len(np.array(self.pcd.points) * ratio)))
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
    # @param Tc_cur this is new camera transformation in pcd origin
    # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
    # @param thres max distance between corresponding points
    def compute_front_ICP(self, Tc_cur=None, To=None, thres=0.1,
                    relative_fitness=1e-15, relative_rmse=1e-15, max_iteration=500000,
                    voxel_size=0.04, visualize=False
                    ):
        if To is None:
            To, fitness = self.auto_init(0, voxel_size)

        if Tc_cur is None:
            Tc_cur = SE3(np.identity(3), (0, 0, 0))

        target = copy.deepcopy(self.pcd)

        T_cb = SE3_inv(Tc_cur) # base here is the point cloud base defined when added
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
        source = copy.deepcopy(front_pcd)

        if visualize:
            cam_coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
            cam_coord.transform(Tc_cur)
            self.draw(To, source, target, [cam_coord])

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

    def draw(self, To, source=None, target=None, option_geos=[]):
        if source is None: source = self.model_sampled
        if target is None: target = self.pcd
        To = np.matmul(To, self.Toff_inv)
        draw_registration_result(source, target, To, option_geos)

    def auto_init(self, init_idx=0, voxel_size=0.04):
        pcd_cam, Tc, _ = self.pcd_Tc_stack[init_idx]
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

##
# @param cdp    ColorDepthMap class
# @param T_bc   camera coordinate w.r.t global coordinate
def extract_mesh(cdp, voxel_lenght=4.0 / 512.0, sdf_trunc=0.04,
                 T_bc=None, visualize=False, depth_trunc=5.0
                 ):
    color = o3d.geometry.Image(cdp.color)
    depth = o3d.geometry.Image(cdp.depth)
    depth_scale = cdp.depth_scale

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale = 1/depth_scale,
                                                                    depth_trunc=depth_trunc, convert_rgb_to_intensity = False)

    # If T_bc is not given, then the mesh coordinate is camera coordinate
    if T_bc is None:
        T_bc = SE3(np.identity(3), (0,0,0))

    volume = o3d.integration.ScalableTSDFVolume(voxel_length=2.0 / 100.0, sdf_trunc=0.04,
                                                color_type=o3d.integration.TSDFVolumeColorType.RGB8)
    volume.integrate(rgbd_image,
                     o3d.camera.PinholeCameraIntrinsic(*cdp.intrins), SE3_inv(T_bc))

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    if visualize:
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=(0,0,0))
        o3d.visualization.draw_geometries([mesh, origin])

    return mesh

##
# @param mesh       o3d.geometry.TriangleMesh
# @param inside     if True, return points inside. if False, return points outside
# @param merge_rule np.any or np.all
def mask_boxes_mesh(mesh, boxes, Q, inside, merge_rule=np.all, link_ref="base_link"):
    mesh = copy.deepcopy(mesh)
    points = np.asarray(mesh.vertices)
    colors = np.asarray(mesh.vertex_colors)
    normals = np.asarray(mesh.vertex_normals)
    points4d = np.pad(points, ((0, 0), (0, 1)), 'constant', constant_values=1)
    mask_list = []
    for box in boxes:
        T_bx = box.get_tf(Q, from_link=link_ref)
        T_xb = SE3_inv(T_bx)
        abs_cuts = np.divide(box.dims, 2)
        points_x = np.matmul(points4d, T_xb.transpose())[:, :3]
        if inside:
            mask = np.all(np.abs(points_x) < abs_cuts, axis=-1)
        else:
            mask = np.any(np.abs(points_x) > abs_cuts, axis=-1)
        mask_list.append(mask)
    idc = np.where(merge_rule(mask_list, axis=0))[0]
    mesh.vertices = o3d.utility.Vector3dVector(points[idc])
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors[idc])
    mesh.vertex_normals = o3d.utility.Vector3dVector(normals[idc])

    triangles = np.asarray(mesh.triangles)
    t_normals = np.asarray(mesh.triangle_normals)

    idc_cvt = {i_old: i_new for i_new, i_old in enumerate(idc)}

    idc_vt = []
    triangles_new = []
    for i_t, trig in enumerate(triangles):
        if all([tp in idc for tp in trig]):
            idc_vt.append(i_t)
            triangles_new.append([idc_cvt[i_v] for i_v in trig])
    mesh.triangles = o3d.utility.Vector3iVector(triangles_new)
    mesh.triangle_normals = o3d.utility.Vector3dVector(t_normals[idc_vt])

    colors_old = np.asarray(mesh.vertex_colors)
    colors = []
    for trig in mesh.triangles:
        colors.append(np.mean(colors_old[trig], axis=0))
    return mesh, np.asarray(colors)

