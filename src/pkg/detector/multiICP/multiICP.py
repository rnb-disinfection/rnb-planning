import os
import sys
import cv2
import copy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from collections import namedtuple
from .config import *
from ..camera.kinect import Kinect
from ..camera.realsense import RealSense
from ..camera.camera_interface import CameraInterface
from ..detector_interface import DetectorInterface
from ...geometry.geotype import GEOTYPE
from ...utils.rotation_utils import *

##
# @class ColorDepthMap
# @param color numpy 8 bit array
# @param depth numpy 16 bit array
# @param intrins CamIntrins
# @param depth_scale multiplier for depthymap
ColorDepthMap = namedtuple('ColorDepthMap', ['color', 'depth', 'intrins', 'depth_scale'])

##
# @param pcd point cloud
# @param voxel_size voxel size of downsampling
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

##
# @param source source point cloud
# @param target target point cloud
# @param transformation estimated transformation from ICP to align source and target
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


##
# @class    MultiICP
# @brief    camera module with multi ICP for 3D object detection
#           Must call initialize/disconnect before/after usage.
#           If "Couldn't resolve requests" error is raised, check if it is connected with USB3.
#           You can check by running realsense-viewer from the terminal.
class MultiICP:
    ##
    # @param aruco_map   ArucoMap dictionary instance
    # @param camera     subclass instances of CameraInterface (realsense or kinect)
    def __init__(self, camera):
        self.camera = camera
        self.img_dim = (720, 1080)
        self.config_list = []
        self.depth_trunc = 5.0
        self.model = None
        self.cdp = None
        self.pcd = None
        self.pcd_Tc_stack = []
        self.model_sampled = None
        self.micp_dict = {}
        self.hrule_dict = {}


    ##
    # @brief initialize camera and set camera configuration
    def initialize(self):
        if isinstance(self.camera, RealSense):
            pipeline = self.camera.initialize()
            profile = pipeline.get_active_profile()
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = depth_sensor.get_depth_scale()

            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                     [0, color_intrinsics.fy, color_intrinsics.ppy],
                                     [0, 0, 1]])
            distCoeffs = np.array(color_intrinsics.coeffs)
            self.config_list = [cameraMatrix, distCoeffs, depth_scale]

        elif isinstance(self.camera, Kinect):
            self.camera.initialize()
            cameraMatrix, distCoeffs = self.camera.get_config()
            depth_scale = 1e-3
            self.config_list = [cameraMatrix, distCoeffs, depth_scale]

        print("Initialize Done")

    ##
    # @brief disconnect camera
    def disconnect(self):
        self.camera.disconnect()

    ##
    # @brief   get camera configuration
    # @return  cameraMatrix 3x3 camera matrix in pixel units,
    # @return  distCoeffs distortion coefficients, 5~14 float array
    # @return  depthscale scale of depth value
    def get_camera_config(self):
        return self.config_list

    ##
    # @brief   get aligned RGB image and depthmap
    def get_image(self):
        color_image, depth_image = self.camera.get_image_depthmap()
        self.img_dim = (color_image.shape[0], color_image.shape[1])
        return color_image, depth_image

    ##
    # @param cdp ColorDepthMap
    # @param mask segmented result from object detection algorithm
    def apply_mask(cdp, mask):
        mask_u8 = np.zeros_like(mask).astype(np.uint8)
        mask_u8[np.where(mask)] = 255
        color_masked = cv2.bitwise_and(cdp.color, cdp.color, mask=mask_u8).astype(np.uint8)
        depth_masked = cv2.bitwise_and(cdp.depth, cdp.depth, mask=mask_u8).astype(np.uint16)
        return ColorDepthMap(color_masked, depth_masked, cdp.intrins, cdp.depth_scale)

    ##
    # @param cdp ColorDepthMap
    # @param Tc camera coord w.r.t base coord
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

    ##
    # @brief detect objects in 2D image through mmdet
    # @param  color_image   color image of object
    # @param  depth_image   color image of object
    # @param  Tc camera transformation matrix
    def detect(self, sd, obj_name, color_image=None, depth_image=None,
               Tc=None, ratio=0.5):
        self.obj_name = obj_name
        if color_image is None or depth_image is None:
            color_image, depth_image = self.get_image()

        camera_mtx = self.config_list[0]
        cam_intrins = [self.img_dim[1], self.img_dim[0],
                       camera_mtx[0,0],camera_mtx[1,1],
                       camera_mtx[0,2],camera_mtx[1,2]]
        depth_scale = self.config_list[2]
        cdp = ColorDepthMap(color_image, depth_image, cam_intrins, depth_scale)

        # Output of inference(mask for detected object you choose)
        mask_out_list = sd.inference(color_img=cdp.color)
        mask_out = mask_out_list[class_dict[obj_name]]
        cdp_masked = apply_mask(cdp, mask_out)

        self.cdp = cdp_masked

        if Tc is None:
            Tc = np.identity(4)
        pcd_cam = cdp2pcd(cdp_masked, depth_trunc=self.depth_trunc)
        pcd = cdp2pcd(cdp_masked, Tc=Tc, depth_trunc=self.depth_trunc)

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
        return cdp_masked.color


    ##
    # @brief add model
    # @param model open3d.geometry.TriangleMesh or file path
    # @param hrule  GeometryItem coordinate in TriangleMesh coordinate. externally, we use geometry coordinate for all TFs
    def set_config(self, model, hrule):
        self.model = self.add_model(model, Toff, scale)
        self.micp_dict = {}
        self.hrule_dict = {}


    ##
    # @brief add cad model to compute ICP
    # @param model open3d.geometry.TriangleMesh or file path
    # @param Toff  GeometryItem coordinate in TriangleMesh coordinate. externally, we use geometry coordinate for all TFs
    # @param scale scale multiplier if model is given as file path
    def add_model(self, model, Toff=None, scale=[1e-3,1e-3,1e-3]):
        if Toff is None:
            self.Toff = np.identity(4)
            self.Toff_inv = SE3_inv(self.Toff)
        else :
            self.Toff = Toff
            self.Toff_inv = SE3_inv(self.Toff)
        if isinstance(model, o3d.geometry.TriangleMesh):
            self.model = model
        elif isinstance(model, str):
            self.model = o3d.io.read_triangle_mesh(model)
            self.model.vertices = o3d.utility.Vector3dVector(
                np.asarray(self.model.vertices) * np.array([scale[0],scale[1],scale[2]]))
        else:
            raise (NotImplementedError("non available input for model : \n".format(model)))

    ##
    # @param cdp open3d.geometry.PointCloud
    # @param Tc camera transformation matrix
    def add_pointcloud(self, pcd, Tc=None, ratio=0.5):
        if Tc is None:
            pcd_cam = copy.deepcopy(pcd)
            pcd = copy.deepcopy(pcd)
            Tc = np.identity(4)
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

        self.pose = ICP_result
        self.pose_dict[self.obj_name] = ICP_result
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
            if self.pose is not None:
                To = self.pose
            else:
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

        self.pose = ICP_result
        self.pose_dict[self.obj_name] = ICP_result
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
        self.cdp = None

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

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1 / depth_scale,
                                                                        depth_trunc=depth_trunc,
                                                                        convert_rgb_to_intensity=False)

        # If T_bc is not given, then the mesh coordinate is camera coordinate
        if T_bc is None:
            T_bc = SE3(np.identity(3), (0, 0, 0))

        volume = o3d.integration.ScalableTSDFVolume(voxel_length=2.0 / 100.0, sdf_trunc=0.04,
                                                    color_type=o3d.integration.TSDFVolumeColorType.RGB8)
        volume.integrate(rgbd_image,
                         o3d.camera.PinholeCameraIntrinsic(*cdp.intrins), SE3_inv(T_bc))

        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()

        if visualize:
            origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=(0, 0, 0))
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

    ##
    # @brief    list registered targets of specific detection level
    # @param    gscene  geometry scene
    # @param    detection_level list of target detection levels
    # @return   names list of target names
    def get_targets_of_levels(self, gscene, detection_levels=None):
        obj_info = get_obj_info()
        names = []
        for name in class_dict.keys():
            try:
                handle = gscene.NAME_DICT[name]
                if obj_info[name].dlevel == detection_levels:
                    names.append(name)
            except:
                pass
        return names

    ##
    # @brief    Acquire geometry kwargs of item
    # @param    gscene  geometry scene
    # @param    name    item name
    # @return   kwargs  kwargs
    def get_geometry_kwargs(self, name):
        obj_info = get_obj_info()
        if name in class_dict.keys():
            try:
                handle = gscene.NAME_DICT[name]
                return obj_info[name].get_geometry_kwargs()
            except:
                raise (NotImplementedError("{} is not detected yet. Non available \n".format(name)))
        else :
            raise (NotImplementedError("{} is not detected yet. Non available \n".format(name)))

    ##
    # @brief    add axis marker to GeometryHandle
    def add_item_axis(self, gscene, hl_key, item, axis_name=None):
        oname = item.oname
        axis_name = axis_name or oname
        if oname in gscene.NAME_DICT:
            aobj = gscene.NAME_DICT[oname]
            link_name = aobj.link_name
            Toff = np.matmul(aobj.Toff, item.Toff)
        else:
            link_candis = list(set([lname for lname in gscene.link_names
                                    if oname in lname
                                    and lname in [child_pair[1]
                                                  for child_pair
                                                  in gscene.urdf_content.child_map["base_link"]]
                                    ]))
            if len(link_candis) == 0:
                link_name = "base_link"
            elif len(link_candis) == 1:
                link_name = link_candis[0]
            else:
                raise(RuntimeError("Multiple object link candidates - marker link cannot be determined"))
            Toff = item.Toff
        gscene.add_highlight_axis(hl_key, axis_name, link_name, Toff[:3,3], Toff[:3,:3], axis="xyz")