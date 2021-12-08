import os
import sys
import cv2
import copy
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from collections import namedtuple
from .heuristics import *
from ..camera.kinect import Kinect
from ..camera.realsense import RealSense
from ..camera.camera_interface import CameraInterface
from ..detector_interface import DetectorInterface
from ...geometry.geotype import GEOTYPE
from ...utils.rotation_utils import *
from ...utils.utils import TextColors

##
# @class ColorDepthMap
# @param color numpy 8 bit array
# @param depth numpy 16 bit array
# @param intrins CamIntrins
# @param depth_scale multiplier for depthymap
ColorDepthMap = namedtuple('ColorDepthMap', ['color', 'depth', 'intrins', 'depth_scale'])


##
# @brief convert cdp to pcd
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
    # @param camera     subclass instances of CameraInterface (realsense or kinect)
    def __init__(self, camera):
        self.camera = camera
        self.img_dim = (720, 1280)
        self.ratio = 0.3
        self.config_list = []
        self.micp_dict = {}
        self.objectPose_dict = {}
        self.gscene = None
        self.crob = None
        self.sd = None
        self.visualize = False
        self.cache = None
        self.pcd_total = None


    ##
    # @brief initialize camera and set camera configuration
    def initialize(self):
        if isinstance(self.camera, RealSense):
            # for Realsense camera
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
            # for Azure Kinect camera
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
        Q = self.crob.get_real_robot_pose()
        return color_image, depth_image, Q

    ##
    # @brief  add micp_dict, hrule_dict, combined robot, geometry scene and shared detector
    # @param  micp_dict MultiICP class for each object
    # @param  sd   shared detector to detect object
    # @param  crob   CombinedRobot
    # @param  viewpoint     GeometryItem for viewpoint
    def set_config(self, micp_dict, sd, crob, viewpoint):
        self.micp_dict = micp_dict
        self.crob = crob
        self.sd = sd
        self.viewpoint = viewpoint
        self.gscene = viewpoint.gscene

    ##
    # @brief  add arbitrary color, depth image and joint value
    # @param  color_image   color image of object
    # @param  depth_image   depth image of object
    # @param  Q             joint values of robot
    def cache_sensor(self, color_image, depth_image, Q, cam_intrins, depth_scale):
        self.cache = color_image, depth_image, Q, cam_intrins, depth_scale


    ##
    # @brief    detect 3D objects pose
    # @param    name_mask   object names to detect
    # @param    level_mask  list of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @param    visualize   visualize the detection result, especially visualization in Open3D during ICP
    def detect(self, name_mask=None, level_mask=None, visualize=False):
        if level_mask is not None:
            name_mask_level = self.get_targets_of_levels(level_mask)
            if name_mask is not None:
                name_mask = list(set(name_mask).intersection(set(name_mask_level)))
            else:
                name_mask = name_mask_level
        print("name_mask is {}".format(name_mask))

        if self.crob is None:
            TextColors.YELLOW.println("[WARN] CombinedRobot is not set: call set_config()")
            return {}
        if self.cache is None:
            color_image, depth_image, Q = self.get_image()
            camera_mtx = self.config_list[0]
            cam_intrins = [self.img_dim[1], self.img_dim[0],
                           camera_mtx[0, 0], camera_mtx[1, 1],
                           camera_mtx[0, 2], camera_mtx[1, 2]]
            depth_scale = self.config_list[2]
        else:
            color_image, depth_image, Q, cam_intrins, depth_scale = self.cache
            self.cache = None

        self.last_input = color_image, depth_image, Q, cam_intrins, depth_scale

        cdp = ColorDepthMap(color_image, depth_image, cam_intrins, depth_scale)
        Tc = self.viewpoint.get_tf(Q)
        T_cb = SE3_inv(Tc)

        if self.sd is None:
            TextColors.YELLOW.println("[WARN] SharedDetector is not set: call set_config()")
            return {}
        # Output of inference(mask for detected object)
        mask_out_list = self.sd.inference(color_img=cdp.color)
        mask_dict = {}
        for idx in range(80):
            if np.any(mask_out_list[idx]):
                for name, value in class_dict.items():
                    if name_mask is not None:
                        if name in name_mask:
                            if value == idx:
                                num = int(np.max(mask_out_list[value]))
                                print('===== Detected : {}, {} object(s) ====='.format(name, num))
                                mask_dict[name] = mask_out_list[value]
                    else:
                        if value == idx:
                            num = int(np.max(mask_out_list[value]))
                            print('===== Detected : {}, {} object(s) ====='.format(name, num))
                            mask_dict[name] = mask_out_list[value]
            else:
                pass

        objectPose_dict = {}
        hrule_targets_dict = {}
        detect_dict = {}

        if name_mask is not None:
            for name, micp in self.micp_dict.items():
                if name in name_mask:
                    detect_dict[name] = micp
        else:
            detect_dict = self.micp_dict

        for name, micp in detect_dict.items():
            if name in mask_dict.keys():
                micp.clear()

                # add to micp
                micp.set_model()
                masks = mask_dict[name]
                mask_list = []
                mask_zero = np.empty((self.img_dim[0],self.img_dim[1]), dtype=bool)
                mask_zero[:,:] = False
                mask_zero[np.where(masks ==  1)] = True
                mask_list.append(mask_zero)

                # for i in range(int(np.max(masks))):
                #     mask_tmp = mask_zero
                #     mask_tmp[np.where(masks==i+1)] = True
                #     mask_list.append(mask_tmp)

                # Check whether the object exists in gscene
                try:
                    g_handle = self.gscene.NAME_DICT[name]
                    Tguess = g_handle.get_tf(Q)
                except:
                    print("\n'{}' is not in gscene. Use manual input for initial guess\n".format(name))
                    pass

                for i_m, mask in enumerate(mask_list):
                    cdp_masked = apply_mask(cdp, mask)
                    micp.make_pcd(cdp_masked, ratio=self.ratio)
                    Tguess = None if micp.grule is None else micp.grule.get_initial(micp.pcd)

                    # Compute ICP, front iCP
                    T, _ = micp.compute_ICP(To=Tguess, visualize=visualize)
                    T, _ = micp.compute_front_ICP(To=T, visualize=visualize)

                    # name_i = "{}_{:02}".format(name, i_m)
                    objectPose_dict[name] = np.matmul(Tc, T)
                    self.objectPose_dict[name] = np.matmul(Tc, T)
                    print('Found 6DoF pose of {}'.format(name))
            elif micp.hrule is not None:
                micp.clear()
                hrule_targets_dict[name] = micp
            else:
                raise (RuntimeError("Detection rule undefined for {}".format(name)))

        for name, micp in sorted(hrule_targets_dict.items()):
            # add to micp
            micp.set_model()
            micp.make_pcd(cdp, ratio=self.ratio)
            print('===== Apply heuristic rule for {} ====='.format(name))
            hrule = micp.hrule
            mrule = hrule.update_rule(micp, detect_dict[hrule.parent], Tc=Tc)
            micp.make_pcd(cdp, ratio=self.ratio)
            # micp.make_pcd(cdp, Tc=Tc, ratio=self.ratio)
            pcd_dict = mrule.apply_rule(micp.pcd, objectPose_dict)
            T_list = []

            # Check whether the object exists in gscene
            try:
                g_handle = self.gscene.NAME_DICT[name]
                initial_guess = g_handle.get_tf(Q)
            except:
                print("\n'{}' is not in gscene. Use manual input for initial guess\n".format(name))
                pass

            for name_i, pcd in pcd_dict.items():
                micp.pcd = pcd
                Tguess = micp.grule.get_initial(micp.pcd,
                                                        R=detect_dict[hrule.parent].pose[:3, :3])

                # Compute ICP, front iCP
                T, _ = micp.compute_ICP(To=Tguess, visualize=visualize)
                T, _ = micp.compute_front_ICP(To=T, visualize=visualize)
                T_list.append(T)
                objectPose_dict[name_i] = np.matmul(Tc, T)
                self.objectPose_dict[name_i] = np.matmul(Tc, T)
                print('Found 6DoF pose of {}'.format(name_i))

            # for i_t, T in enumerate(T_list):
            #     name_i = "{}_{:02}".format(name, i_t)
            #     objectPose_dict[name_i] = T
            #     self.objectPose_dict[name_i] = T
            #     print('Found 6DoF pose of {}'.format(name_i))

        return objectPose_dict

    ##
    # @brief    list registered targets of specific detection level
    # @param    detection_level list of target detection levels
    # @return   names list of target names
    def get_targets_of_levels(self, detection_levels=None):
        names = []
        for name, info in self.micp_dict.items():
            obj_info = info.get_info()
            if obj_info.dlevel in detection_levels:
                names.append(name)
        return names

    ##
    # @brief    Acquire geometry kwargs of item
    # @param    name    item name
    # @return   kwargs  kwargs
    def get_geometry_kwargs(self, name):
        if "_" in name:
            name_cat = name.split("_")[0]
        else:
            name_cat = name
        micp = self.micp_dict[name_cat]
        model = micp.model
        return dict(gtype=GEOTYPE.MESH, name=name_cat,
                    dims=(0.1, 0.1, 0.1), color=(0.8, 0.8, 0.8, 1),
                    display=True, fixed=True, collision=False,
                    vertices=np.matmul(np.asarray(model.vertices), micp.Toff_inv[:3,:3].transpose())+micp.Toff_inv[:3,3],
                    triangles=np.asarray(model.triangles))

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


##
# @class    MultiICP_Obj
# @brief    class for each object which has pcd, model information only to be dected
class MultiICP_Obj:
    ##
    # @param obj_info   object information
    # @param hrule      heuristic rule class
    # @param grule      initial guess rule class
    def __init__(self, obj_info, hrule=None, grule=None):
        self.depth_trunc = 5.0
        self.model = None
        self.cdp = None
        self.pcd = None
        self.pcd_Tc_stack = []
        self.model_sampled = None
        self.obj_info = obj_info
        self.hrule = hrule
        self.grule = grule
        self.pose = None

    def get_info(self):
        return self.obj_info

    def get_rule(self):
        return self.hrule, self.grule

    ##
    # @brief add model mesh
    # @param model_name name of CAD model
    def set_model(self):
        # obj_info = get_obj_info()
        # model_info = obj_info[model_name]
        model_info = self.obj_info
        if model_info.Toff is None:
            self.Toff = np.identity(4)
            self.Toff_inv = SE3_inv(self.Toff)
        else:
            self.Toff = model_info.Toff
            self.Toff_inv = SE3_inv(self.Toff)

        if isinstance(model_info.url, o3d.geometry.TriangleMesh):
            self.model = model
        elif isinstance(model_info.url, str):
            self.model = o3d.io.read_triangle_mesh(model_info.url)
            self.model.vertices = o3d.utility.Vector3dVector(
                np.asarray(self.model.vertices) * np.array([model_info.scale[0],
                                                            model_info.scale[1],
                                                            model_info.scale[2]]))
        else:
            raise (NotImplementedError("non available input for model : \n".format(model)))

    ##
    # @brief add pcd from image, sampled pcd from mesh
    # @param cdp_masked ColorDepthMap
    # @param Tc camera coord w.r.t base coord
    # @param ratio ratio of number of points
    def make_pcd(self, cdp_masked, Tc=None, ratio=0.3):
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
        self.pcd = self.pcd.uniform_down_sample(every_k_points=int(1/ratio))
        self.model.compute_vertex_normals()
        self.model_sampled = self.model.sample_points_uniformly(
            number_of_points=int(len(np.array(self.pcd.points)) * ratio))
        # self.model_sampled = self.model.sample_points_poisson_disk(
        #                                             number_of_points=int(len(np.array(self.pcd.points) * ratio)))

    ##
    # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
    # @param thres max distance between corresponding points
    def compute_ICP(self, To=None, thres=0.15,
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
        return ICP_result, reg_p2p.fitness

    ##
    # @param Tc_cur this is new camera transformation in pcd origin
    # @param To    initial transformation matrix of geometry object in the intended icp origin coordinate
    # @param thres max distance between corresponding points
    def compute_front_ICP(self, Tc_cur=None, To=None, thres=0.12,
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

        T_cb = SE3_inv(Tc_cur)  # base here is the point cloud base defined when added
        T_co = np.matmul(np.matmul(T_cb, To), self.Toff_inv)
        # model_mesh = self.model.compute_vertex_normals()
        model_pcd = self.model_sampled

        normals = np.asarray(model_pcd.normals)
        points = np.asarray(model_pcd.points)
        # point_normals = normals
        # view_vec = SE3_inv(Tguess)[:3,2]
        Poc = (np.max(points, axis=0) + np.min(points, axis=0)) / 2
        P_co = np.matmul(T_co[:3, :3], Poc) + T_co[:3, 3]

        point_normals = np.matmul(T_co[:3, :3], normals.T).T
        view_vec = P_co / np.linalg.norm(P_co)
        idx = []
        for i in range(len(point_normals)):
            if np.dot(view_vec, point_normals[i]) < 0:
                idx.append(i)

        # pts = np.zeros((len(idx), 3))
        # for i in range(len(idx)):
        #     pts[i] = points[idx[i]]
        pts_md = np.array(points[idx])

        point_c = np.asarray(np.matmul(pts_md, np.transpose(T_co[:3, :3])) + T_co[:3, 3])
        points_sph = np.transpose(cart2spher(*np.transpose(np.matmul(point_c, Rot_axis(1, np.pi / 2)))))
        points_xyz = np.transpose(spher2cart(*np.transpose(points_sph)))

        verts = np.asarray(np.matmul(self.model.vertices, np.transpose(T_co[:3, :3])) + T_co[:3, 3])
        verts_sph = np.transpose(cart2spher(*np.transpose(np.matmul(verts, Rot_axis(1, np.pi / 2)))))
        trigs = np.asarray(self.model.triangles)

        pts = points_sph[:, 1:]
        dists = points_sph[:, 0] + 3e-3
        in_mask_accum = np.zeros(len(points_sph), dtype=bool)
        for count, (i, j, k) in enumerate(trigs):
            r = np.max(verts_sph[[i, j, k]][:, 0])
            p1, p2, p3 = verts_sph[[i, j, k]][:, 1:]
            p12 = p1 - p2
            pt2 = pts - p2
            p23 = p2 - p3
            pt3 = pts - p3
            p31 = p3 - p1
            pt1 = pts - p1
            sign2 = np.sign(np.cross(p12, pt2))
            sign3 = np.sign(np.cross(p23, pt3))
            sign1 = np.sign(np.cross(p31, pt1))

            in_mask = np.all([sign1 == sign2,
                              sign2 == sign3,
                              dists > r], axis=0)

            in_mask_accum = np.logical_or(in_mask_accum, in_mask)
        idc_masked = np.logical_not(in_mask_accum)

        points_front = np.asarray(pts_md)[idc_masked]

        front_pcd = o3d.geometry.PointCloud()
        front_pcd.points = o3d.utility.Vector3dVector(points_front)
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
# @param cdp ColorDepthMap
# @param mask segmented result from object detection algorithm
def apply_mask(cdp, mask):
    mask_u8 = np.zeros_like(mask).astype(np.uint8)
    mask_u8[np.where(mask)] = 255
    color_masked = cv2.bitwise_and(cdp.color, cdp.color, mask=mask_u8).astype(np.uint8)
    depth_masked = cv2.bitwise_and(cdp.depth, cdp.depth, mask=mask_u8).astype(np.uint16)
    return ColorDepthMap(color_masked, depth_masked, cdp.intrins, cdp.depth_scale)
