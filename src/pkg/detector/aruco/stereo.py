from .detector import *
from ..detector_interface import DetectorInterface
from ...utils.rotation_utils import *
import autograd.numpy as np

LAST_LINE = np.array([[0, 0, 0, 1]])
R_ZERO = np.identity(3)

from pymanopt.manifolds import Rotations
from pymanopt.manifolds import Euclidean
from pymanopt.manifolds import Product
from pymanopt import Problem
from pymanopt.solvers import TrustRegions


STEREO_TIMEOUT = 10

##
# @class ArucoStereo
# @brief Stereo camera aruco marker detector.
#        Must call initialize/disconnect before/end of usage
#        Resolution of first camera is used as reference. Use a camera with higher resolution as the first camera.
class ArucoStereo(DetectorInterface):
    ##
    # @param aruco_map   ArucoMap dictionary instance
    # @param camera_list list of 2 subclass instances of CameraInterface
    def __init__(self, aruco_map, camera_list):
        self.aruco_map = aruco_map
        self.camera_list = camera_list
        self.config_list = []
        self.T_c12 = None

    ##
    # @brief initialize cameras
    def initialize(self):
        for cam in self.camera_list:
            cam.initialize()

    ##
    # @brief disconnect cameras
    def disconnnect(self):
        for cam in self.camera_list:
            cam.disconnect()

    ##
    # @brief calibrate stereo offset
    # @return config_ref config of ref cam
    # @return config_sub config of sub cam
    # @return T_c12      Transformtation from sub cam to ref cam
    def calibrate(self, N_trial=5):
        retval_min = 10000
        self.T_c12 = None
        self.config_list = []
        for cam in self.camera_list:
            self.config_list.append(cam.get_config())

        ref_config = self.config_list[0]
        sub_config = self.config_list[1]

        for _ in range(N_trial):
            images = []
            image_shapes = []
            for cam in self.camera_list:
                image = cam.get_image()
                images.append(image)
                image_shapes.append(image.shape)

            ref_image = images[0]
            sub_image = images[1]

            ref_shape = image_shapes[0]
            sub_shape = image_shapes[1]

            # resize
            imageSize = (ref_shape[1], ref_shape[0])
            sub_camK = sub_config[0] * (ref_shape[0] / sub_shape[0])
            sub_camK[2, 2] = 1
            sub_config_res = (sub_camK, np.zeros_like(ref_config[1]))
            sub_img_res = cv2.resize(sub_image, imageSize)

            ref_corners, ref_ids, ref_rejectedImgPoints = aruco.detectMarkers(ref_image, self.aruco_map.dictionary, parameters=aruco_param)
            sub_corners, sub_ids, sub_rejectedImgPoints = aruco.detectMarkers(sub_img_res, self.aruco_map.dictionary, parameters=aruco_param)

            ref_corner_dict = {k[0]: v[0] for k, v in zip(ref_ids, ref_corners)}
            sub_corner_dict = {k[0]: v[0] for k, v in zip(sub_ids, sub_corners)}
            keys_all = sorted(set(ref_corner_dict.keys() + sub_corner_dict.keys()))

            objectPoints = []
            imagePoints_kn = []
            imagePoints_rs = []
            for oname, markers in self.aruco_map.items():
                objectPoints_o = []
                imagePoints_ref_o = []
                imagePoints_sub_o = []
                occluded = 0
                for mk in markers:
                    if mk.idx in ref_corner_dict and mk.idx in sub_corner_dict:
                        objectPoints_o.append(mk.corners.copy())
                        imagePoints_ref_o.append(ref_corner_dict[mk.idx].copy())
                        imagePoints_sub_o.append(sub_corner_dict[mk.idx].copy())
                    elif mk.idx in ref_corner_dict:
                        occluded += 1

                if ((len(objectPoints_o) == 0) or
                        (len(objectPoints_o) <= 2 and (len(objectPoints_o)+occluded >= 3))):
                    continue
                objectPoints.append(np.concatenate(objectPoints_o, axis=0).astype(np.float32))
                imagePoints_kn.append(np.concatenate(imagePoints_ref_o, axis=0))
                imagePoints_rs.append(np.concatenate(imagePoints_sub_o, axis=0))
            retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = \
                cv2.stereoCalibrate(objectPoints,
                                    imagePoints_kn,
                                    imagePoints_rs,
                                    cameraMatrix1=ref_config[0], distCoeffs1=ref_config[1],
                                    cameraMatrix2=sub_config_res[0], distCoeffs2=sub_config_res[1],
                                    imageSize=imageSize, flags=cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_RATIONAL_MODEL)
            if retval < retval_min:
                retval_min = retval
                T_c21 = SE3(R, T.flatten())
                T_c12 = SE3_inv(T_c21)
                self.T_c12 = T_c12
        return self.config_list + [self.T_c12]

    ##
    # @brief detect objects
    # @param    name_mask   object names to detect
    # @param    level_mask  list of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @return object_pose_dict dictionary for object transformations
    def detect(self, name_mask=None, level_mask=None):
        if level_mask is not None:
            name_mask_level = self.get_targets_of_levels(level_mask)
            if name_mask is not None:
                name_mask = list(set(name_mask).intersection(set(name_mask_level)))
            else:
                name_mask = name_mask_level
        return self.get_object_pose_dict(name_mask=name_mask)[0]

    ##
    # @brief    list registered targets of specific detection level
    # @param    detection_level list of target detection levels
    # @return   names target names
    def get_targets_of_levels(self, detection_levels=None):
        return [atem.name for atem in self.aruco_map.values() if atem.dlevel in detection_levels]
    ##
    # @brief    Acquire geometry kwargs of item
    # @param    name    item name
    # @return   kwargs  kwargs
    def get_geometry_kwargs(self, name):
        return self.aruco_map[name].get_geometry_kwargs()

    ##
    # @brief get object_pose_dict from stereo camera
    # @return object_pose_dict object pose (4x4) dictionary from reference camera
    # @return ref_corner_dict corner dictionary from reference camera
    # @return ref_img reference image
    # @return sub_img sub-camera image
    # @return sub_objectPose_dict object pose (4x4) dictionary from sub-camera
    # @return sub_corner_dict corner dictionary from sub-camera
    def get_object_pose_dict(self, ref_config=None, sub_config=None, T_c12=None, name_mask=None):
        if T_c12 is None:
            T_c12 = self.T_c12
        T_c21 = SE3_inv(T_c12)
        if ref_config is None:
            ref_config = self.config_list[0]
        if sub_config is None:
            sub_config = self.config_list[1]

        ref_img = self.camera_list[0].get_image()
        sub_img = self.camera_list[1].get_image()

        ref_objectPose_dict, ref_corner_dict = self.aruco_map.get_object_pose_dict(ref_img, *ref_config, name_mask=name_mask)
        sub_objectPose_dict, sub_corner_dict = self.aruco_map.get_object_pose_dict(sub_img, *sub_config, name_mask=name_mask)

        projMatr1 = np.matmul(ref_config[0], np.identity(4)[:3])
        projMatr2 = np.matmul(sub_config[0], T_c21[:3])

        objectPose_dict = {}

        for oname, markers in self.aruco_map.items():
            if name_mask is not None and oname not in name_mask:
                continue
            ref_count = 0
            sub_count = 0
            objectPoints_dp = []
            imagePoints_ref_dp = []
            imagePoints_sub_dp = []

            for mk in markers:
                if mk.idx in ref_corner_dict:
                    ref_count += 1
                if mk.idx in sub_corner_dict:
                    sub_count += 1
                if mk.idx in ref_corner_dict and mk.idx in sub_corner_dict:
                    objectPoints_dp.append(mk.corners.copy())
                    imagePoints_ref_dp.append(ref_corner_dict[mk.idx].copy())
                    imagePoints_sub_dp.append(sub_corner_dict[mk.idx].copy())

            if ref_count + sub_count > 0:
                if ref_count >= sub_count:
                    objectPose_dict[oname] = ref_objectPose_dict[oname]
                else:
                    objectPose_dict[oname] = np.matmul(T_c12, sub_objectPose_dict[oname])
            elif oname in ref_objectPose_dict:
                objectPose_dict[oname] = ref_objectPose_dict[oname]

            if len(objectPoints_dp) > 0:
                objectPoints_dp = np.concatenate(objectPoints_dp, axis=0).astype(np.float32).transpose()
                imagePoints_ref_dp = np.concatenate(imagePoints_ref_dp, axis=0)
                imagePoints_ref_dp = np.reshape(
                    cv2.undistortPoints(np.expand_dims(imagePoints_ref_dp, axis=1), ref_config[0], ref_config[1], None,
                                        ref_config[0]), [-1, 2]).transpose()
                imagePoints_sub_dp = np.concatenate(imagePoints_sub_dp, axis=0)
                imagePoints_sub_dp = np.reshape(
                    cv2.undistortPoints(np.expand_dims(imagePoints_sub_dp, axis=1), sub_config[0], sub_config[1], None,
                                        sub_config[0]), [-1, 2]).transpose()
                points4D = cv2.triangulatePoints(projMatr1, projMatr2, imagePoints_ref_dp, imagePoints_sub_dp)
                point3D_o = points4D[:3] / points4D[3:]
                objectPose = objectPose_dict[oname]
                objectPoints_tf = np.matmul(objectPose[:3, :3], objectPoints_dp) + objectPose[:3, 3:]
                dPco = np.mean(point3D_o - objectPoints_tf, axis=1)
                objectPose_dict[oname][:3, 3] += dPco

        return objectPose_dict, ref_corner_dict, ref_img, sub_img, sub_objectPose_dict, sub_corner_dict

    ##
    # @brief (deprecated) get object_pose_dict from stereo camera, minimizing triangular error
    def get_object_pose_dict_stereo_triangulate_error_minimize(self, ref_config=None, sub_config=None, T_c12=None):
        if T_c12 is None:
            T_c12 = self.T_c12
        T_c21 = SE3_inv(T_c12)
        if ref_config is None:
            ref_config = self.config_list[0]
        if sub_config is None:
            sub_config = self.config_list[1]

        ref_img = self.camera_list[0].get_image()
        sub_img = self.camera_list[1].get_image()

        ref_corners, ref_ids, ref_rejectedImgPoints = aruco.detectMarkers(ref_img, self.aruco_map.dictionary, parameters=aruco_param)
        sub_corners, sub_ids, sub_rejectedImgPoints = aruco.detectMarkers(sub_img, self.aruco_map.dictionary, parameters=aruco_param)

        projMatr1 = np.matmul(ref_config[0], np.identity(4)[:3])
        projMatr2 = np.matmul(sub_config[0], T_c21[:3])

        ref_corner_dict = {k[0]: v[0] for k, v in zip(ref_ids, ref_corners)}
        sub_corner_dict = {k[0]: v[0] for k, v in zip(sub_ids, sub_corners)}

        objectPose_dict = {}
        err_dict = {}
        objectPoints_dict = {}
        point3D_dict = {}
        oname_omit = []
        for oname, markers in self.aruco_map.items():
            objectPoints_o = []
            imagePoints_ref_o = []
            imagePoints_sub_o = []
            for mk in markers:
                if mk.idx in ref_corner_dict and mk.idx in sub_corner_dict:
                    objectPoints_o.append(mk.corners.copy())
                    imagePoints_ref_o.append(ref_corner_dict[mk.idx].copy())
                    imagePoints_sub_o.append(sub_corner_dict[mk.idx].copy())

            if len(objectPoints_o) == 0:
                oname_omit.append(oname)
                continue
            objectPoints_o = np.concatenate(objectPoints_o, axis=0).astype(np.float32).transpose()
            imagePoints_ref_o = np.concatenate(imagePoints_ref_o, axis=0)
            imagePoints_ref_o = np.reshape(
                cv2.undistortPoints(np.expand_dims(imagePoints_ref_o, axis=1), ref_config[0], ref_config[1], None,
                                    ref_config[0]), [-1, 2]).transpose()
            imagePoints_sub_o = np.concatenate(imagePoints_sub_o, axis=0).transpose()
            points4D = cv2.triangulatePoints(projMatr1, projMatr2, imagePoints_ref_o, imagePoints_sub_o)
            point3D_o = points4D[:3] / points4D[3:]

            Tobj, err = self.__calc_T_stereo(objectPoints_o, point3D_o)
            objectPose_dict[oname] = Tobj
            err_dict[oname] = err
            objectPoints_dict[oname] = objectPoints_o
            point3D_dict[oname] = point3D_o
        objectPose_dict_omit, corner_dict = self.aruco_map.get_object_pose_dict(ref_img, ref_config[0], ref_config[1],
                                                                           name_mask=oname_omit)
        objectPose_dict.update(objectPose_dict_omit)
        ref_corner_dict.update(corner_dict)
        return objectPose_dict, ref_corner_dict, ref_img, sub_img, sub_corner_dict, objectPoints_dict, point3D_dict, err_dict


    @classmethod
    def __XYT_err(cls, T):
        R = T[0]
        P = np.expand_dims(T[1], axis=1)
        ERR = ArucoStereo.__XYT_err.Y - (np.matmul(R, ArucoStereo.__XYT_err.X) + P)
        return np.sum(ERR * ERR)


    def __calc_T_stereo(objectPoints, points3D):
        ArucoStereo.__XYT_err.X = objectPoints.copy()
        ArucoStereo.__XYT_err.Y = points3D.copy()

        # (1) Instantiate a manifold
        manifold = Product((Rotations(3), Euclidean(3)))

        # (2) Define the cost function (here using autograd.numpy)
        # def cost(T): return np.sum(T[0])+np.sum(T[1])

        problem = Problem(manifold=manifold, cost=ArucoStereo.__XYT_err)
        problem.verbosity = False
        # (3) Instantiate a Pymanopt solver
        solver = TrustRegions(mingradnorm=1e-5, maxtime=STEREO_TIMEOUT)

        # let Pymanopt do the rest
        Xopt = solver.solve(problem)

        return SE3(*Xopt), ArucoStereo.__XYT_err(Xopt)