from pkg.realsense import *
from pkg.kinect import *

from scipy.spatial.transform import Rotation
from pkg.rotation_utils import *
from pkg.utils import *
from pkg.global_config import *
import autograd.numpy as np
from autograd.numpy import linalg

LAST_LINE = np.array([[0, 0, 0, 1]])
R_ZERO = np.identity(3)

from pymanopt.manifolds import Rotations
from pymanopt.manifolds import Euclidean
from pymanopt.manifolds import Product
from pymanopt import Problem
from pymanopt.solvers import TrustRegions

STEREO_TIMEOUT = 10


def calibrate_stereo(aruco_map, dictionary, N_trial=5):
    retval_min = 10000
    T_c21 = None
    kn_config = get_kn_config()
    rs_config = get_rs_config()
    for _ in range(N_trial):
        kn_img = get_kn_image()
        rs_img = get_rs_image()
        # resize
        imageSize = (kn_img.shape[1], kn_img.shape[0])
        rs_camK = rs_config[0] * (kn_img.shape[0] / rs_img.shape[0])
        rs_camK[2, 2] = 1
        rs_config_res = (rs_camK, np.zeros_like(kn_config[1]))
        rs_img_res = cv2.resize(rs_img, imageSize)

        kn_corners, kn_ids, kn_rejectedImgPoints = aruco.detectMarkers(kn_img, dictionary)
        rs_corners, rs_ids, rs_rejectedImgPoints = aruco.detectMarkers(rs_img_res, dictionary)

        kn_corner_dict = {k[0]: v[0] for k, v in zip(kn_ids, kn_corners)}
        rs_corner_dict = {k[0]: v[0] for k, v in zip(rs_ids, rs_corners)}
        keys_all = sorted(set(kn_corner_dict.keys() + rs_corner_dict.keys()))

        objectPoints = []
        imagePoints_kn = []
        imagePoints_rs = []
        for oname, markers in aruco_map.items():
            objectPoints_o = []
            imagePoints_kn_o = []
            imagePoints_rs_o = []
            occluded = 0
            for mk in markers:
                if mk.idx in kn_corner_dict and mk.idx in rs_corner_dict:
                    objectPoints_o.append(mk.corners.copy())
                    imagePoints_kn_o.append(kn_corner_dict[mk.idx].copy())
                    imagePoints_rs_o.append(rs_corner_dict[mk.idx].copy())
                elif mk.idx in kn_corner_dict:
                    occluded += 1

            if ((len(objectPoints_o) == 0) or
                    (len(objectPoints_o) <= 2 and (len(objectPoints_o)+occluded >= 3))):
                continue
            objectPoints.append(np.concatenate(objectPoints_o, axis=0).astype(np.float32))
            imagePoints_kn.append(np.concatenate(imagePoints_kn_o, axis=0))
            imagePoints_rs.append(np.concatenate(imagePoints_rs_o, axis=0))
        retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = \
            cv2.stereoCalibrate(objectPoints,
                                imagePoints_kn,
                                imagePoints_rs,
                                cameraMatrix1=kn_config[0], distCoeffs1=kn_config[1],
                                cameraMatrix2=rs_config_res[0], distCoeffs2=rs_config_res[1],
                                imageSize=imageSize, flags=cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_RATIONAL_MODEL)
        if retval < retval_min:
            retval_min = retval
            T_c21 = SE3(R, T.flatten())
    return T_c21, kn_config, rs_config


def get_object_pose_dict_stereo(T_c21, kn_config, rs_config, aruco_map, dictionary):
    kn_img = get_kn_image()
    rs_img = get_rs_image()

    kn_corners, kn_ids, kn_rejectedImgPoints = aruco.detectMarkers(kn_img, dictionary)
    rs_corners, rs_ids, rs_rejectedImgPoints = aruco.detectMarkers(rs_img, dictionary)

    projMatr1 = np.matmul(kn_config[0], np.identity(4)[:3])
    projMatr2 = np.matmul(rs_config[0], T_c21[:3])

    kn_corner_dict = {k[0]: v[0] for k, v in zip(kn_ids, kn_corners)}
    rs_corner_dict = {k[0]: v[0] for k, v in zip(rs_ids, rs_corners)}
    keys_all = sorted(set(kn_corner_dict.keys() + rs_corner_dict.keys()))

    objectPose_dict = {}
    err_dict = {}
    objectPoints_dict = {}
    point3D_dict = {}
    oname_omit = []
    for oname, markers in aruco_map.items():
        objectPoints_o = []
        imagePoints_kn_o = []
        imagePoints_rs_o = []
        for mk in markers:
            if mk.idx in kn_corner_dict and mk.idx in rs_corner_dict:
                objectPoints_o.append(mk.corners.copy())
                imagePoints_kn_o.append(kn_corner_dict[mk.idx].copy())
                imagePoints_rs_o.append(rs_corner_dict[mk.idx].copy())

        if len(objectPoints_o) == 0:
            oname_omit.append(oname)
            continue
        objectPoints_o = np.concatenate(objectPoints_o, axis=0).astype(np.float32).transpose()
        imagePoints_kn_o = np.concatenate(imagePoints_kn_o, axis=0)
        imagePoints_kn_o = np.reshape(
            cv2.undistortPoints(np.expand_dims(imagePoints_kn_o, axis=1), kn_config[0], kn_config[1], None,
                                kn_config[0]), [-1, 2]).transpose()
        imagePoints_rs_o = np.concatenate(imagePoints_rs_o, axis=0).transpose()
        points4D = cv2.triangulatePoints(projMatr1, projMatr2, imagePoints_kn_o, imagePoints_rs_o)
        point3D_o = points4D[:3] / points4D[3:]

        Tobj, err = calc_T_stereo(objectPoints_o, point3D_o)
        objectPose_dict[oname] = Tobj
        err_dict[oname] = err
        objectPoints_dict[oname] = objectPoints_o
        point3D_dict[oname] = point3D_o
    objectPose_dict_omit, corner_dict = get_object_pose_dict(kn_img, {oname: aruco_map[oname] for oname in oname_omit},
                                                             dictionary, kn_config[0], kn_config[1])
    objectPose_dict.update(objectPose_dict_omit)
    kn_corner_dict.update(corner_dict)
    return objectPose_dict, kn_corner_dict, kn_img, rs_img, rs_corner_dict, objectPoints_dict, point3D_dict, err_dict


def XYT_err(T):
    R = T[0]
    P = np.expand_dims(T[1], axis=1)
    ERR = XYT_err.Y - (np.matmul(R, XYT_err.X) + P)
    return np.sum(ERR * ERR)


def calc_T_stereo(objectPoints, points3D):
    XYT_err.X = objectPoints.copy()
    XYT_err.Y = points3D.copy()

    # (1) Instantiate a manifold
    manifold = Product((Rotations(3), Euclidean(3)))

    # (2) Define the cost function (here using autograd.numpy)
    # def cost(T): return np.sum(T[0])+np.sum(T[1])

    problem = Problem(manifold=manifold, cost=XYT_err)
    problem.verbosity = False
    # (3) Instantiate a Pymanopt solver
    solver = TrustRegions(mingradnorm=1e-5, maxtime=STEREO_TIMEOUT)

    # let Pymanopt do the rest
    Xopt = solver.solve(problem)

    return SE3(*Xopt), XYT_err(Xopt)