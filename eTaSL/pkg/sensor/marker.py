import os
import cv2
from cv2 import aruco
from ..utils.rotation_utils import *
from cv2.aruco import DetectorParameters_create

aruco_param = DetectorParameters_create()
aruco_param.adaptiveThreshConstant = 7.0
aruco_param.adaptiveThreshWinSizeMax = 23
aruco_param.adaptiveThreshWinSizeMin = 3
aruco_param.adaptiveThreshWinSizeStep = 10
aruco_param.cornerRefinementMaxIterations = 30
aruco_param.cornerRefinementMinAccuracy = 0.1
aruco_param.cornerRefinementWinSize = 5
aruco_param.doCornerRefinement = True
aruco_param.errorCorrectionRate = 0.6
aruco_param.markerBorderBits = 1
aruco_param.maxErroneousBitsInBorderRate = 0.35
aruco_param.maxMarkerPerimeterRate = 4.0
aruco_param.minCornerDistanceRate = 0.01
aruco_param.minDistanceToBorder = 3
aruco_param.minMarkerDistanceRate = 0.01
aruco_param.minMarkerPerimeterRate = 0.03
aruco_param.minOtsuStdDev = 5.0
aruco_param.perspectiveRemoveIgnoredMarginPerCell = 0.13
aruco_param.perspectiveRemovePixelPerCell = 4
aruco_param.polygonalApproxAccuracyRate = 0.01

from enum import Enum

class DetectType(Enum):
    ENVIRONMENT = 0
    ROBOT = 1
    MOVABLE = 2
    ONLINE = 3

    @classmethod
    def fixed(cls, item):
        return item in [DetectType.ENVIRONMENT, DetectType.ROBOT, DetectType.ONLINE]


class MarkerSet(list):
    def __init__(self, dtype, gtype=None, dims=None, color=(0.6,0.6,0.6,1), soft=False, K_col=None, _list=[]):
        self.dtype, self.gtype = dtype, gtype
        self.dims, self.color = dims, color
        self.soft, self.K_col = soft, K_col
        self += _list

    def get_kwargs(self):
        return dict(gtype=self.gtype, dims=self.dims, color=self.color,
                    fixed=DetectType.fixed(self.dtype), soft=self.soft,
                    online=self.dtype==DetectType.ONLINE, K_col=self.K_col)


# define aruco/charuco - object mapping
# should contain marker & offset
class ObjectMarker:
    def __init__(self, oname, idx, size, point, direction):
        self.oname = oname
        self.idx = idx
        self.Toff = np.identity(4)
        self.set_size(size)
        self.set_offset(point, direction)

    def set_size(self, size):
        self.size = size
        self.__corners = [[-size / 2, -size / 2, 0, 1],
                          [size / 2, -size / 2, 0, 1],
                          [size / 2, size / 2, 0, 1],
                          [-size / 2, size / 2, 0, 1],
                          ]
        self.corners = np.matmul(self.__corners, np.transpose(self.Toff))[:, :3]

    def set_offset(self, point, direction):
        self.point, self.direction = point, direction
        self.Toff = SE3(Rot_rpy(self.direction), self.point)
        self.corners = np.matmul(self.__corners, np.transpose(self.Toff))[:, :3]

def get_object_pose_dict(color_image, aruco_map, dictionary, cameraMatrix, distCoeffs):
    corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, dictionary, parameters=aruco_param)
    if ids is None:
        return {}, {}
    corner_dict = {}
    for idx, corner4 in zip(ids, corners):
        corner_dict[idx[0]] = corner4[0]
    objectPose_dict = {}
    for obj_name, marker_list in aruco_map.items():
        objectPoints = np.zeros((0, 3))
        imagePoints = np.zeros((0, 2))
        for marker in marker_list:
            if marker.idx in corner_dict:
                objectPoints = np.concatenate([objectPoints, marker.corners], axis=0)
                imagePoints = np.concatenate([imagePoints, corner_dict[marker.idx]], axis=0)
        if len(objectPoints) == 0:
            objectPose_dict[obj_name] = SE3(np.identity(3), [0, 0, 10])
            continue
        ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
        R, jac = cv2.Rodrigues(rvec)
        Tobj = SE3(R, tvec.flatten())

        objectPose_dict[obj_name] = Tobj
    return objectPose_dict, corner_dict


def refine_by_depth(depth_image, objectPose_dict, corner_dict, aruco_map, cameraMatrix, distCoeffs):
    keys_sorted = sorted(corner_dict.keys())
    points_sorted = np.concatenate([corner_dict[k] for k in keys_sorted], axis=0)
    points_sorted_undist = np.reshape(
        cv2.undistortPoints(np.expand_dims(points_sorted, axis=1), cameraMatrix, distCoeffs, None, cameraMatrix),
        [-1, 2])
    depth_values = depth_image.flatten()[
        np.ravel_multi_index(points_sorted_undist[:, [1, 0]].transpose().astype(np.int), depth_image.shape)]
    pt3d_sorted = np.matmul(np.linalg.inv(cameraMatrix), np.pad(points_sorted_undist, [[0, 0], [0, 1]], 'constant',
                                                                constant_values=1).transpose()) * depth_values * 1e-3
    pt3d_sorted = np.reshape(pt3d_sorted.transpose(), (-1, 4, 3))
    pt3d_dict = {keys_sorted[i_k]: pt3d_sorted[i_k] for i_k in range(len(keys_sorted))}
    for obj_name, marker_list in aruco_map.items():
        objectPoints = np.zeros((0, 3))
        pt3dPoints = np.zeros((0, 3))
        for marker in marker_list:
            if marker.idx in corner_dict:
                objectPoints = np.concatenate([objectPoints, marker.corners], axis=0)
                pt3dPoints = np.concatenate([pt3dPoints, pt3d_dict[marker.idx]], axis=0)
        if len(objectPoints) == 0:
            continue

        idx_zs = np.where(pt3dPoints[:, 2] > 0.01)[0]
        if len(idx_zs) > 0:
            Tobj = objectPose_dict[obj_name]
            O3 = objectPoints[idx_zs].transpose()
            O_pad = np.pad(O3, [[0, 1], [0, 0]], 'constant', constant_values=1)
            O_pad.astype(np.float16)
            P3_pnp = np.matmul(Tobj, O_pad)[:3]
            P3 = pt3dPoints[idx_zs].transpose()
            zscale = np.mean(P3[2, :] / P3_pnp[2, :])
            objectPose_dict[obj_name][:3, 3] *= zscale


def print_markers(aruco_map, dictionary, px_size=800, dir_img="./markers"):
    try:
        os.mkdir(dir_img)
    except:
        pass
    for mk in sorted(sum(aruco_map.values(), []), key=lambda x: x.idx):
        img = aruco.drawMarker(dictionary, mk.idx, px_size)
        cv2.imwrite("{}/marker_{}.png".format(dir_img, mk.idx), img)


def draw_objects(color_image, aruco_map, objectPose_dict, corner_dict, cameraMatrix, distCoeffs, axis_len=0.1):
    color_image_out = color_image.copy()
    text_scale = float(color_image_out.shape[1]) / 2000
    font_thickness = int(round(text_scale * 2))
    for k, Tco in objectPose_dict.items():
        # draw object location
        marker_o = aruco_map[k]
        rvec, _ = cv2.Rodrigues(Tco[:3, :3])
        tvec = Tco[:3, 3]
        aruco.drawAxis(color_image_out, cameraMatrix, distCoeffs, rvec, tvec, axis_len)
        cco = np.matmul(cameraMatrix, tvec)
        cco_px = (cco[:2] / cco[2:]).flatten().astype(np.int64)
        cv2.putText(color_image_out, str(k), tuple(cco_px + np.array([40, -40])), cv2.FONT_HERSHEY_SIMPLEX, text_scale,
                    (255, 0, 0), font_thickness)

        # draw model
        marker_o_list = aruco_map[k]
        for marker_o in marker_o_list:
            Toff_corner = SE3(marker_o.Toff[:3, :3], marker_o.corners[0])
            Tcc = np.matmul(Tco, Toff_corner)
            rvec, _ = cv2.Rodrigues(Tcc[:3, :3])
            tvec = Tcc[:3, 3]
            aruco.drawAxis(color_image_out, cameraMatrix, distCoeffs, rvec, tvec, marker_o.size)
    # draw corners
    markerIds = np.reshape(list(sorted(corner_dict.keys())), (-1, 1))
    markerCorners = [np.expand_dims(corner_dict[k[0]], axis=0) for k in markerIds]
    color_image_out = aruco.drawDetectedMarkers(color_image_out, markerCorners, markerIds)
    return color_image_out


def get_T_rel(coord_from, coord_to, objectPose_dict):
    return np.matmul(SE3_inv(objectPose_dict[coord_from]), objectPose_dict[coord_to])


def get_put_dir(Robj, dir_vec_dict):
    downvec = np.matmul(np.transpose(Robj), [[0], [0], [-1]])
    dir_match_dict = {k: np.dot(v, downvec) for k, v in dir_vec_dict.items()}
    key_match = sorted(dir_match_dict.keys(), key=lambda k: dir_match_dict[k])[-1]
    return key_match

