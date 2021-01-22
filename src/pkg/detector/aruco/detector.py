import os
import cv2
from cv2 import aruco
from ...utils.rotation_utils import *
from ..detector_interface import DetectionLevel
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


##
# @class    ObjectMarker
# @brief    An aruco marker definition and offset from parent object
class ObjectMarker:
    ##
    # @param oname name of object
    # @param idx   aruco marker index
    # @param size  size of marker in m scale
    # @param point marker's center offset from object
    # @param direction marker's orientation respect to object in rpy
    def __init__(self, oname, idx, size, point, direction):
        self.oname = oname
        self.idx = idx
        self.Toff = np.identity(4)
        self.__set_size(size)
        self.set_offset(point, direction)

    def __set_size(self, size):
        self.size = size
        self.__corners = [[-size / 2, -size / 2, 0, 1],
                          [size / 2, -size / 2, 0, 1],
                          [size / 2, size / 2, 0, 1],
                          [-size / 2, size / 2, 0, 1],
                          ]
        self.corners = np.matmul(self.__corners, np.transpose(self.Toff))[:, :3]

    ##
    # @brief set marker offset from parent object
    # @param point marker's center offset from object
    # @param direction marker's orientation respect to object in rpy
    def set_offset(self, point, direction):
        self.point, self.direction = point, direction
        self.Toff = SE3(Rot_rpy(self.direction), self.point)
        self.corners = np.matmul(self.__corners, np.transpose(self.Toff))[:, :3]


##
# @class MarkerSet
# @brief a set of ObjectMarker that are indicates an object
class MarkerSet(list):
    ##
    # @param name  name of object
    # @param dlevel detection level
    # @param gtype geometry type
    # @param dims  geometry dimensions
    # @param color geometry visualization color
    # @param soft  flag for soft collision boundary (eTaSL)
    # @param K_col K value for soft collision boundary (eTaSL)
    # @param _list list of ObjectMarker
    def __init__(self, name, dlevel, gtype=None, dims=None, color=None, soft=False, K_col=None, _list=None):
        if color is None:
            color = (0.6,0.6,0.6,1)
        if _list is None:
            _list = []
        self.name, self.dlevel, self.gtype = name, dlevel, gtype
        self.dims, self.color = dims, color
        self.soft, self.K_col = soft, K_col
        self += _list

    def get_geometry_kwargs(self):
        ##
        # @brief get kwargs to create geometry item
        return dict(gtype=self.gtype, dims=self.dims, color=self.color,
                    fixed=DetectionLevel.is_fixed(self.dlevel), soft=self.soft,
                    online=self.dlevel==DetectionLevel.ONLINE, K_col=self.K_col)


##
# @class ArucoMap
# @brief dictionary of MarkerSet for a scene
class ArucoMap(dict):
    def __init__(self, dictionary, _dict):
        self.dictionary = dictionary
        for k, v in _dict.items():
            self[k] = v

    ##
    # @brief change marker names
    def change_marker_name(self, name_old, name_new):
        atem = self[name_old]
        atem.oname = name_new
        self[name_new].append(atem)
        del self[name_old][atem]

    ##
    # @brief detect objects
    def get_object_pose_dict(self, color_image, cameraMatrix, distCoeffs, name_mask=None):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, self.dictionary, parameters=aruco_param)
        if ids is None:
            return {}, {}
        corner_dict = {}
        for idx, corner4 in zip(ids, corners):
            corner_dict[idx[0]] = corner4[0]
        objectPose_dict = {}
        items = self.items() if name_mask is None else {k:self[k] for k in name_mask}.items()
        for obj_name, marker_list in items:
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

    ##
    # @brief print markers to files
    def print_markers(self, px_size=800, dir_img="./markers"):
        try:
            os.mkdir(dir_img)
        except:
            pass
        for mk in sorted(sum(self.values(), []), key=lambda x: x.idx):
            img = aruco.drawMarker(self.dictionary, mk.idx, px_size)
            cv2.imwrite("{}/marker_{}.png".format(dir_img, mk.idx), img)

    ##
    # @brief draw objects and markers
    def draw_objects(self, color_image, objectPose_dict, corner_dict, cameraMatrix, distCoeffs, axis_len=0.1):
        color_image_out = color_image.copy()
        text_scale = float(color_image_out.shape[1]) / 2000
        font_thickness = int(round(text_scale * 2))
        for k, Tco in objectPose_dict.items():
            # draw object location
            marker_o = self[k]
            rvec, _ = cv2.Rodrigues(Tco[:3, :3])
            tvec = Tco[:3, 3]
            aruco.drawAxis(color_image_out, cameraMatrix, distCoeffs, rvec, tvec, axis_len)
            cco = np.matmul(cameraMatrix, tvec)
            cco_px = (cco[:2] / cco[2:]).flatten().astype(np.int64)
            cv2.putText(color_image_out, str(k), tuple(cco_px + np.array([40, -40])), cv2.FONT_HERSHEY_SIMPLEX, text_scale,
                        (255, 0, 0), font_thickness)

            # draw model
            marker_o_list = self[k]
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