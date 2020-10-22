import sys
import os
sys.path.insert(1, os.path.join(os.environ["TAMP_ETASL_DIR"],'pyKinectAzure/pyKinectAzure/'))

import numpy as np
from pyKinectAzure import pyKinectAzure, _k4a
import cv2

from cv2 import aruco
from .utils import *
from .rotation_utils import *
from .constants import *
from scipy.spatial.transform import Rotation
import time

# Path to the module
# TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
MODULEPATH = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
# for windows 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
# under linux please use r'/usr/lib/x86_64-linux-gnu/libk4a.so'

pyK4A = None
color_image_handle = None

def init_kn():
    # Initialize the library with the path containing the module
    global pyK4A
    if pyK4A is not None:
        disconnect_kn()
    pyK4A = pyKinectAzure(MODULEPATH)

    # Open device
    pyK4A.device_open()

    # Modify camera configuration
    device_config = pyK4A.config
    device_config.color_format = _k4a.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_2160P
    device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
    print(device_config)

    # Start cameras using modified configuration
    pyK4A.device_start_cameras(device_config)

def get_kn_image(h_iter_duration=0.01, h_iter_max=100):
    count = 0
    color_image_handle = None
    while not color_image_handle and count < h_iter_max:
        # Get capture
        pyK4A.device_get_capture()
        # Get the depth image from the capture
        # depth_image_handle = pyK4A.capture_get_depth_image()
        # Get the color image from the capture
        color_image_handle = pyK4A.capture_get_color_image()
        time.sleep(h_iter_duration)
        count += 1

    # Check the image has been read correctly
    if color_image_handle : #and depth_image_handle:

        # Read and convert the image data to numpy array:
        color_image = pyK4A.image_convert_to_numpy(color_image_handle)[:,:,:3]
        color_image = color_image.copy()

        # Transform the depth image to the color format
#         transformed_depth_image = pyK4A.transform_depth_to_color(depth_image_handle,color_image_handle)

    #     # Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
    #     transformed_depth_color_image = cv2.applyColorMap(np.round(transformed_depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

    #     # Add the depth image over the color image:
    #     combined_image = cv2.addWeighted(color_image,0.7,transformed_depth_color_image,0.3,0)

    #     # Plot the image
    #     cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
    #     cv2.imshow('Colorized Depth Image',combined_image)
    #     k = cv2.waitKey(25)

#         pyK4A.image_release(depth_image_handle)
        pyK4A.image_release(color_image_handle)
        pyK4A.capture_release()
        return color_image#, transformed_depth_image
    else:
        print("None image handle: %i"%(count))
        pyK4A.capture_release()
        return None


def get_kn_image_depth(h_iter_duration=0.01, h_iter_max=100):
    count = 0
    color_image_handle = None
    while not color_image_handle and count < h_iter_max:
        # Get capture
        pyK4A.device_get_capture()
        # Get the depth image from the capture
        depth_image_handle = pyK4A.capture_get_depth_image()
        # Get the color image from the capture
        color_image_handle = pyK4A.capture_get_color_image()
        time.sleep(h_iter_duration)
        count += 1

    # Check the image has been read correctly
    if color_image_handle : #and depth_image_handle:

        # Read and convert the image data to numpy array:
        color_image = pyK4A.image_convert_to_numpy(color_image_handle)[:,:,:3]
        color_image = color_image.copy()

        # Transform the depth image to the color format
        transformed_depth_image = pyK4A.transform_depth_to_color(depth_image_handle,color_image_handle)

        # Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
#         transformed_depth_color_image = cv2.applyColorMap(np.round(transformed_depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

    #     # Add the depth image over the color image:
    #     combined_image = cv2.addWeighted(color_image,0.7,transformed_depth_color_image,0.3,0)

    #     # Plot the image
    #     cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
    #     cv2.imshow('Colorized Depth Image',combined_image)
    #     k = cv2.waitKey(25)

#         pyK4A.image_release(depth_image_handle)
        pyK4A.image_release(color_image_handle)
        pyK4A.capture_release()
        return color_image, transformed_depth_image
    else:
        print("None image handle: %i"%(count))
        pyK4A.capture_release()
        return None, None


def get_kn_config():
    calibration = _k4a.k4a_calibration_t()
    # Get the camera calibration
    pyK4A.device_get_calibration(pyK4A.config.depth_mode,pyK4A.config.color_resolution,calibration)
    cal_param = calibration.color_camera_calibration.intrinsics.parameters.param
    cameraMatrix = np.diag([cal_param.fx, cal_param.fx, 1])
    cameraMatrix[:2,2] = [cal_param.cx, cal_param.cy]
    distCoeffs = np.array([cal_param.k1, cal_param.k2, cal_param.p1, cal_param.p2, cal_param.k3, cal_param.k4, cal_param.k5, cal_param.k6])
    return cameraMatrix, distCoeffs

def disconnect_kn():
    global pyK4A, color_image_handle
    pyK4A.device_stop_cameras()
    pyK4A.device_close()
    pyK4A = None
    color_image_handle = None
    
# define aruco/charuco - object mapping
# should contain marker & offset
class ObjectMarker:
    def __init__(self, idx, size, Toff):
        self.idx = idx
        self.size = size
        self.Toff = Toff # zero-offset orienation: z-axis inward, y-axis down-ward,  x-axis right-ward
        self.__corners = [[-size/2, -size/2, 0, 1],
                          [size/2, -size/2, 0, 1],
                          [size/2, size/2, 0, 1],
                          [-size/2, size/2, 0, 1],
                         ]
        self.corners = np.matmul(self.__corners,np.transpose(Toff))[:, :3]
        
def get_object_pose_dict(color_image, aruco_map, dictionary, cameraMatrix, distCoeffs):
    corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, dictionary, parameters=aruco_param)
    if ids is None:
        return {}, {}
    corner_dict = {}
    for idx, corner4 in zip(ids, corners):
        corner_dict[idx[0]] = corner4[0]
    objectPose_dict = {}
    for obj_name, marker_list in aruco_map.items():
        objectPoints = np.zeros((0,3))
        imagePoints = np.zeros((0,2))
        for marker in marker_list:
            if marker.idx in corner_dict:
                objectPoints = np.concatenate([objectPoints, marker.corners], axis=0)
                imagePoints = np.concatenate([imagePoints, corner_dict[marker.idx]], axis=0)
        if len(objectPoints) == 0:
            objectPose_dict[obj_name] = SE3(np.identity(3), [0,0,10])
            continue
        ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
        R, jac = cv2.Rodrigues(rvec)
        Tobj = SE3(R, tvec.flatten())

        objectPose_dict[obj_name] = Tobj
    return objectPose_dict, corner_dict

def refine_by_depth(depth_image, objectPose_dict, corner_dict, aruco_map, cameraMatrix, distCoeffs):
    keys_sorted = sorted(corner_dict.keys())
    points_sorted = np.concatenate([corner_dict[k] for k in keys_sorted], axis=0)
    points_sorted_undist = np.reshape(cv2.undistortPoints(np.expand_dims(points_sorted, axis=1), cameraMatrix, distCoeffs, None, cameraMatrix), [-1,2])
    depth_values = depth_image.flatten()[np.ravel_multi_index(points_sorted_undist[:,[1,0]].transpose().astype(np.int), depth_image.shape)]
    pt3d_sorted = np.matmul(np.linalg.inv(cameraMatrix), np.pad(points_sorted_undist, [[0,0],[0,1]], 'constant', constant_values=1).transpose())*depth_values*1e-3
    pt3d_sorted = np.reshape(pt3d_sorted.transpose(), (-1,4,3))
    pt3d_dict = {keys_sorted[i_k]:pt3d_sorted[i_k] for i_k  in range(len(keys_sorted))}
    for obj_name, marker_list in aruco_map.items():
        objectPoints = np.zeros((0,3))
        pt3dPoints = np.zeros((0,3))
        for marker in marker_list:
            if marker.idx in corner_dict:
                objectPoints = np.concatenate([objectPoints, marker.corners], axis=0)
                pt3dPoints = np.concatenate([pt3dPoints, pt3d_dict[marker.idx]], axis=0)
        if len(objectPoints) == 0:
            continue

        idx_zs = np.where(pt3dPoints[:,2]>0.01)[0]
        if len(idx_zs)>0:
            Tobj = objectPose_dict[obj_name]
            O3 = objectPoints[idx_zs].transpose()
            O_pad = np.pad(O3, [[0,1],[0,0]], 'constant', constant_values=1)
            O_pad.astype(np.float16)
            P3_pnp = np.matmul(Tobj, O_pad)[:3]
            P3 = pt3dPoints[idx_zs].transpose()
            zscale = np.mean(P3[2,:]/P3_pnp[2,:])
            objectPose_dict[obj_name][:3,3] *= zscale



def print_markers(aruco_map, dictionary, px_size=800, dir_img="./markers"):
    try: os.mkdir(dir_img)
    except: pass
    for mk in sorted(sum(aruco_map.values(), []), key=lambda x: x.idx):
        img = aruco.drawMarker(dictionary,mk.idx, px_size)
        cv2.imwrite("{}/marker_{}.png".format(dir_img, mk.idx), img)
        
        
def draw_objects(color_image, aruco_map, objectPose_dict, corner_dict, cameraMatrix, distCoeffs, axis_len=0.1):
    color_image_out = color_image.copy()
    text_scale = float(color_image_out.shape[1])/2000
    font_thickness = int(round(text_scale*2))
    for k,Tco in objectPose_dict.items():
        # draw object location
        marker_o = aruco_map[k]
        rvec,_ = cv2.Rodrigues(Tco[:3,:3])
        tvec = Tco[:3,3]
        aruco.drawAxis(color_image_out, cameraMatrix, distCoeffs, rvec,tvec, axis_len)
        cco = np.matmul(cameraMatrix, tvec)
        cco_px = (cco[:2]/cco[2:]).flatten().astype(np.int64)
        cv2.putText(color_image_out, str(k), tuple(cco_px+np.array([40, -40])), cv2.FONT_HERSHEY_SIMPLEX, text_scale, (255,0,0), font_thickness)

        # draw model
        marker_o_list = aruco_map[k]
        for marker_o in marker_o_list:
            Toff_corner = SE3(marker_o.Toff[:3,:3], marker_o.corners[0])
            Tcc = np.matmul(Tco, Toff_corner)
            rvec,_ = cv2.Rodrigues(Tcc[:3,:3])
            tvec = Tcc[:3,3]
            aruco.drawAxis(color_image_out, cameraMatrix, distCoeffs, rvec,tvec, marker_o.size)
    # draw corners
    markerIds = np.reshape(list(sorted(corner_dict.keys())), (-1,1))
    markerCorners = [np.expand_dims(corner_dict[k[0]], axis=0) for k in markerIds]
    color_image_out = aruco.drawDetectedMarkers(color_image_out, markerCorners, markerIds)
    return color_image_out

def get_T_rel(coord_from, coord_to, objectPose_dict):
    return np.matmul(SE3_inv(objectPose_dict[coord_from]), objectPose_dict[coord_to])

def get_put_dir(Robj, dir_vec_dict):
    downvec = np.matmul(np.transpose(Robj), [[0],[0],[-1]])
    dir_match_dict = {k: np.dot(v, downvec) for k, v in dir_vec_dict.items()}
    key_match = sorted(dir_match_dict.keys(), key=lambda k: dir_match_dict[k])[-1]
    return key_match
