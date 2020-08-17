import sys
import os
sys.path.insert(1, os.path.join(os.environ["TF_GMT_ETASL_DIR"],'pyKinectAzure/pyKinectAzure/'))

import numpy as np
from pyKinectAzure import pyKinectAzure, _k4a
import cv2

from cv2 import aruco
from pkg.utils import *
from pkg.rotation_utils import *

# Path to the module
# TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
MODULEPATH = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
# for windows 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
# under linux please use r'/usr/lib/x86_64-linux-gnu/libk4a.so'

def init_kinect():
    # Initialize the library with the path containing the module
    pyK4A = pyKinectAzure(MODULEPATH)

    # Open device
    pyK4A.device_open()

    # Modify camera configuration
    device_config = pyK4A.config
    device_config.color_format = _k4a.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
    print(device_config)

    # Start cameras using modified configuration
    pyK4A.device_start_cameras(device_config)
    return pyK4A

def get_image_set(pyK4A):
    # Get capture
    pyK4A.device_get_capture()

    # Get the depth image from the capture
    depth_image_handle = pyK4A.capture_get_depth_image()

    # Get the color image from the capture
    color_image_handle = pyK4A.capture_get_color_image()

    # Check the image has been read correctly
    if depth_image_handle and color_image_handle:

        # Read and convert the image data to numpy array:
        color_image = pyK4A.image_convert_to_numpy(color_image_handle)[:,:,:3]

        # Transform the depth image to the color format
        transformed_depth_image = pyK4A.transform_depth_to_color(depth_image_handle,color_image_handle)

    #     # Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
    #     transformed_depth_color_image = cv2.applyColorMap(np.round(transformed_depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

    #     # Add the depth image over the color image:
    #     combined_image = cv2.addWeighted(color_image,0.7,transformed_depth_color_image,0.3,0)

    #     # Plot the image
    #     cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
    #     cv2.imshow('Colorized Depth Image',combined_image)
    #     k = cv2.waitKey(25)

        pyK4A.image_release(depth_image_handle)
        pyK4A.image_release(color_image_handle)
        pyK4A.capture_release()
        return color_image, transformed_depth_image
    else:
        pyK4A.capture_release()
        return None, None



def get_calibration(pyK4A):
    calibration = _k4a.k4a_calibration_t()
    # Get the camera calibration
    pyK4A.device_get_calibration(pyK4A.config.depth_mode,pyK4A.config.color_resolution,calibration)
    cal_param = calibration.color_camera_calibration.intrinsics.parameters.param
    cameraMatrix = np.diag([cal_param.fx, cal_param.fx, 1])
    cameraMatrix[:2,2] = [cal_param.cx, cal_param.cy]
    distCoeffs = np.array([cal_param.k1, cal_param.k2, cal_param.p1, cal_param.p2, cal_param.k3, cal_param.k4, cal_param.k5, cal_param.k6])
    return cameraMatrix, distCoeffs

def disconnect_kinect(pyK4A):
    pyK4A.device_stop_cameras()
    pyK4A.device_close()
    
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
    corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, dictionary)
    corner_dict = {}
    for idx, corner4 in zip(ids, corners):
        corner_dict[idx[0]] = corner4[0]
        print("{}:{}".format(idx, corner4))
    objectPose_dict = {}
    objectrtvec_dict = {}
    for obj_name, marker_list in aruco_map.items():
        objectPoints = np.zeros((0,3))
        imagePoints = np.zeros((0,2))
        for marker in marker_list:
            if marker.idx in corner_dict:
                objectPoints = np.concatenate([objectPoints, marker.corners], axis=0)
                imagePoints = np.concatenate([imagePoints, corner_dict[marker.idx]], axis=0)
        ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs)
        R, jac = cv2.Rodrigues(rvec)
        Tobj = SE3(R, tvec.flatten())
        objectPose_dict[obj_name] = Tobj
    #     objectrtvec_dict[obj_name] = (rvec, tvec)
    #     axis_len = 0.05
    #     aruco.drawAxis(color_image, cameraMatrix, distCoeffs, rvec,tvec, axis_len)
    return objectPose_dict