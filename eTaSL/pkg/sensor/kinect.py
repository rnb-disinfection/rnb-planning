import sys
import os
sys.path.insert(1, os.path.join(os.environ["TAMP_ETASL_DIR"],'pyKinectAzure/pyKinectAzure/'))

from pyKinectAzure import pyKinectAzure, _k4a
from ..utils.rotation_utils import *
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
    

