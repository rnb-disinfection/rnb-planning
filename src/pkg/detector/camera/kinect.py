import sys
import os
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"],'third-party/pyKinectAzure/pyKinectAzure/'))

from pyKinectAzure import pyKinectAzure, _k4a
import numpy as np
import time
from .camera_interface import CameraInterface


##
# @class    Kinect
# @brief    Kinect camera interface.
#           Must call initialize/disconnect before/after usage
class Kinect(CameraInterface):
    # Path to the module
    # TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
    MODULEPATH = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
    # for windows 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
    # under linux please use r'/usr/lib/x86_64-linux-gnu/libk4a.so'

    pyK4A = None

    ##
    # @brief   initialize kinect
    def initialize(self):
        # Initialize the library with the path containing the module
        if Kinect.pyK4A is not None:
            self.disconnect()
        Kinect.pyK4A = pyKinectAzure(Kinect.MODULEPATH)

        # Open device
        Kinect.pyK4A.device_open()

        # Modify camera configuration
        device_config = Kinect.pyK4A.config
        device_config.color_format = _k4a.K4A_IMAGE_FORMAT_COLOR_BGRA32
        device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_2160P
        device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
        print(device_config)

        # Start cameras using modified configuration
        Kinect.pyK4A.device_start_cameras(device_config)

    ##
    # @brief    get camera configuration
    # @return   cameraMatrix 3x3 camera matrix in pixel units,
    # @return   distCoeffs distortion coefficients, 5~14 float array
    # @return   depth_scale scaling constant to be multiplied to depthmap(int) to get m scale values
    def get_config(self):
        calibration = _k4a.k4a_calibration_t()
        # Get the camera calibration
        Kinect.pyK4A.device_get_calibration(Kinect.pyK4A.config.depth_mode,Kinect.pyK4A.config.color_resolution,calibration)
        cal_param = calibration.color_camera_calibration.intrinsics.parameters.param
        cameraMatrix = np.diag([cal_param.fx, cal_param.fx, 1])
        cameraMatrix[:2,2] = [cal_param.cx, cal_param.cy]
        distCoeffs = np.array([cal_param.k1, cal_param.k2, cal_param.p1, cal_param.p2, cal_param.k3, cal_param.k4, cal_param.k5, cal_param.k6])
        depth_scale = 1e-3
        return cameraMatrix, distCoeffs, depth_scale

    ##
    # @brief   get RGB image
    def get_image(self, h_iter_duration=0.01, h_iter_max=100):
        count = 0
        color_image_handle = None
        while not color_image_handle and count < h_iter_max:
            # Get capture
            Kinect.pyK4A.device_get_capture()
            # Get the depth image from the capture
            # depth_image_handle = pyK4A.capture_get_depth_image()
            # Get the color image from the capture
            color_image_handle = Kinect.pyK4A.capture_get_color_image()
            time.sleep(h_iter_duration)
            count += 1

        # Check the image has been read correctly
        if color_image_handle : #and depth_image_handle:

            # Read and convert the image data to numpy array:
            color_image = Kinect.pyK4A.image_convert_to_numpy(color_image_handle)[:,:,:3]
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
            Kinect.pyK4A.image_release(color_image_handle)
            Kinect.pyK4A.capture_release()
            return color_image#, transformed_depth_image
        else:
            print("None image handle: %i"%(count))
            Kinect.pyK4A.capture_release()
            return None


    ##
    # @brief   get RGB image and depthmap
    def get_image_depthmap(self, h_iter_duration=0.01, h_iter_max=100):
        count = 0
        color_image_handle = None
        while not color_image_handle and count < h_iter_max:
            # Get capture
            Kinect.pyK4A.device_get_capture()
            # Get the depth image from the capture
            depth_image_handle = Kinect.pyK4A.capture_get_depth_image()
            # Get the color image from the capture
            color_image_handle = Kinect.pyK4A.capture_get_color_image()
            time.sleep(h_iter_duration)
            count += 1

        # Check the image has been read correctly
        if color_image_handle : #and depth_image_handle:

            # Read and convert the image data to numpy array:
            color_image = Kinect.pyK4A.image_convert_to_numpy(color_image_handle)[:,:,:3]
            color_image = color_image.copy()

            # Transform the depth image to the color format
            transformed_depth_image = Kinect.pyK4A.transform_depth_to_color(depth_image_handle,color_image_handle)

            # Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
    #         transformed_depth_color_image = cv2.applyColorMap(np.round(transformed_depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

        #     # Add the depth image over the color image:
        #     combined_image = cv2.addWeighted(color_image,0.7,transformed_depth_color_image,0.3,0)

        #     # Plot the image
        #     cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
        #     cv2.imshow('Colorized Depth Image',combined_image)
        #     k = cv2.waitKey(25)

    #         pyK4A.image_release(depth_image_handle)
            Kinect.pyK4A.image_release(color_image_handle)
            Kinect.pyK4A.capture_release()
            return color_image, transformed_depth_image
        else:
            print("None image handle: %i"%(count))
            Kinect.pyK4A.capture_release()
            return None, None

    ##
    # @brief   disconnect kinect
    def disconnect(self):
        if Kinect.pyK4A is not None:
            Kinect.pyK4A.device_stop_cameras()
            Kinect.pyK4A.device_close()
            Kinect.pyK4A = None


