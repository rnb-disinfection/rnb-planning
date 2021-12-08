import pyrealsense2 as rs
import numpy as np
from .camera_interface import CameraInterface

##
# @class    RealSense
# @brief    realsense camera interface
#           Must call initialize/disconnect before/after usage.
#           If "Couldn't resolve requests" error is raised, check if it is connected with USB3.
#           You can check by running realsense-viewer from the terminal.
class RealSense(CameraInterface):
    def __init__(self):
        self.connected = False

    ##
    # @brief   initialize RealSense
    def initialize(self):
        if self.connected:
            return self.pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        print("Start streaming")
        self.pipeline.start(config)
        self.connected = True
        return self.pipeline

    ##
    # @brief   get camera configuration
    # @return  cameraMatrix 3x3 camera matrix in pixel units,
    # @return  distCoeffs distortion coefficients, 5~14 float array
    def get_config(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                 [0, color_intrinsics.fy, color_intrinsics.ppy],
                                 [0, 0, 1]])
        distCoeffs = np.array(color_intrinsics.coeffs)
        return cameraMatrix, distCoeffs

    ##
    # @brief   get RGB image
    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    ##
    # @brief   get RGB image and depthmap
    def get_image_depthmap(self):
        frames = self.pipeline.wait_for_frames()

        # Align depth to color
        align_to = rs.stream.color
        align = rs.align(align_to)
        frames = align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image

    ##
    # @brief   disconnect realsense
    def disconnect(self):
        if not self.connected:
            raise(RuntimeError("Tried to disconnect non-initialized RealSense"))
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception as e:
                print(e)
            self.connected = False