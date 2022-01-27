from concurrent import futures
import logging
import math
import time
import grpc
import RemoteCam_pb2
import RemoteCam_pb2_grpc
from threading import Thread
import cv2
import numpy as np
import pyrealsense2 as rs


PORT_CAM = 10509
DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (480, 640)

##
# @brief camera streaming from remote computer
class RemoteCamServicer(RemoteCam_pb2_grpc.RemoteCamProtoServicer):
    def __init__(self): # start camera streaming
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Set stream resolution
        self.config.enable_stream(rs.stream.depth, DEPTHMAP_SIZE[1], DEPTHMAP_SIZE[0], rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, IMAGE_SIZE[1], IMAGE_SIZE[0], rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_sensor.set_option(rs.option.visual_preset, 3)
        # Custom = 0, Default = 1, Hand = 2, HighAccuracy = 3, HighDensity = 4, MediumDensity = 5
        self.depth_sensor.set_option(rs.option.post_processing_sharpening, 3)
        self.depth_sensor.set_option(rs.option.receiver_gain, 13)
        self.depth_sensor.set_option(rs.option.noise_filtering, 4)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        print("=======Start camera streaming=======")


    def GetImage(self, request, context):
        request_id = request.request_id

        frame = self.pipeline.wait_for_frames()
        frames = self.align.process(frame)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        # color = cv2.imread("test-container.png")         # sample image
        # depth = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)  # fake depth map with gray image
        return RemoteCam_pb2.GetImageResponse(response_id=request_id,
                                              width=color.shape[1],height=color.shape[0],
                                              color=color.flatten(),
                                              depth=depth.flatten())
    def GetConfig(self, request, context):
        request_id = request.request_id

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                 [0, color_intrinsics.fy, color_intrinsics.ppy],
                                 [0, 0, 1]])
        distCoeffs = np.array(color_intrinsics.coeffs)
        depth_scale = self.depth_sensor.get_depth_scale()
        return RemoteCam_pb2.GetConfigResponse(response_id=request_id,
                                               depth_scale=depth_scale,
                                               camera_matrix=cameraMatrix.flatten(),
                                               dist_coeffs=distCoeffs.flatten())



##
# @brief start grpc server
def serve(servicer, host='[::]'):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    RemoteCam_pb2_grpc.add_RemoteCamProtoServicer_to_server(
        servicer, server)
    server.add_insecure_port('{}:{}'.format(host, PORT_CAM))
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':
    logging.basicConfig()
    servicer = RemoteCamServicer()
    serve(servicer)