import pyrealsense2 as rs
import numpy as np

pipeline = None

def init_rs():
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
    print("Start streaming")
    pipeline.start(config)
    return pipeline

def get_rs_image():
    global pipeline
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def get_rs_config():
    global pipeline
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                             [0, color_intrinsics.fy, color_intrinsics.ppy],
                             [0, 0, 1]])
    distCoeffs = np.array(color_intrinsics.coeffs)
    return cameraMatrix, distCoeffs

def disconnect_rs():
    global pipeline
    if pipeline is not None:
        try:
            pipeline.stop()
        except Exception as e:
            print(e)