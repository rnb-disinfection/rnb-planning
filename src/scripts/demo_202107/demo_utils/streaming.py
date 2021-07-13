import pyrealsense2 as rs
import cv2
import numpy as np
import open3d as o3d

# Directory setting
import os

from demo_config import *

CONFIG_DIR = os.path.join(DEMO_DIR, "configs")
SAVE_DIR = os.path.join(DEMO_DIR, "save_img")
CROP_DIR = os.path.join(DEMO_DIR, "crop_img")
MODEL_DIR = os.path.join(DEMO_DIR, "model_CAD")

DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (720, 1280)


def camera_streaming():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Set stream resolution
    config.enable_stream(rs.stream.depth, DEPTHMAP_SIZE[1], DEPTHMAP_SIZE[0], rs.format.z16, 30)
    # config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, IMAGE_SIZE[1], IMAGE_SIZE[0], rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.visual_preset, 3)
    # Custom = 0, Default = 1, Hand = 2, HighAccuracy = 3, HighDensity = 4, MediumDensity = 5
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.stream.color
    align = rs.align(align_to)

    intrins = []
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # aligned_color_frame = aligned_frames.get_color_frame()
            # depth_frame = aligned_frames.get_depth_frame()

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not aligned_depth_frame or not color_frame:
                continue

            depth_to_color_extrins = depth_frame.profile.get_extrinsics_to(color_frame.profile)
            depth_intrins = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            # depth_intrins = depth_frame.profile.as_video_stream_profile().intrinsics
            # color_intrins = aligned_color_frame.profile.as_video_stream_profile().intrinsics
            intrins = depth_intrins
            # print(depth_intrins)
            # print(color_intrins)
            # print(depth_to_color_extrins)

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                                 interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)

            if (key == 115):
                # If 's' pressed, save color, depth images (far from table)
                cv2.imwrite(SAVE_DIR + '/color.jpg', color_image)
                cv2.imwrite(SAVE_DIR + '/depth.png', depth_image)

            if (key == 99):
                # If 'c' pressed, save color, depth images to refine (near from table)
                cv2.imwrite(SAVE_DIR + '/table.jpg', color_image)
                cv2.imwrite(SAVE_DIR + '/table.png', depth_image)

            if (key == 27):
                # If 'esc' pressed, stop streaming and exit
                cv2.destroyAllWindows()
                break
    except Exception as e:
        print(e)
    finally:

        # Stop streaming
        pipeline.stop()
        return intrins, depth_scale