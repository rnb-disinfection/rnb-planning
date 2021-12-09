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
    def __init__(self, serial=None):
        self.connected = False
        self.device = get_device(serial)
        self.resolution_fps_dict = get_resolution_fps_dict(device=self.device)

    ##
    # @brief   initialize RealSense
    # @param resolution_color, resolution_depth
    def initialize(self, resolution_color=None, resolution_depth=None):
        if self.connected:
            return self.pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        if resolution_color is None:
            resolution_color, fps_color = sorted(self.resolution_fps_dict['color'])[-1]
        else:
            fps_list = [fps_ for resolution_, fps_ in self.resolution_fps_dict['color'] if resolution_==resolution_color]
            assert len(fps_list)>0, "Not available color resolution! Availables are: \n{}".format(
                self.self.resolution_fps_dict['color'])
            fps_color = fps_list[-1]
        if resolution_depth is None:
            resolution_depth, fps_depth = sorted(self.resolution_fps_dict['depth'])[-1]
        else:
            fps_list = [fps_ for resolution_, fps_ in self.resolution_fps_dict['depth'] if resolution_==resolution_depth]
            assert len(fps_list)>0, "Not available depth resolution! Availables are: \n{}".format(
                self.self.resolution_fps_dict['depth'])
            fps_depth = fps_list[-1]
        config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps_depth)
        config.enable_stream(rs.stream.color, resolution_color[0], resolution_color[1], rs.format.bgr8, fps_color)

        # Start streaming
        print("Start streaming")
        self.pipeline.start(config)
        self.connected = True
        return self.pipeline

    ##
    # @brief   get camera configuration
    # @return  cameraMatrix 3x3 camera matrix in pixel units,
    # @return  distCoeffs distortion coefficients, 5~14 float array
    # @return  depth_scale scaling constant to be multiplied to depthmap(int) to get m scale values
    def get_config(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                 [0, color_intrinsics.fy, color_intrinsics.ppy],
                                 [0, 0, 1]])
        distCoeffs = np.array(color_intrinsics.coeffs)
        profile = self.pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        return cameraMatrix, distCoeffs, depth_scale

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


def get_device_serials():
    ctx = rs.context()
    devices = ctx.query_devices()
    serial_list = []
    for dev in devices:
        serial_list.append(dev.get_info(rs.camera_info.serial_number))
    return serial_list


def get_device(serial=None):
    ctx = rs.context()
    devices = ctx.query_devices()
    assert len(devices) > 0, "No connected realsense"
    if serial is None:
        if len(devices) == 1:
            return devices[0]
        else:
            raise (RuntimeError("Serial number should be given when multiple devices are connected - use get_device_serials() to get serial numbers"))
    for dev in devices:
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return dev
        raise (RuntimeError(
            "Invalid serial number given: {} not in valid serials: {}".format(serial, get_device_serials())))


def get_resolution_fps_dict(device=None, serial=None):
    if device is None:
        device = get_device(serial)
    sensors = list(device.query_sensors())
    resfps_list_dict = {}
    for s in sensors:
        res_fps_dict = {}
        stream_name = None
        for pf in s.get_stream_profiles():
            video = pf.as_video_stream_profile()
            res = (video.width(), video.height())
            fps = pf.fps()
            stream_name = str(pf.stream_name()).lower()
            if stream_name not in ["color", "depth"]:
                continue
            if res in res_fps_dict and fps <= res_fps_dict[res]:
                continue
            res_fps_dict[res] = fps
        resfps_list_dict[stream_name] = sorted([(res, fps) for res, fps in res_fps_dict.items()])
    return resfps_list_dict