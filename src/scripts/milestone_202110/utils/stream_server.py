import socket
import cv2
import numpy as np
from queue import Queue
from _thread import *
import pyrealsense2 as rs
import json
import socket
import argparse

PORT = 8580
DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (720, 1280)

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


##
# @brief send color, depth, intrinsic, depth scale when client sends signal
# @remark   send dictionary {'color_len', 'depth_len', 'intrins', 'depth_scale'} at first signal \n
#           'color_len', 'depth_len' are cv2 encoded image string length to be sent \n
#           send color image and depth image as concatenated cv2 encoded string
def threaded(socket, addr, cam_gen):
    threaded.test = 1
    try:
        print('Connected by :', addr[0], ':', addr[1])
        while True:
            try:
                data = socket.recv(1024)

                if not data:
                    print('Disconnected by ' + addr[0], ':', addr[1])
                    break
                stringData_color, stringData_depth, intrins, depth_scale = next(cam_gen)

                stringData = stringData_color + stringData_depth
                sdict = {'color_len': len(stringData_color),
                         'depth_len': len(stringData_depth),
                         'intrins': intrins, 'depth_scale': depth_scale}
                threaded.sdict = sdict
                sjson = json.dumps(sdict, ensure_ascii=False)
                threaded.sjson = sjson
                sbuff = sjson.encode()
                threaded.sbuff = sbuff
                socket.send(sbuff)

                data = socket.recv(1024)

                if not data:
                    print('Disconnected by ' + addr[0], ':', addr[1])
                    break
                socket.send(stringData)

            except Exception as e:
                print('Loop error')
                print(e)
                break
    except Exception as e:
        print('Total error')
        print(e)
    finally:
        socket.close()
        print('Disconnected by ' + addr[0], ':', addr[1])


class CameraGenerator:
    def __init__(self):
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Set stream resolution
        self.config.enable_stream(rs.stream.depth, DEPTHMAP_SIZE[1], DEPTHMAP_SIZE[0], rs.format.z16, 30)
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, IMAGE_SIZE[1], IMAGE_SIZE[0], rs.format.bgr8, 30)
        # self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_sensor.set_option(rs.option.visual_preset, 3)
        # Custom = 0, Default = 1, Hand = 2, HighAccuracy = 3, HighDensity = 4, MediumDensity = 5
        self.depth_sensor.set_option(rs.option.post_processing_sharpening, 3)
        self.depth_sensor.set_option(rs.option.receiver_gain, 13)
        self.depth_sensor.set_option(rs.option.noise_filtering, 4)
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def catch_frame(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)


        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return None, None, None

        depth_intrins = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image, depth_intrins

    def encode_img(self, color_image, depth_image):

        # data = connection.recv(1024)

        # cv2. imencode(ext, img [, params])
        result, color_img = cv2.imencode('.jpg', color_image, self.encode_param)
        result, depth_img = cv2.imencode('.png', depth_image, self.encode_param)

        color_data = np.array(color_img)
        depth_data = np.array(depth_img)

        stringData_color = color_data.tostring()
        stringData_depth = depth_data.tostring()

        return stringData_color, stringData_depth

    def gen(self):
        try:
            while True:
                color_image, depth_image, depth_intrins = self.catch_frame()
                intrinsics = (depth_intrins.width, depth_intrins.height,
                              depth_intrins.fx, depth_intrins.fy,
                              depth_intrins.ppx, depth_intrins.ppy)
                if color_image is None or depth_image is None:
                    continue
                stringData_color, stringData_depth = self.encode_img(color_image, depth_image)
                yield (stringData_color, stringData_depth, intrinsics, self.depth_scale)
        except Exception as e:
            print(e)

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__del__()

    def __del__(self):
        try:
            self.pipeline.stop()
        finally:
            pass


def run_server(host=None, port=PORT):
    if host is None or host == "None":
        host = get_ip_address()
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket created on: {}'.format(host))

    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    print('Socket bind complete on - {}:{}'.format(host, port))

    server_socket.listen(10)
    print('Socket now listening')

    # connection, address = server_socket.accept()
    # print('Server start')

    camgen = CameraGenerator()
    frame_gen = camgen.gen()
    with camgen:
        try:
            while True:
                print('wait')

                connection, addr = server_socket.accept()
                # connection2, addr2 = server_socket.accept()

                print('start thread')
                threaded(connection, addr, frame_gen)
                # start_new_thread(thread1, (connection1, addr1, queue_color,))
                # start_new_thread(thread2, (connection2, addr2, queue_depth,))
        except Exception as e:
            print(e)
        finally:
            server_socket.close()

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Realsense RGBD Stream server')
    parser.add_argument('--ip', type=str, default="None",
                        help='(Optional) explicit host ip')
    args = parser.parse_args()
    run_server(host=args.ip)