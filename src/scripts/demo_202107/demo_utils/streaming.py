import pyrealsense2 as rs
import cv2
import numpy as np
import open3d as o3d

# Directory setting
import os

from demo_config import *
import socket

CONFIG_DIR = os.path.join(DEMO_DIR, "configs")
SAVE_DIR = os.path.join(DEMO_DIR, "save_img")
CROP_DIR = os.path.join(DEMO_DIR, "crop_img")
MODEL_DIR = os.path.join(DEMO_DIR, "model_CAD")

DEPTHMAP_SIZE = (480, 640)
IMAGE_SIZE = (720, 1280)

PORT = 8580

import json

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

def send_recv_demo_cam(sdict, host, port=PORT, buffer_len=1024):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect((host, port))

    rdict = {}
    try:
        sjson = json.dumps(sdict, ensure_ascii = False)
        sbuff = sjson.encode()
        client_socket.sendall(sbuff)

        rjson = client_socket.recv(buffer_len)
        send_recv_demo_cam.rjson = rjson
        rdict = json.loads(rjson)
        send_recv_demo_cam.rdict = rdict
        if rdict is None:
            rdict = {}
        rdict = {str(k): v for k,v in rdict.items()}

        client_socket.sendall(sbuff)

        length_color = int(rdict['color_len'])
        length_depth = int(rdict['depth_len'])
        stringData = recvall(client_socket, length_color+length_depth)
        stringData_color = stringData[0:length_color]
        stringData_depth = stringData[int(length_color):int(length_color) + int(length_depth)]
        data_color = np.frombuffer(stringData_color, dtype='uint8')
        data_depth = np.frombuffer(stringData_depth, dtype='uint8')

        decimg_color = cv2.imdecode(data_color, cv2.IMREAD_COLOR)
        decimg_depth = cv2.imdecode(data_depth, cv2.IMREAD_ANYDEPTH)

        rdict['color'] = decimg_color
        rdict['depth'] = decimg_depth
    except Exception as e:
        print(e)
    finally:
        client_socket.close()
    return rdict


from enum import Enum


class ImageType(Enum):
    FirstView = 1
    CloseView = 2


##
# @brief press s to save image
def stream_capture_image(img_type, host):
    print("== press s to save image ==")
    while True:
        rdict = send_recv_demo_cam({1: 1}, host=host, port=PORT, buffer_len=1024)
        cv2.imshow('ColorImage', rdict['color'])
        cv2.imshow('DepthImage', rdict['depth'])

        key = cv2.waitKey(1)
        if (key == 27):
            cv2.destroyAllWindows()
            break
        elif key == 115:
            if img_type == ImageType.FirstView:
                cv2.imwrite(SAVE_DIR + '/color.jpg', rdict['color'])
                cv2.imwrite(SAVE_DIR + '/depth.png', rdict['depth'])
            if img_type == ImageType.CloseView:
                cv2.imwrite(SAVE_DIR + '/table.jpg', rdict['color'])
                cv2.imwrite(SAVE_DIR + '/table.png', rdict['depth'])
    return rdict