import pyrealsense2 as rs
import cv2
import numpy as np
import open3d as o3d

# Directory setting
import os

from milestone_config import *
import socket

SAVE_DIR = os.path.join(MILESTONE_DIR, "save_img")
CROP_DIR = os.path.join(MILESTONE_DIR, "crop_img")
MODEL_DIR = os.path.join(MILESTONE_DIR, "model_CAD")

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
    FullView = 3
    # ReconstView = 3


##
# @brief press s to save image
def stream_capture_image(img_type, obj_type, host):
    # print("== press s to save image ==")
    while True:
        rdict = send_recv_demo_cam({1: 1}, host=host, port=PORT, buffer_len=1024)
        cv2.imshow('ColorImage', rdict['color'])
        cv2.imshow('DepthImage', rdict['depth'])

        # key = cv2.waitKey(1)
        key = 115
        if (key == 27):
            cv2.destroyAllWindows()
            break
        elif key == 115:
            if img_type == ImageType.FirstView:
                if obj_type == "closet":
                    cv2.imwrite(SAVE_DIR + '/top_table.jpg', rdict['color'])
                    cv2.imwrite(SAVE_DIR + '/top_table.png', rdict['depth'])
                else:
                    cv2.imwrite(SAVE_DIR + '/bed.jpg', rdict['color'])
                    cv2.imwrite(SAVE_DIR + '/bed.png', rdict['depth'])
            if img_type == ImageType.CloseView:
                if obj_type == "closet":
                    cv2.imwrite(SAVE_DIR + '/top_table_close.jpg', rdict['color'])
                    cv2.imwrite(SAVE_DIR + '/top_table_close.png', rdict['depth'])
                else:
                    cv2.imwrite(SAVE_DIR + '/bed_close.jpg', rdict['color'])
                    cv2.imwrite(SAVE_DIR + '/bed_close.png', rdict['depth'])
            if img_type == ImageType.FullView:
                cv2.imwrite(SAVE_DIR + '/full_view.jpg', rdict['color'])
                cv2.imwrite(SAVE_DIR + '/full_view.png', rdict['depth'])
            break
    return rdict


# def stream_scan_image(img_type, obj_type, count, host):
#     # print("== press s to save image ==")
#     while True:
#         rdict = send_recv_demo_cam({1: 1}, host=host, port=PORT, buffer_len=1024)
#         cv2.imshow('ColorImage', rdict['color'])
#         cv2.imshow('DepthImage', rdict['depth'])
#
#         # key = cv2.waitKey(1)
#         key = 115
#         if (key == 27):
#             cv2.destroyAllWindows()
#             break
#         elif key == 115:
#             if img_type == ImageType.FirstView:
#                 if obj_type == "closet":
#                     cv2.imwrite(SAVE_DIR + '/top_table.jpg', rdict['color'])
#                     cv2.imwrite(SAVE_DIR + '/top_table.png', rdict['depth'])
#                 else:
#                     cv2.imwrite(SAVE_DIR + '/bed.jpg', rdict['color'])
#                     cv2.imwrite(SAVE_DIR + '/bed.png', rdict['depth'])
#             if img_type == ImageType.CloseView:
#                 if obj_type == "closet":
#                     cv2.imwrite(SAVE_DIR + '/top_table_close.jpg', rdict['color'])
#                     cv2.imwrite(SAVE_DIR + '/top_table_close.png', rdict['depth'])
#                 else:
#                     cv2.imwrite(SAVE_DIR + '/bed_close.jpg', rdict['color'])
#                     cv2.imwrite(SAVE_DIR + '/bed_close.png', rdict['depth'])
#             if img_type == ImageType.ReconstView:
#                 cv2.imwrite(SAVE_DIR + '/top_table/color/top_table_close_{0:04d}.jpg'.format(count), rdict['color'])
#                 cv2.imwrite(SAVE_DIR + '/top_table/depth/top_table_close_{0:04d}.png'.format(count), rdict['depth'])
#             break
#     return rdict
