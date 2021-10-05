import cv2
import numpy as np
import argparse
import os
import sys
import time
import shutil
import json
import open3d as o3d
from sklearn.cluster import KMeans
from mmdet.apis import init_detector, inference_detector, show_result_pyplot

from perception_config import *
from function_utils import *

scores_thr = 0.97


def remove_folder(path_folder):
    if not os.path.exists(path_folder):
        pass
    else:
        shutil.rmtree(path_folder)


def make_folder(path_folder):
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
    else:
        pass


def filter_segmentation_class(masks):
    object_class_list = []
    for i in range(90):
        idx = i
        if idx in valid_class_dict.keys():
            if masks[idx] is not None:
                object_class_list.append(i)

    return object_class_list


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--src_path", default='', help="set cfg, checkpoint file path")
    parser.add_argument("--color_path", default='', help="set color folder path")
    parser.add_argument("--depth_path", default='', help="set depth folder path")
    parser.add_argument("--img_num", type=int, default=5, help='set image number')
    parser.add_argument("--output_path", default='', help="set segmented result file path")
    args = parser.parse_args()

    # Load config, checkpoint file of cascade mask rcnn swin based
    # config_file = '/home/jhkim/Swin-Transformer-Object-Detection/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py'
    # checkpoint_file = '/home/jhkim/Swin-Transformer-Object-Detection/cascade_mask_rcnn_swin_base_patch4_window7.pth'
    config_file = args.src_path + '/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py'
    checkpoint_file = args.src_path +'/cascade_mask_rcnn_swin_base_patch4_window7.pth'

    device = 'cuda:0'

    # Initiate model(object detector)
    model = init_detector(config_file, checkpoint_file, device=device)

    # Remove existing folders
    remove_folder(DATASET_DIR + '/color_segmented')
    remove_folder(DATASET_DIR + '/depth_segmented')

    # Load intrinsic, extrinsic parameters
    cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy, depth_scale = read_intrinsic_from_json(INTRINSIC_PATH)
    cam_traj = load_camera_trajectory(EXTRINSIC_PATH)

    output_class_dict = {}
    center_list = []
    class_list = []
    depth_list = []
    color_list = []
    obj_num = []
    # Segmentation
    for j in range(args.img_num):
        # Set color, depth image path
        color_img_path = args.color_path + '/color_{}.jpg'.format(j)
        depth_img_path = args.depth_path + '/depth_{}.png'.format(j)

        color_img = cv2.imread(color_img_path, flags=cv2.IMREAD_UNCHANGED)
        depth_img = cv2.imread(depth_img_path, flags=cv2.IMREAD_UNCHANGED)

        # Inference object detection & segmentation
        result = inference_detector(model, color_img_path)
        boxes, masks = result[0], result[1]

        # Filter to get detected class only
        object_class_list = filter_segmentation_class(masks)

        obj_num_tmp = 0
        for idx in object_class_list:
            for i in range(len(masks[idx])):
                if (boxes[idx][i][4] > scores_thr):
                    mask = masks[idx][i]
                    vis_mask = (mask * 255).astype('uint8')
                    color_instance = cv2.bitwise_and(color_img, color_img, mask=vis_mask).astype(np.uint16)
                    depth_instance = cv2.bitwise_and(depth_img, depth_img, mask=vis_mask).astype(np.uint16)
                    # cv2.imwrite(DATASET_DIR + '/temp_img.png', depth_instance)
                    # depth = o3d.io.read_image(DATASET_DIR + '/temp_img.png')
                    depth = o3d.geometry.Image(depth_instance)
                    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,
                                                                        o3d.camera.PinholeCameraIntrinsic(
                                                                        cam_width,
                                                                        cam_height, cam_fx, cam_fy,
                                                                        cam_ppx, cam_ppy),
                                                                        np.linalg.inv(cam_traj[j]),
                                                                        depth_scale=depth_scale)

                    d_trunc = np.linalg.norm(pcd.get_center()) * 1.4
                    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth,
                                                                        o3d.camera.PinholeCameraIntrinsic(
                                                                        cam_width,
                                                                        cam_height, cam_fx, cam_fy,
                                                                        cam_ppx, cam_ppy),
                                                                        np.linalg.inv(cam_traj[j]),
                                                                        depth_scale=depth_scale, depth_trunc=d_trunc)

                    center_list.append(np.round(pcd.get_center(), 5))
                    class_list.append(idx)
                    depth_list.append(depth_instance)
                    color_list.append(color_instance)
                    obj_num_tmp += 1

        obj_num.append(obj_num_tmp)


    # Clustering number is obj_num
    kmeans = KMeans(n_clusters=max(obj_num), random_state=0)
    kmeans.fit(center_list)


    for i in range(max(obj_num)):
        output_color_temp = args.output_path + '/color_segmented' + '/{}'.format(i+1)
        output_depth_temp = args.output_path + '/depth_segmented' + '/{}'.format(i+1)
        make_folder(output_color_temp)
        make_folder(output_depth_temp)

        idx_arr = np.where(kmeans.labels_ == i)[0]

        for j in range(len(idx_arr)):
            output_color_img_path = output_color_temp + '/color_mask_{}.jpg'.format(j)
            output_depth_img_path = output_depth_temp + '/depth_mask_{}.png'.format(j)
            cv2.imwrite(output_color_img_path, color_list[idx_arr[j]])
            cv2.imwrite(output_depth_img_path, depth_list[idx_arr[j]])

        # if (len(os.listdir(output_color_temp)) > args.img_num):
        #     os.remove(output_color_temp + '/color_mask_0.jpg')
        #     os.remove(output_depth_temp + '/depth_mask_0.png')

        # show_result_pyplot(model, color_img, result, score_thr=0.8)
    print(max(obj_num))

