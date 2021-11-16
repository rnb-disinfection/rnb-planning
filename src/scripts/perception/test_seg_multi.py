from mmdet.apis import init_detector, inference_detector, show_result_pyplot
import cv2
import numpy as np
import os

config_file = '/home/jhkim/Swin-Transformer-Object-Detection/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py'
checkpoint_file = '/home/jhkim/Swin-Transformer-Object-Detection/cascade_mask_rcnn_swin_base_patch4_window7.pth'

device = 'cuda:0'
model = init_detector(config_file, checkpoint_file, device=device)

# os.makedirs('/home/jhkim/Open3D/examples/python/reconstruction_system/dataset/realsense/color_segmented')
# os.makedirs('/home/jhkim/Open3D/examples/python/reconstruction_system/dataset/realsense/depth_segmented')

img_num = 20
for i in range(0, img_num):
    j = i
    # color_img_path = 'save_img/mini_table/color/color_{}.jpg'.format(j)
    # depth_img_path = 'save_img/mini_table/depth/depth_{}.png'.format(j)
    # color_img_path = 'save_img/washstand/color/color_{}.jpg'.format(j)
    # depth_img_path = 'save_img/washstand/depth/depth_{}.png'.format(j)
    color_img_path = 'save_img/bed/color/color_{}.jpg'.format(j)
    depth_img_path = 'save_img/bed/depth/depth_{}.png'.format(j)

    result = inference_detector(model, color_img_path)

    boxes, masks = result[0], result[1]

    color_img = cv2.imread(color_img_path, flags=cv2.IMREAD_UNCHANGED)
    depth_img = cv2.imread(depth_img_path, flags=cv2.IMREAD_UNCHANGED)
    # 총 detect한 것들 중에서 하나만 masking 해보자고
    # index - table 60, bed 59, toilet 61, sink 71
    mask = masks[59][0]
    # if len(masks[60]) == 0:
    #     mask = masks[59][0]
    # else:
    #     mask = masks[60][0]
    vis_mask = (mask * 255).astype('uint8')
    color_instance = cv2.bitwise_and(color_img, color_img, mask=vis_mask).astype(np.uint16)
    depth_instance = cv2.bitwise_and(depth_img, depth_img, mask=vis_mask).astype(np.uint16)

    # cv2.imwrite(
    #     'save_img/mini_table/color_segmented/color_mask_{}.jpg'.format(j), color_instance)
    # cv2.imwrite(
    #   'save_img/mini_table/depth_segmented/depth_mask_{}.png'.format(j), depth_instance)
    # cv2.imwrite(
    #     'save_img/washstand/color_segmented/color_mask_{}.jpg'.format(j), color_instance)
    # cv2.imwrite(
    #   'save_img/washstand/depth_segmented/depth_mask_{}.png'.format(j), depth_instance)
    cv2.imwrite(
        'save_img/bed/color_segmented/color_mask_{}.jpg'.format(j), color_instance)
    cv2.imwrite(
      'save_img/bed/depth_segmented/depth_mask_{}.png'.format(j), depth_instance)
    #show_result_pyplot(model, img, result, score_thr=0.3)

print("Segmentation Finish!")