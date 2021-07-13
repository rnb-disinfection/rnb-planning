import SharedArray as sa
import numpy as np
import cv2
import time
from mmdet.apis import init_detector, inference_detector
import os
os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
RNB_PLANNING_DIR = os.environ["RNB_PLANNING_DIR"]

IMG_URI = "shm://color_img"
MASK_URI = "shm://mask_img"
REQ_URI = "shm://request"
RESP_URI = "shm://response"

IMG_DIM = (720, 1280, 3)


class SharedDetector:
    def __init__(self):
        # Load config, checkpoint file of cascade mask rcnn swin based
        config_file = os.path.join(RNB_PLANNING_DIR,
                                   'src/scripts/demo_202107/demo_utils/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py')
        checkpoint_file = os.path.join(RNB_PLANNING_DIR,
                                       'src/scripts/demo_202107/demo_utils/cascade_mask_rcnn_swin_base_patch4_window7.pth')

        # config_file = '/home/jhkim/Swin-Transformer-Object-Detection/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py'
        # checkpoint_file = '/home/jhkim/Swin-Transformer-Object-Detection/cascade_mask_rcnn_swin_base_patch4_window7.pth'
        device = 'cuda:0'

        # Initiate model(object detector)
        self.model = init_detector(config_file, checkpoint_file, device=device)

    def serve_forever(self):
        self.request[:] = 0
        self.resp[:] = 0
        print("===== Ready Inference Server =====")
        while True:
            while not self.request[:]:
                time.sleep(0.01)
            self.request[:] = 0
            self.resp[:] = 0
            # Inference object detection & segmentation
            result = inference_detector(self.model, self.color_img)
            boxes, masks = result[0], result[1]
            # Index 60 means dining table
            mask_res = masks[60][0]
            self.return_img[:] = mask_res
            self.resp[:] = 1

    def __enter__(self):
        self.color_img = sa.create(IMG_URI, IMG_DIM, dtype=np.uint8)
        self.return_img = sa.create(MASK_URI, IMG_DIM[:2], dtype=np.uint8)
        self.request = sa.create(REQ_URI, (1,), dtype=np.uint8)
        self.resp = sa.create(RESP_URI, (1,), dtype=np.uint8)
        self.request[:] = 0
        self.resp[:] = 0

    def __exit__(self, type, value, traceback):
        sa.delete(IMG_URI)
        sa.delete(MASK_URI)
        sa.delete(REQ_URI)
        sa.delete(RESP_URI)


if __name__ == "__main__":
    sdet = SharedDetector()
    with sdet:
        sdet.serve_forever()