import SharedArray as sa
import numpy as np
import cv2
import time
import os
import sys
import subprocess
RNB_PLANNING_DIR = os.environ["RNB_PLANNING_DIR"]
sys.path.append(os.path.join(os.path.join(RNB_PLANNING_DIR, 'src')))
sys.path.append(os.path.join(RNB_PLANNING_DIR, 'src/scripts/milestone_202110'))
if sys.version.startswith("3"):
    from pkg.utils.utils_python3 import *
elif sys.version.startswith("2"):
    from pkg.utils.utils import *

from pkg.utils.shared_function import shared_fun, CallType, ArgProb, ResProb, set_serving, is_serving, serve_forever, SHARED_FUNC_ALL

IMG_URI = "shm://color_img"
MASK_URI = "shm://mask_img"
REQ_URI = "shm://request"
RESP_URI = "shm://response"

IMG_DIM = (720, 1280, 3)

FILE_PATH = os.path.join(RNB_PLANNING_DIR, 'src/scripts/demo_202107/demo_utils/shared_detector.py')


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
        if __name__ == "__main__":
            try:
                self.model = init_detector(config_file, checkpoint_file, device=device)
            except Exception as e:
                TextColors.RED.println("[ERROR] Could not initialize detector")
                print(e)
        else:
            output = subprocess.Popen(['python3', FILE_PATH], cwd=os.path.dirname(FILE_PATH))


    @shared_fun(CallType.SYNC, "SharedDetector",
                ArgProb("color_img", IMG_DIM, np.uint8),
                ResProb(0, IMG_DIM[:2], float))
    def inference(self, color_img):
        # Inference object detection & segmentation
        result = inference_detector(self.model, color_img)
        boxes, masks = result[0], result[1]
        # Index 60 means dining table
        mask_res = masks[60][0]
        return mask_res

if __name__ == "__main__":
    try:
        from mmdet.apis import init_detector, inference_detector
    except Exception as e:
        print(TextColors.RED.println("[ERROR] Could not import mmdat"))
        print(e)
    sdet = SharedDetector()
    set_serving(True)
    serve_forever("SharedDetector", [sdet.inference], verbose=True)