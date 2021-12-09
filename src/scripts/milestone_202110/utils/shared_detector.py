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

from pkg.utils.shared_function import shared_fun, CallType, ArgSpec, ResSpec, set_serving, is_serving, serve_forever, SHARED_FUNC_ALL

IMG_URI = "shm://color_img"
MASK_URI = "shm://mask_img"
REQ_URI = "shm://request"
RESP_URI = "shm://response"

IMG_DIM = (720, 1280, 3)

FILE_PATH = os.path.join(RNB_PLANNING_DIR, 'src/scripts/milestone_202110/utils/shared_detector.py')

class SharedDetector:
    def __init__(self):        self.initizlied = False

    @shared_fun(CallType.SYNC, "SharedDetector")
    def init(self):
        if not self.initizlied:
            self.initizlied = True
            # Load config, checkpoint file of cascade mask rcnn swin based
            config_file = os.path.join(RNB_PLANNING_DIR,
                                       'src/scripts/milestone_202110/utils/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py')
            checkpoint_file = os.path.join(RNB_PLANNING_DIR,
                                           'src/scripts/milestone_202110/utils/cascade_mask_rcnn_swin_base_patch4_window7.pth')

            # config_file = '/home/jhkim/Swin-Transformer-Object-Detection/configs/swin/cascade_mask_rcnn_swin_base_patch4_window7_mstrain_480-800_giou_4conv1f_adamw_3x_coco.py'
            # checkpoint_file = '/home/jhkim/Swin-Transformer-Object-Detection/cascade_mask_rcnn_swin_base_patch4_window7.pth'
            device = 'cuda:0'
            try:
                self.model = init_detector(config_file, checkpoint_file, device=device)
            except Exception as e:
                TextColors.RED.println("[ERROR] Could not initialize detector")
                raise(e)

    @shared_fun(CallType.SYNC, "SharedDetector",
                ArgSpec("color_img", IMG_DIM, np.uint8),
                ResSpec(0, IMG_DIM[:2], float))
    def inference(self, color_img):
        if self.initizlied:
            result = inference_detector(self.model, color_img)
            boxes, masks = result   [0], result[1]

            detect_false = np.empty((720,1280), dtype=bool)
            detect_false[:, :] = False

            return_img = np.zeros(IMG_DIM[:2])
            # Index 59 means bed
            if len(masks[59]) != 0:
                box = boxes[59][0]
                left, top, right, bottom = box[0], box[1], box[2], box[3]
                center_x = (left + right) / 2
                center_y = (top + bottom) / 2
                # check center position of bounding box of detected bed
                if center_x < 330:
                    return_img[:] = detect_false
                elif center_x > 950:
                    return_img[:] = detect_false
                elif center_y < 175:
                    return_img[:] = detect_false
                elif center_y > 545:
                    return_img[:] = detect_false
                else:
                    mask_res = masks[59][0]
                    return_img[:] = mask_res
            else:
                return_img[:] = detect_false
            return return_img
        else:
            raise(RuntimeError("[Error] Not initilized"))


if __name__ == "__main__":
    try:
        from mmdet.apis import init_detector, inference_detector
    except Exception as e:
        print(TextColors.RED.println("[ERROR] Could not import mmdat"))
        print(e)
    sdet = SharedDetector()
    set_serving(True)
    serve_forever("SharedDetector", [sdet.inference, sdet.init], verbose=True)
else:
    output = subprocess.Popen(['python3', FILE_PATH], cwd=os.path.dirname(FILE_PATH))