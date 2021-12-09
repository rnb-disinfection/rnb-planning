import SharedArray as sa
import numpy as np
import cv2
import time
import os
import sys

# os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
# sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/perception'))
from perception_config import *
from pkg.global_config import RNB_PLANNING_DIR
from scripts.perception.boost_gpd.gpd_interface import grasp_interface_py as gi


# WORKING_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/perception")
CFG_PATH = WORKING_DIR + "/boost_gpd/cfg/eigen_params.cfg"
PCD_NUM_URI = "shm://pcd_num"
GPD_LIST_URI = "shm://gpd_list"
REQ_URI = "shm://request"
RESP_URI = "shm://response"



class SharedDetector:
    def __init__(self):
        print("=============================================")

    def serve_forever(self):
        self.request[:] = 0
        self.resp[:] = 0
        print("===== Ready Grasp Pose Detection Server =====")
        print("=============================================")
        while True:
            while not self.request[:]:
                time.sleep(0.01)
            self.request[:] = 0
            self.resp[:] = 0

            # Get Grasp Pose
            grasp_list = gi.Vec4ListList()
            grasp_list = gi.getGPD(CFG_PATH, WORKING_DIR + "/object_{}.pcd".format(self.pcd_num[0]))

            # Unpack Vec4ListList
            for j in range(len(grasp_list)):
                temp_list = grasp_list[j]
                for k in range(4):
                    temp = temp_list[k]
                    if k == 0:
                        grasp_pose = np.array([temp[0], temp[1], temp[2], temp[3]])
                    else:
                        grasp_pose = np.vstack([grasp_pose, np.array([temp[0], temp[1], temp[2], temp[3]])])

                self.return_gpd_list[j][:, :] = grasp_pose

            print("====== Success to Grasp Pose Detection ======")
            print("=============================================")
            self.resp[:] = 1

    def __enter__(self):
        try:
            sa.delete(PCD_NUM_URI)
            sa.delete(GPD_LIST_URI)
            sa.delete(REQ_URI)
            sa.delete(RESP_URI)
        except Exception as e:
            pass

        self.pcd_num = sa.create(PCD_NUM_URI, (1,), dtype=np.uint8)
        self.return_gpd_list = sa.create(GPD_LIST_URI, (10, 4, 4))
        self.request = sa.create(REQ_URI, (1,), dtype=np.uint8)
        self.resp = sa.create(RESP_URI, (1,), dtype=np.uint8)
        self.request[:] = 0
        self.resp[:] = 0

    def __exit__(self, type, value, traceback):
        sa.delete(PCD_NUM_URI)
        sa.delete(GPD_LIST_URI)
        sa.delete(REQ_URI)
        sa.delete(RESP_URI)


if __name__ == "__main__":
    sdet = SharedDetector()
    with sdet:
        sdet.serve_forever()
