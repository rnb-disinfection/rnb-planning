import os
import sys
import shutil
import random
import time
PROJ_DIR = os.environ["RNB_PLANNING_DIR"]
sys.path.append(os.path.join(PROJ_DIR, "src"))

import SharedArray as sa
import numpy as np
import time
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='start shared lattice predictor')
    parser.add_argument('--rtype', type=str, help='robot type name')
    parser.add_argument('--model_path', type=str, default="None", help='relative path to model')
    parser.add_argument('--precision', type=str, default="FP16", help='TRT precision. FP32, FP16 (Default), INT8. INT8 is not currently supported needs calibration and decreases accuracy')
    args = parser.parse_args()
    if args.model_path == "None":
        args.model_path = None
    PRECISION = args.precision

    ok_to_go = False
    while not ok_to_go:
        try:
            prepared_p = sa.create(f"shm://{args.rtype}.prepared", (1,), dtype=np.bool)
            ok_to_go = True
        except Exception as e:
            print(e)
            robot_type_name = args.rtype
            query_quit = sa.attach("shm://{}.query_quit".format(robot_type_name))
            query_quit[0] = True
            time.sleep(0.5)
            try:
                sa.delete("shm://{}.grasp_img".format(robot_type_name))
                sa.delete("shm://{}.arm_img".format(robot_type_name))
                sa.delete("shm://{}.rh_vals".format(robot_type_name))
                sa.delete("shm://{}.result".format(robot_type_name))
                sa.delete("shm://{}.query_in".format(robot_type_name))
                sa.delete("shm://{}.response_out".format(robot_type_name))
                sa.delete("shm://{}.query_quit".format(robot_type_name))
            except:
                pass
    prepared_p[0] = False

import tensorflow as tf
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession
config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

from tensorflow.python.saved_model import tag_constants, signature_constants
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2


from pkg.utils.utils_python3 import *
from pkg.controller.robot_config import RobotType
from pkg.planning.filtering.lattice_model.data_utils import *
import numpy as np
int2rtypename = {v.value:v.name for v in RobotType}
DATA_PATH = os.path.join(PROJ_DIR, "data")
MODEL_PATH = os.path.join(PROJ_DIR, "model")
LAT_MODEL_PATH = os.path.join(MODEL_PATH,"latticized")
try_mkdir(MODEL_PATH)
try_mkdir(LAT_MODEL_PATH)
GRASP_FOLDER = "grasp"
ARM10_FOLDER = "arm_10"
ARM05_FOLDER = "arm_05"
FULLS_FOLDER = "full_scene"

ARM_FOLDER = ARM10_FOLDER
GRASP_SHAPE = (20,20,20)
ARM_SHAPE = (20,20,20)
RH_MASK_SIZE = 512
RH_MASK_STEP = 64

BATCH_SIZE = 1
SERVER_PERIOD = 1e-3

##
# @class SharedLatticePredictor
class SharedLatticePredictor:
    ##
    # @param ROBOT_TYPE_NAME robot type name
    # @param model_path_rel relative model path from model/latticized/
    def __init__(self, ROBOT_TYPE_NAME="indy7", model_path_rel=None):
        self.ROBOT_TYPE_NAME = ROBOT_TYPE_NAME
        self.ROBOT_MODEL_ROOT = os.path.join(LAT_MODEL_PATH, self.ROBOT_TYPE_NAME)
        if model_path_rel is None:
            last_model = sorted(os.listdir(self.ROBOT_MODEL_ROOT))[-1]
            last_save = sorted([item for item in os.listdir(os.path.join(self.ROBOT_MODEL_ROOT, last_model)) if item.startswith("model")])[-1]
            model_path_rel = os.path.join(last_model, last_save)
        model_log_dir = os.path.join(self.ROBOT_MODEL_ROOT, model_path_rel)
        print("============== Load model from: ====================")
        print(model_log_dir)
        self.model = tf.keras.models.load_model(model_log_dir)

    @tf.function
    def inference(self, images):
        # training=False is only needed if there are layers with different
        # behavior during training versus inference (e.g. Dropout).
        predictions = self.model(images, training=False)
        return predictions

    ##
    # @brief Create an array in shared memory.
    # @param prepared_p bool shared array (1,) to signal readiness
    def start_server(self, prepared_p):
        grasp_img_p = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.grasp_img", (BATCH_SIZE,) + GRASP_SHAPE + (3,))
        arm_img_p = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.arm_img", (BATCH_SIZE,) + ARM_SHAPE + (1,))
        rh_vals_p = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.rh_vals", (BATCH_SIZE, 2))
        result_p = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.result", (BATCH_SIZE, 2))
        query_in = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.query_in", (1,), dtype=np.bool)
        response_out = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.response_out", (1,), dtype=np.bool)
        query_quit = sa.create(f"shm://{self.ROBOT_TYPE_NAME}.query_quit", (1,), dtype=np.bool)
        gtimer = GlobalTimer.instance()
        gtimer.reset()
        grasp_img_p[:] = 0
        arm_img_p[:] = 0
        rh_vals_p[:] = 0
        result_p[:] = 0
        query_in[0] = False
        response_out[0] = False
        query_quit[0] = False
        rh_mask = np.zeros((BATCH_SIZE, 54))

        print("============= wait for initialization ================")
        r_mask = div_r_gaussian(rh_vals_p[0][0])
        h_mask = div_h_gaussian(rh_vals_p[0][1])
        rh_mask[0] = np.concatenate([r_mask, h_mask])
        inputs = [tf.constant(grasp_img_p, dtype=tf.float32),
                  tf.constant(arm_img_p, dtype=tf.float32),
                  tf.constant(rh_mask, dtype=tf.float32)]
        self.inference(inputs)
        print("=============== initialization done ==================")
        prepared_p[0] = True

        try:
            while not query_quit[0]:
                if not query_in[0]:
                    time.sleep(SERVER_PERIOD)
                    continue
                query_in[0] = False
                ## TODO: inference depending on robot type
                r_mask = div_r_gaussian(rh_vals_p[0][0])
                h_mask = div_h_gaussian(rh_vals_p[0][1])
                rh_mask[0] = np.concatenate([r_mask, h_mask])
                inputs = [tf.constant(grasp_img_p, dtype=tf.float32),
                          tf.constant(arm_img_p, dtype=tf.float32),
                          tf.constant(rh_mask, dtype=tf.float32)]
                result = self.inference(inputs).numpy()
                for i_b in range(BATCH_SIZE):
                    result_p[i_b] = result[i_b]
                response_out[0] = True
        finally:
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.grasp_img")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.arm_img")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.rh_vals")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.result")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.query_in")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.response_out")
            sa.delete(f"shm://{self.ROBOT_TYPE_NAME}.query_quit")

if __name__ == "__main__":
    try:
        slp = SharedLatticePredictor(ROBOT_TYPE_NAME=args.rtype, model_path_rel=args.model_path)
        slp.start_server(prepared_p)
    finally:
        sa.delete(f"shm://{args.rtype}.prepared")