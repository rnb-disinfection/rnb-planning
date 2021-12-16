import os
import sys
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from pkg.global_config import RNB_PLANNING_DIR
# Directory setting
DEMO_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/demo_202107")
DATASET_DIR = os.path.join(DEMO_DIR, "exp_datasets")

SAVE_DIR = os.path.join(DEMO_DIR, "save_img")
CROP_DIR = os.path.join(DEMO_DIR, "crop_img")
EXP_IMG_DIR = os.path.join(DEMO_DIR, "exp_dataset")
MODEL_DIR = os.path.join(DEMO_DIR, "model_CAD")
COLOR_PATH = os.path.join(DEMO_DIR, "save_img/top_table/color")
DEPTH_PATH = os.path.join(DEMO_DIR, "save_img/top_table/depth")
INTRINSIC_PATH = os.path.join(DEMO_DIR, "save_img/top_table")