import os
import sys


# os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.rotation_utils import *
from pkg.geometry.geotype import GEOTYPE

# Directory setting
WORKING_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/perception")
DATASET_DIR = WORKING_DIR + "/dataset"
COLOR_PATH = DATASET_DIR + "/color"
DEPTH_PATH = DATASET_DIR + "/depth"
COLOR_SEG_PATH = DATASET_DIR + "/color_segmented"
DEPTH_SEG_PATH = DATASET_DIR + "/depth_segmented"
INTRINSIC_PATH = DATASET_DIR + "/intrinsic.json"
EXTRINSIC_PATH = WORKING_DIR + "/trajectory.log"
RECONST_PCD_PATH = WORKING_DIR + "/pcd.ply"


# Valid class for object detection & segmentation
valid_class_dict = {24:'backpack', 25:'umbrella', 26:'handbag', 27:'tie', 28:'suitcase',
                   29:'bottle', 40:'wine glass', 41:'cup', 42:'fork', 43:'knife',
                   44:'spoon', 45:'bowl', 56:'chair', 57:'couch', 58:'potted plant',
                   59:'bed', 60:'dining table', 61:'toilet', 62:'tv', 63:'laptop',
                   64:'mouse', 65:'remote', 66:'keyboard', 67:'cell phone', 73:'book',
                   74:'clock', 75:'vase', 76:'scissors', 77:'teddy bear', 78:'hair drier', 79:'toothbrush'}