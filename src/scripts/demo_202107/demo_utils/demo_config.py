import os
import sys
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
from pkg.global_config import RNB_PLANNING_DIR
# Directory setting
DEMO_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/demo_202107")
DATASET_DIR = os.path.join(DEMO_DIR, "exp_datasets")