import os
import numpy as np
from enum import Enum

PROJ_DIR = os.getcwd()
TF_GMT_ETASL_DIR = os.environ['TF_GMT_ETASL_DIR']

OFFSET_DIR = "./offset/"
try: os.mkdir(OFFSET_DIR)
except: pass

class RobotType(Enum):
    indy7_robot=0
    panda_robot=1