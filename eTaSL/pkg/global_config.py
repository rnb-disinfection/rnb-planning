import os
import numpy as np
from enum import Enum

PROJ_DIR = os.getcwd()
TAMP_ETASL_DIR = os.environ['TAMP_ETASL_DIR']

OFFSET_DIR = "./offset/"
try: os.mkdir(OFFSET_DIR)
except: pass

class RobotType(Enum):
    indy7_robot=0
    panda_robot=1

    @classmethod
    def get_base_link(cls, _type, name):
        if _type in [RobotType.indy7_robot, RobotType.panda_robot]:
            return name+'_link0'