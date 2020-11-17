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

    @classmethod
    def get_joints(cls, _type, name):
        if _type == RobotType.indy7_robot:
            return ["{}_joint{}".format(name, idx) for idx in range(6)]
        elif _type == RobotType.panda_robot:
            return ["{}_joint{}".format(name, idx) for idx in range(1,8)]