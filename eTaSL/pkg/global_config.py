import os
import numpy as np
from enum import Enum

PROJ_DIR = os.getcwd()
TAMP_ETASL_DIR = os.environ['TAMP_ETASL_DIR']

OFFSET_DIR = "./offset/"
try: os.mkdir(OFFSET_DIR)
except: pass

class RobotTemplate:
    def __init__(self, base_name, joint_names, joint_limits, vel_limits, acc_limits):
        self.base_name, self.joint_names, self.joint_limits, self.vel_limits, self.acc_limits = \
            base_name, joint_names, joint_limits, vel_limits, acc_limits

class RobotType(Enum):
    indy7=0
    panda=1

class RobotSpecs:
    SPEC_DICT = {
        RobotType.indy7: RobotTemplate(base_name="link0", joint_names=["joint{}".format(idx) for idx in range(6)],
                                       joint_limits=[(-3.05432619099, 3.05432619099)]*5+[(-3.75245789179, 3.75245789179)],
                                       vel_limits=np.deg2rad([150, 150, 150, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([360]*6)),
        RobotType.panda: RobotTemplate(base_name="link0", joint_names=["joint{}".format(idx) for idx in range(1,8)],
                                       joint_limits=[(-2.75, 2.75), (-1.70, 1.70), (-2.75, 2.75),
                                                     (-2.9, -0.1), (-2.75, 2.75), (0.1, 3.6), (-2.75, 2.75)],
                                       vel_limits=np.deg2rad([150, 150, 150, 150, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([360]*7))
    }
    @classmethod
    def get_base_name(cls, _type, rname):
        return rname + "_" + cls.SPEC_DICT[_type].base_name

    @classmethod
    def get_joint_names(cls, _type, rname):
        return [rname + "_" + jname for jname in cls.SPEC_DICT[_type].joint_names]

    @classmethod
    def get_joint_limits(cls, _type):
        return cls.SPEC_DICT[_type].joint_limits

    @classmethod
    def get_vel_limits(cls, _type):
        return cls.SPEC_DICT[_type].vel_limits

    @classmethod
    def get_acc_limits(cls, _type):
        return cls.SPEC_DICT[_type].acc_limits
