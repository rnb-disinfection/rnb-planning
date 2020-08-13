# -*- coding: utf-8 -*- 

import math

NAME_INDY_7 = "NRMK-Indy7"
NAME_INDY_12 = "NRMK-Indy12"
NAME_INDY_RP2 = "NRMK-IndyRP2"

# FIXME model limits
class IndyBaseModel:
    NAME = ""
    JOINT_DOF = 6
    MAX_JOINT_VEL_DEG = 180
    MAX_JOINT_VEL_RAD = math.radians(MAX_JOINT_VEL_DEG)
    MAX_JOINT_ACC_DEG = 360
    MAX_JOINT_ACC_RAD = math.radians(MAX_JOINT_ACC_DEG)
    MAX_DISP_VEL_METER = 1
    MAX_DISP_ACC_METER = 2
    MAX_ROT_VEL_DEG = 180
    MAX_ROT_VEL_RAD = math.radians(MAX_ROT_VEL_DEG)
    MAX_ROT_ACC_DEG = 360
    MAX_ROT_ACC_RAD = math.radians(MAX_ROT_ACC_DEG)
    MIN_MOVE_TIME = 0.5
    MAX_MOVE_TIME = 600.0
    JOINT_LIMITS_FORWARD_DEG = [175, 175, 175, 175, 175, 215]
    JOINT_LIMITS_BACKWARD_DEG = [-175, -175, -175, -175, -175, -215]
    JOINT_LIMITS_FORWARD_RAD = [math.radians(deg) for deg in JOINT_LIMITS_FORWARD_DEG]
    JOINT_LIMITS_BACKWARD_RAD = [math.radians(deg) for deg in JOINT_LIMITS_BACKWARD_DEG]

class Indy7Model(IndyBaseModel):
    NAME = NAME_INDY_7

class IndyRP2Model(IndyBaseModel):
    NAME = NAME_INDY_RP2
    JOINT_DOF = 7
    JOINT_LIMITS_FORWARD_DEG = [150, 150, 150, 180, 180, 180, 180]
    JOINT_LIMITS_BACKWARD_DEG = [-150, -150, -150, -180, -180, -180, -180]
    JOINT_LIMITS_FORWARD_RAD = [math.radians(deg) for deg in JOINT_LIMITS_FORWARD_DEG]
    JOINT_LIMITS_BACKWARD_RAD = [math.radians(deg) for deg in JOINT_LIMITS_BACKWARD_DEG]
