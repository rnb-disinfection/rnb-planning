# -*- coding: utf-8 -*- 

from enum import Enum
from nrmkindy import IndyMotionProperty_pb2 as prop


class TrajStateEvent(Enum):
    BEGIN = 1
    ACC_DONE = 2
    CRZ_DONE = 3
    DEC_DONE = 4
    CANCEL = 5


class MoveBaseMode(Enum):
    ABSOLUTE = prop.MoveBaseMode.INDY_MOVE_BASE_MODE_ABSOLUTE
    RELATIVE = prop.MoveBaseMode.INDY_MOVE_BASE_MODE_RELATIVE
    _USE_GLOBAL = prop.MoveBaseMode.INDY_MOVE_BASE_MODE_USE_GLOBAL


class InterpolatorType(Enum):
    CONSTVEL = prop.InterpolatorType.INDY_INTPRTR_TYPE_VEL_CONSTVEL
    TIMEOPT = prop.InterpolatorType.INDY_INTPRTR_TYPE_VEL_TIMEOPT
    TIME = prop.InterpolatorType.INDY_INTPRTR_TYPE_TIME
    _USE_GLOBAL = prop.InterpolatorType.INDY_INTPRTR_TYPE_USE_GLOBAL


class BlendingType(Enum):
    # NOT_USED = prop.BlendingType.INDY_BLEND_TYPE_NOT_USED
    DUPLICATE = prop.BlendingType.INDY_BLEND_TYPE_DUPLICATE
    OVERRIDE = prop.BlendingType.INDY_BLEND_TYPE_OVERRIDE
    _USE_GLOBAL = prop.BlendingType.INDY_BLEND_TYPE_USE_GLOBAL


class ReferenceCoordinateType(Enum):
    BASE = prop.ReferenceCoordinateType.INDY_REF_COORD_TYPE_BASE
    REF_FRAME = prop.ReferenceCoordinateType.INDY_REF_COORD_TYPE_FRAME
    TOOL = prop.ReferenceCoordinateType.INDY_REF_COORD_TYPE_TOOL
    _USE_GLOBAL = prop.ReferenceCoordinateType.INDY_REF_COORD_TYPE_USE_GLOBAL


class ReferenceFrameType(Enum):
    DIRECT = prop.ReferenceFrameType.INDY_REF_FRAME_SET_TYPE_DIRECT
    LINEAR = prop.ReferenceFrameType.INDY_REF_FRAME_SET_TYPE_LINEAR
    CIRCULAR = prop.ReferenceFrameType.INDY_REF_FRAME_SET_TYPE_CIRCULAR
    _USE_GLOBAL = prop.ReferenceFrameType.INDY_REF_FRAME_SET_TYPE_USE_GLOBAL


class JointMotionVel:
    def __init__(self, vel=-1, acc=-1):
        self.vel = vel if vel is not None and (isinstance(vel, float) or isinstance(vel, int)) else -1
        self.acc = acc if acc is not None and (isinstance(acc, float) or isinstance(acc, int)) else -1

    def get(self):
        return self.vel, self.acc


class TaskMotionVel:
    def __init__(self, disp_vel=-1, disp_acc=-1, rot_vel=-1, rot_acc=-1):
        self.disp_vel = disp_vel if disp_vel is not None and (isinstance(disp_vel, float) or isinstance(disp_vel, int)) else -1
        self.disp_acc = disp_acc if disp_acc is not None and (isinstance(disp_acc, float) or isinstance(disp_acc, int)) else -1
        self.rot_vel = rot_vel if rot_vel is not None and (isinstance(rot_vel, float) or isinstance(rot_vel, int)) else -1
        self.rot_acc = rot_acc if rot_acc is not None and (isinstance(rot_vel, float) or isinstance(rot_vel, int)) else -1

    def get(self):
        return self.disp_vel, self.disp_acc, self.rot_vel, self.rot_acc


class ReferenceFrame:
    def __init__(self, ref_type=ReferenceFrameType._USE_GLOBAL,
                 ref_tref=(0, 0, 0, 0, 0, 0),
                 ref_points=(0, 0, 0, 0, 0, 0, 0, 0, 0)):
        self.ref_type = ref_type if ref_type is not None and (isinstance(ref_type, ReferenceFrameType) or isinstance(ref_type, int)) else ReferenceFrameType._USE_GLOBAL
        self.ref_tref = ref_tref if ref_tref is not None and (isinstance(ref_tref, list) or isinstance(ref_tref, tuple)) else (0, 0, 0, 0, 0, 0)
        self.ref_points = ref_points if ref_points is not None and (isinstance(ref_points, list) or isinstance(ref_points, tuple)) else (0, 0, 0, 0, 0, 0, 0, 0, 0)

    def get(self):
        # return self.ref_type, self.ref_tref, self.ref_points
        return {
            'type': self.ref_type,
            'tref': self.ref_tref,
            'points': self.ref_points,
        }


class ToolCenterPoint:
    def __init__(self, tcp=(0, 0, 0, 0, 0, 0)):
        self.tcp = tcp if tcp is not None and (isinstance(tcp, list) or isinstance(tcp, tuple)) else (0, 0, 0, 0, 0, 0)

    def get(self):
        return self.tcp

