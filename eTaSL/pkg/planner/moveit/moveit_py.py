import os
import ctypes
from numpy.ctypeslib import ndpointer
import numpy as np
from ...global_config import PROJ_DIR

PLAN_LIB_PATH = os.path.join(PROJ_DIR, "moveit_plan_compact", "libmoveit_plan_compact.so")

from enum import Enum

class ObjectType(Enum):
    BOX = 1
    SPHERE = 2
    CYLINDER = 3

class ObjectAction(Enum):
    ADD = 0
    REMOVE = 1
    APPEND = 2
    MOVE = 3

clib = ctypes.cdll.LoadLibrary(PLAN_LIB_PATH)
MAX_STR_LEN = clib.get_max_str_len()
MAX_NAME_LEN = clib.get_max_name_len()
MAX_JOINT_NUM = clib.get_max_joint_num()
MAX_TRAJ_LEN = clib.get_max_traj_len()

class c_string(ctypes.Structure):
    _fields_ = [("buffer", ctypes.c_char*MAX_STR_LEN),
                ("len", ctypes.c_int)
                ]

class c_trajectory(ctypes.Structure):
    _fields_ = [("joints", ctypes.c_double*(MAX_JOINT_NUM*MAX_TRAJ_LEN)),
                ("name_len", ctypes.c_int),
                ("traj_len", ctypes.c_int),
                ("success", ctypes.c_bool)
                ]

class c_plan_request(ctypes.Structure):
    _fields_ = [("group_name", ctypes.c_char*MAX_NAME_LEN),
                ("tool_link", ctypes.c_char*MAX_NAME_LEN),
                ("goal_link", ctypes.c_char*MAX_NAME_LEN),
                ("init_state", ctypes.c_double*MAX_JOINT_NUM),
                ("goal_pose", ctypes.c_double*7),
                ("timeout", ctypes.c_double)
                ]

class c_object_msg(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char*MAX_NAME_LEN),
                ("link_name", ctypes.c_char*MAX_NAME_LEN),
                ("pose", ctypes.c_double*7),
                ("dims", ctypes.c_double*3),
                ("type", ctypes.c_int),
                ("action", ctypes.c_int)
                ]

clib.hello_cstr.restype = c_string
clib.hello_char.restype = ndpointer(dtype=ctypes.c_char, shape=(MAX_STR_LEN,))
clib.init_planner.restype = c_string
clib.plan_compact.restype = c_trajectory
clib.plan_compact.argtypes = [c_plan_request]
clib.process_object.argtypes = [c_object_msg]

def convert_trajectory(traj, joint_num):
    joints = []
    for i_traj in range(traj.traj_len):
        joints.append(traj.joints[i_traj*MAX_JOINT_NUM:i_traj*MAX_JOINT_NUM+joint_num])
    return np.array(joints), traj.success
