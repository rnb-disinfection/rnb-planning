import os
import ctypes
from gtimer import GlobalTimer
from numpy.ctypeslib import ndpointer
import numpy as np

PLAN_LIB_PATH = "../cmake-build-debug/moveit_plan_compact.so"

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
    _fields_ = [("names_flt", ctypes.c_char*(MAX_JOINT_NUM*MAX_NAME_LEN)),
                ("joints", ctypes.c_double*(MAX_JOINT_NUM*MAX_TRAJ_LEN)),
                ("name_len", ctypes.c_int),
                ("joint_count", ctypes.c_int),
                ("joint_max", ctypes.c_int),
                ("traj_len", ctypes.c_int),
                ("success", ctypes.c_bool)
                ]

class c_plan_request(ctypes.Structure):
    _fields_ = [("group_name", ctypes.c_char*MAX_NAME_LEN),
                ("tool_link", ctypes.c_char*MAX_NAME_LEN),
                ("goal_link", ctypes.c_char*MAX_NAME_LEN),
                ("goal_pose", ctypes.c_double*7)
                ]

clib.hello_cstr.restype = c_string
clib.hello_char.restype = ndpointer(dtype=ctypes.c_char, shape=(MAX_STR_LEN,))
clib.plan_compact.restype = c_trajectory
clib.plan_compact.argtypes = [c_plan_request]

def convert_trajectory(traj):
    names = traj.names_flt.split()
    joints = []
    for i_traj in range(traj.traj_len):
        joints.append(traj.joints[i_traj*traj.joint_max:i_traj*traj.joint_max+traj.joint_count])
    return names, np.array(joints), traj.success


gtimer = GlobalTimer.instance()


urdf_path = "../test_assets/custom_robots.urdf"
srdf_path = "../test_assets/custom_robots.srdf"

urdf_str = c_string()
srdf_str = c_string()
with open(urdf_path, 'r') as f:
    while True:
        line = f.readline()
        urdf_str.buffer += line
        if not line: break
            
urdf_str.buffer
with open(srdf_path, 'r') as f:
    while True:
        line = f.readline()
        srdf_str.buffer += line
        if not line: break
            
            
gtimer.reset(1e3,'ms')
gtimer.tic("init_ros")
clib.init_planner(urdf_str, srdf_str)
gtimer.toc("init_ros")



for _ in range(10):
    gtimer.tic("plan_compact")
    goal = c_plan_request()
    goal.group_name = "indy0"
    goal.tool_link = "indy0_tcp"
    goal_pose = [-0.3, -0.2, 0.4, 0, 0, 0, 1]
    for igp in range(7): goal.goal_pose[igp] = goal_pose[igp]
    goal.goal_link = "base_link"
    c_traj = clib.plan_compact(goal)
    gtimer.toc("plan_compact")
    gtimer.tic("convert_trajectory")
    names, traj, succ = convert_trajectory(c_traj)
    gtimer.toc("convert_trajectory")
print(gtimer)
print(succ)


