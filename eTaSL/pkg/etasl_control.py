import os
from pkg.global_config import *

import ctypes

lib = ctypes.cdll.LoadLibrary(os.path.join(TAMP_ETASL_DIR, 'ws_ros/devel/lib/libetasl_ros_controllers.so'))
lib.etasl_create.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.DoubleMap_get.restype = ctypes.c_double

JOINT_NAMES_CONTROL = None

def initialize_etasl_control(context_text, joint_names, zeros_pose,
                             nWSR=300, cputime=0.0, regularization_factor=0.001, 
                             initialization_time=10.0, sample_time=1e-3, convergence_crit=1e-4
                            ):
    global JOINT_NAMES_CONTROL
    JOINT_NAMES_CONTROL = joint_names
    lib.etasl_create("etasl", nWSR,cputime,regularization_factor)
    lib.etasl_readTaskSpecification("etasl", context_text)
    lib.DoubleMap_create("init_val")
    lib.DoubleMap_create("conv_val")
    lib.DoubleMap_create("input")
    lib.DoubleMap_create("output")
    lib.DoubleMap_create("joint_val")
    lib.DoubleMap_create("joint_val_new")
    lib.DoubleMap_create("joint_vel")
    
    for jname, jval in zip(JOINT_NAMES_CONTROL, zeros_pose):
        lib.DoubleMap_set("init_val", jname, ctypes.c_double(jval))
        lib.DoubleMap_set("joint_val", jname, ctypes.c_double(jval))
    lib.etasl_initialize("etasl", "init_val", 
                         ctypes.c_double(initialization_time), 
                         ctypes.c_double(sample_time), 
                         ctypes.c_double(convergence_crit), "conv_val")
    lib.etasl_setInput("etasl", "input")
    
def update_step(dt, joint_pos=None, input_dict=None):
    if input_dict is not None:
        for iname, ival in input_dict.items():
            lib.DoubleMap_set("joint_val", iname, ctypes.c_double(ival))
        lib.etasl_setInput("etasl", "input")
    if joint_pos is not None:
        for jname, jval in zip(JOINT_NAMES_CONTROL, joint_pos):
            lib.DoubleMap_set("joint_val", jname, ctypes.c_double(jval))
        lib.etasl_setJointPos("etasl", "joint_val")
    lib.etasl_updateStep("etasl", ctypes.c_double(dt))
    lib.etasl_getJointPos("etasl", "joint_val_new")
    lib.etasl_getJointVel("etasl", "joint_vel")
    joint_vals = []
    joint_vels = []
    for jname in JOINT_NAMES_CONTROL:
        joint_vals.append(lib.DoubleMap_get("joint_val_new", jname))
        joint_vels.append(lib.DoubleMap_get("joint_vel", jname))
    return joint_vals, joint_vels