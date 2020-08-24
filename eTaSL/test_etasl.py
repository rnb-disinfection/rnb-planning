from pkg.robots_custom import *
xcustom = XacroCustomizer()
xcustom.clear()
xcustom.add_robot(RobotType.indy7_robot, xyz=[0,-0.5,0], rpy= [0,0,0])
xcustom.add_robot(RobotType.panda_robot, xyz=[0,0.5,0], rpy= [0,0,0])
xcustom.write_xacro()
vel_scale = 1.0/2.0
JOINT_NAMES, LINK_NAMES, ZERO_JOINT_POSE, urdf_content = \
        xcustom.convert_xacro_to_urdf(
        joint_fix_dict={'finger':'upper'},
        vel_limit_dict={k: v*vel_scale for k,v in {
            'panda1_joint1': np.deg2rad(150),  'panda1_joint2': np.deg2rad(150), 
            'panda1_joint3': np.deg2rad(150), 'panda1_joint4': np.deg2rad(150), 
            'panda1_joint5': np.deg2rad(180), 'panda1_joint6': np.deg2rad(180), 'panda1_joint7': np.deg2rad(180), 
            'indy0_joint0': np.deg2rad(150), 'indy0_joint1': np.deg2rad(150), 'indy0_joint2': np.deg2rad(150), 
            'indy0_joint3': np.deg2rad(180), 'indy0_joint4': np.deg2rad(180), 'indy0_joint5': np.deg2rad(180), 
        }.items()}
    )
ZERO_JOINT_POSE=np.array([0,0,-np.pi/2,0,-np.pi/2,0,
                          0,-np.pi/8,0,-np.pi/2,0,np.pi/2,0])


import os
from pkg.global_config import *
import ctypes

lib = ctypes.cdll.LoadLibrary(os.path.join(TF_GMT_ETASL_DIR, 'ws_ros/devel/lib/libetasl_ros_controllers.so'))

lib.etasl_create.argtypes = [ctypes.c_char_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.DoubleMap_get.restype = ctypes.c_double

lib.etasl_create("etasl", 300,0.0,0.001)

context_text = np.load("ctx.npy")
lib.etasl_readTaskSpecification("etasl", context_text.tostring())

lib.DoubleMap_create("init_val")
lib.DoubleMap_create("conv_val")
lib.DoubleMap_create("input")
lib.DoubleMap_create("output")
lib.DoubleMap_create("joint_val")
lib.DoubleMap_create("joint_val_new")

for jname, jval in zip(JOINT_NAMES, ZERO_JOINT_POSE):
    lib.DoubleMap_set("init_val", jname, ctypes.c_double(jval))
    lib.DoubleMap_set("joint_val", jname, ctypes.c_double(jval))
    
lib.etasl_initialize("etasl", "init_val", 
                     ctypes.c_double(10.0), 
                     ctypes.c_double(0.004), 
                     ctypes.c_double(1E-4), "conv_val")
lib.etasl_setInput("etasl", "input")

print("etasl_setJointPos")
joint_vals = []
for jname in JOINT_NAMES:
    joint_vals.append(lib.DoubleMap_get("joint_val", jname))
print(np.round(joint_vals,2))

lib.etasl_setJointPos("etasl", "joint_val")

print("etasl_updateStep")
lib.etasl_updateStep("etasl", ctypes.c_double(1e-2))

lib.etasl_getJointPos("etasl", "joint_val_new")

print("etasl_getJointPos")
joint_vals = []
for jname in JOINT_NAMES:
    joint_vals.append(lib.DoubleMap_get("joint_val_new", jname))
print(np.round(joint_vals,2))



