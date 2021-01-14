import ctypes
lib = ctypes.cdll.LoadLibrary("./devel/lib/libetasl_ros_controllers.so")
# print("size")
# print(ctypes.sizeof(ctypes.c_double(300)))
# lib.etasl_create.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
# _e = lib.etasl_create(ctypes.c_double(300), 
#                       ctypes.c_double(0.0), 
#                       ctypes.c_double(0.0001))

# _dmap = lib.DoubleMap_create()
# lib.DoubleMap_set.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double]
# _dmap_ = lib.DoubleMap_set(ctypes.c_void_p(_dmap), 
#                            ctypes.c_char_p("k1"), 
#                            ctypes.c_double(0.0))

lib.DoubleMap_create("joint_map")
lib.DoubleMap_set("joint_map", "j1", ctypes.c_double(1.0))
lib.DoubleMap_get("joint_map", "j1")
