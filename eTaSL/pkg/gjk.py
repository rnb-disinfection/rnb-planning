from .global_config import *
import ctypes
from ctypes import *

OBJ_MAX = 100
VTX_MAX = 20
COL_MAX = 1000

clib = ctypes.cdll.LoadLibrary(os.path.join(TF_GMT_ETASL_DIR, "openGJK/lib/libopenGJKlib.so"))
clib.gjk_flat_batch.restype = ctypes.c_double

MAX_VTX_ARR_TYPE = c_double * (VTX_MAX * 3)

class bd_flt(Structure):
    _fields_ = [("numpoints", c_int),
                ("vtx_flat", MAX_VTX_ARR_TYPE)
                ]

BD_FLT_ARR_TYPE = bd_flt * OBJ_MAX

bd_flt_arr = BD_FLT_ARR_TYPE()


def assign_points_c_bd(bd_flt_arr, points_arr):
    for bd_flt, points in zip(bd_flt_arr, points_arr):
        bd_flt.numpoints = len(points)
        bd_flt.vtx_flat = MAX_VTX_ARR_TYPE(*points.flatten().tolist())


def get_distance_batch(points_arr, idx1, idx2):
    assign_points_c_bd(bd_flt_arr, points_arr)
    len_obj = len(points_arr)
    len_col = len(idx1)
    dist_arr = (c_double * len_col)()
    clib.gjk_flat_batch(bd_flt_arr, c_int(len_obj),
                        (c_int * len_col)(*idx1), (c_int * len_col)(*idx2),
                        c_int(len_col), cast(dist_arr, POINTER(c_double)))
    return np.array(dist_arr)