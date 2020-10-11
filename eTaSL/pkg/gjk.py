from .global_config import *
import ctypes
from ctypes import *

OBJ_MAX = 100
VTX_MAX = 100
COL_MAX = 1000

clib = ctypes.cdll.LoadLibrary(os.path.join(TAMP_ETASL_DIR, "openGJK/lib/libopenGJKlib.so"))
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


from scipy.spatial import ConvexHull
from .utils import *
from .geometry import *
from .joint_utils import *
from .constraint_base import *

def swept_volume_test(Q1, Q2, collision_items, joint_names, urdf_content,
                      fixed_collision_items=None, fixed_col_list=[], fixed_col_swept_list=[]):
    vtx1_list = []
    vtx2_list = []
    vtx_swept_list = []
    radius_list = []
    # gtimer.tic("vtx")
    Q1dict = joint_list2dict(Q1, joint_names)
    T_dict1 = get_tf_full('indy0_tcp', Q1dict, urdf_content)
    T_dict1.update(get_tf_full('panda1_leftfinger', Q1dict, urdf_content))
    T_dict1.update({'panda1_rightfinger': get_tf('panda1_rightfinger', Q1dict, urdf_content),
                    'world': np.identity(4)})
    Q2dict = joint_list2dict(Q2, joint_names)
    T_dict2 = get_tf_full('indy0_tcp', Q2dict, urdf_content)
    T_dict2.update(get_tf_full('panda1_leftfinger', Q2dict, urdf_content))
    T_dict2.update({'panda1_rightfinger': get_tf('panda1_rightfinger', Q2dict, urdf_content),
                    'world': np.identity(4)})
    for ctem in GeometryItem.GLOBAL_GEO_LIST:
        vtx_ref, radi = ctem.get_vertice_radius()
        Toff = ctem.get_offset_tf()
        T_c1 = np.matmul(T_dict1[ctem.link_name], Toff)
        vtx1 = np.matmul(vtx_ref, T_c1[:3, :3].transpose()) + T_c1[:3, 3]
        T_c2 = np.matmul(T_dict2[ctem.link_name], Toff)
        vtx2 = np.matmul(vtx_ref, T_c2[:3, :3].transpose()) + T_c2[:3, 3]
        vtx = np.concatenate([vtx1, vtx2], axis=0)
        vtx = np.round(list(set([tuple(v) for v in vtx])), 3)
        if len(vtx) >= 4:
            try:
                hull = ConvexHull(vtx)
                vtx = hull.points[hull.vertices]
            except:
                pass
        vtx1_list.append(vtx1)
        vtx2_list.append(vtx2)
        vtx_swept_list.append(vtx)
        radius_list.append(radi)

    # gtimer.toc("vtx")

    # gtimer.tic("col_list")
    colllision_items_2 = None
    if fixed_collision_items is not None:
        colllision_items_2 = fixed_collision_items+collision_items

    col_list, col_swept_list = make_colliding_list(collision_items, colllision_items_2,
                                                   min_distance_map=get_min_distance_map(),
                                                   link_adjacency_map_ext=get_link_adjacency_map_ext())
    col_list += fixed_col_list
    col_swept_list += fixed_col_swept_list

    idx1_list, idx2_list, dcut_list = [], [], []
    for col in col_list:
        idx1 = GeometryItem.GLOBAL_GEO_LIST.index(col[0])
        idx2 = GeometryItem.GLOBAL_GEO_LIST.index(col[1])
        idx1_list.append(idx1)
        idx2_list.append(idx2)
        dcut_list.append(radius_list[idx1] + radius_list[idx2])
    idx1swept_list, idx2swept_list, dcutswept_list = [], [], []
    for col in col_swept_list:
        idx1 = GeometryItem.GLOBAL_GEO_LIST.index(col[0])
        idx2 = GeometryItem.GLOBAL_GEO_LIST.index(col[1])
        idx1swept_list.append(idx1)
        idx2swept_list.append(idx2)
        dcutswept_list.append(radius_list[idx1] + radius_list[idx2])
    # gtimer.toc("col_list")

    # gtimer.tic("gjk")
    dist2_list = get_distance_batch(vtx2_list, idx1_list, idx2_list)
    dist_swept_list = get_distance_batch(vtx_swept_list, idx1swept_list, idx2swept_list)
    dist2_list = np.subtract(dist2_list, dcut_list)
    dist_swept_list = np.subtract(dist_swept_list, dcutswept_list)
    # gtimer.toc("gjk")
    return all([np.all(dist2_list > 0), np.all(dist_swept_list > 0)])