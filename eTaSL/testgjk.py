from pkg.gjk import *

points = []
for i in range(2):
    for j in range(2):
        for k in range(2):
            points.append((i,j,k))
points = np.array(points)

points1 = points.copy()
points2 = points + 2

points_arr = [points1, points2]

vtx_list = np.load("vtx_list.npy", allow_pickle=True).tolist()
radius_list = np.load("radius_list.npy")
idx1_list = np.load("idx1_list.npy")
idx2_list = np.load("idx2_list.npy")

to_idx = len(vtx_list)
vtx_list_to = vtx_list[:to_idx]
idx1_list_to = [idx1 for idx1, idx2 in zip(idx1_list, idx2_list) if idx1<to_idx and idx2<to_idx]
idx2_list_to = [idx2 for idx1, idx2 in zip(idx1_list, idx2_list) if idx1<to_idx and idx2<to_idx]

# def assign_points_c_bd(bd_flt_arr, points_arr):
bd_flt_arr, points_arr = bd_flt_arr, vtx_list_to
for bd_flt, points in zip(bd_flt_arr, points_arr):
    bd_flt.numpoints = len(points)
    bd_flt.vtx_flat = MAX_VTX_ARR_TYPE(*points.flatten().tolist())

points_arr, idx1, idx2 = vtx_list_to, idx1_list_to, idx2_list_to
len_obj = len(points_arr)
len_col = len(idx1)
dist_arr = (c_double * len_col)()
clib.gjk_flat_batch(bd_flt_arr, c_int(len_obj),
                    (c_int * len_col)(*idx1), (c_int * len_col)(*idx2),
                    c_int(len_col), cast(dist_arr, POINTER(c_double)))