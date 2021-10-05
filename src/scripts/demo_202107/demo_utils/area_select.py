import numpy as np
from enum import Enum
from demo_config import *
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from collections import defaultdict

DATASET_DIR = os.path.join(os.environ["RNB_PLANNING_DIR"], 'data/sweep_reach')
try_mkdir(DATASET_DIR)
# dataset = np.load('scripts/demo_202107/dataset.npy', allow_pickle=True)


def load_sweep_data(robot_type_name):
    return np.load(os.path.join(DATASET_DIR, robot_type_name+".npy"), allow_pickle=True)


##
# @return {depth list: (sweep_width, area, divisions, div_num)}
def get_division_dict(match_range_dict, DEPTH_DIV, TABLE_DIMS, TOOL_DIM, DEPTH_MIN, DEPTH_MAX, MARGIN=0):
    TABLE_DEPTH, TABLE_WIDTH, TABLE_HEIGHT = TABLE_DIMS
    TOOL_DEPTH = TOOL_DIM[0]
    print("== TOOL_DEPTH: {} ==".format(TOOL_DEPTH))
    MOTION_DEPTH =TABLE_DEPTH - TOOL_DEPTH +MARGIN
    print("== MOTION_DEPTH: {} ==".format(MOTION_DEPTH))
    wipe_depth = MOTION_DEPTH/DEPTH_DIV
    print("== WIPE_DEPTH: {} ==".format(wipe_depth))
    depths = sorted([depth for depth in match_range_dict.keys() if DEPTH_MIN<=depth<=DEPTH_MAX])
    division_dict = {}
    for depth1, depth2 in combinations(depths, 2):
        depth1, depth2 = sorted([depth1, depth2])
        wdepth = depth2 - depth1
        if wdepth >= wipe_depth:
            depths_cur = [dp for dp in depths if depth1<=dp<=depth2]
            min_swp = int(ceil(wipe_depth / TOOL_DEPTH))
            for n_swp in range(min_swp, min_swp+2):
                for depths_test in combinations(depths_cur, n_swp):
                    if depths_test[-1]-depths_test[0] < wipe_depth: # total depth too shallow
                        continue
                    sweep_intervals = np.subtract(depths_test[1:], depths_test[:-1])
                    if any(sweep_intervals>TOOL_DEPTH) : # too big step
                        continue
                    ranges_test = np.array([match_range_dict[dp_] for dp_ in depths_test])
                    range_new = np.max(ranges_test[:,0]), np.min(ranges_test[:,1])
                    sweep_width = -np.subtract(*range_new)-MARGIN
                    if sweep_width <= 0: # no available sweep width
                        continue
                    area = sweep_width, np.max(depths_test)-np.min(depths_test)+TOOL_DEPTH
                    divisions = (int(ceil(TABLE_WIDTH/sweep_width)), DEPTH_DIV)
                    div_num = np.multiply(*divisions)
                    division_dict[depths_test] = (sweep_width, area, range_new, divisions, div_num)
    return division_dict


def select_task_area(robot_config, TABLE_DIMS, TOOL_DIM, EE_DEPTH_OFF, EE_HEIGHT, ROBOT_Z_ANGLE,
                     TOOL_DEPTH_MIN=0.6, TOOL_DEPTH_MAX=1.0, MARGIN=0, allow_back_approach=False):
    print("reference height: {}".format(EE_HEIGHT))
    if ROBOT_Z_ANGLE == np.pi:
        flipper = -1
    elif ROBOT_Z_ANGLE == 0:
        flipper = 1
    else:
        raise (NotImplementedError("Unexpected robot orientation"))

    sweep_data = load_sweep_data(robot_config.type.name).reshape((1,))[0]
    range_list_dict, best_range_dict = sweep_data['range_list_dict'], sweep_data['best_range_dict']
    heights = sorted(set([key[2] for key in best_range_dict.keys()]))
    i_high = np.where(np.subtract(heights, EE_HEIGHT) >= 0)[0][0]
    i_low = i_high - 1
    h_high = heights[i_high]
    h_low = heights[i_low]

    match_range_dict_high = defaultdict(lambda: (-1e5, 1e5))
    match_range_dict_low = defaultdict(lambda: (-1e5, 1e5))
    match_range_all = {}
    for key, brange in best_range_dict.items():
        hkey = round(key[2], 4)
        key_new = round(flipper * key[0], 4)
        if hkey == h_low:  # get minimum range from the upper and lower layers
            brange_new = tuple(reversed(np.multiply(flipper, brange)))
            range_old = -np.subtract(*match_range_dict_low[key_new])
            range_new = -np.subtract(*brange_new)
            if range_new < range_old:
                match_range_dict_low[key_new] = brange_new
                match_range_all[key_new] = range_list_dict[key]
        if hkey == h_high:  # get minimum range from the upper and lower layers
            brange_new = tuple(reversed(np.multiply(flipper, brange)))
            range_old = -np.subtract(*match_range_dict_high[key_new])
            range_new = -np.subtract(*brange_new)
            if range_new < range_old:
                match_range_dict_high[key_new] = brange_new
                match_range_all[key_new] = range_list_dict[key]

    match_range_dict = {}
    best_keys = set(match_range_dict_high.keys()).intersection(match_range_dict_low.keys())
    for key in best_keys:
        range_high = match_range_dict_high[key]
        range_low = match_range_dict_high[key]
        match_range_dict[key] = range_high if -np.subtract(*range_high) < -np.subtract(*range_low) else range_low
    #
    # # plot available area
    # plt.figure(figsize=(15, 5))
    # plt.subplot(1, 2, 1)
    # plt.title("best only")
    # for k, v in match_range_dict.items():
    #     plt.plot(v, [k, k], '-s')
    # plt.axis([-1, 1, 0.2, 1.1])
    # plt.grid()
    # plt.subplot(1, 2, 2)
    # for k, v in match_range_all.items():
    #     for vv in v:
    #         plt.plot(vv, [k, k], '-o')
    # plt.title("all ranges")
    # plt.axis([-1, 1, 0.2, 1.1])
    # plt.grid()

    DEPTH_MIN = TOOL_DEPTH_MIN + EE_DEPTH_OFF
    DEPTH_MAX = TOOL_DEPTH_MAX + EE_DEPTH_OFF
    division_dict_1 = get_division_dict(match_range_dict, 1, TABLE_DIMS, TOOL_DIM,
                                        DEPTH_MIN=DEPTH_MIN, DEPTH_MAX=DEPTH_MAX, MARGIN=MARGIN)
    if allow_back_approach:
        division_dict_2 = get_division_dict(match_range_dict, 2, TABLE_DIMS, TOOL_DIM,
                                            DEPTH_MIN=DEPTH_MIN, DEPTH_MAX=DEPTH_MAX, MARGIN=MARGIN)
    else:
        division_dict_2 = {}

    # divisions1_sorted = sorted(division_dict_1.items(), key=lambda item_: item_[1][-1])
    # divisions2_sorted = sorted(division_dict_2.items(), key=lambda item_: item_[1][-1])
    divisions_sorted = sorted(division_dict_1.items() + division_dict_2.items(), key=lambda item_: item_[1][-1])
    assert len(divisions_sorted) > 0, "no available table division solution"

    division_sol = divisions_sorted[0]
    depths = division_sol[0]
    sweep_width, (area_width, area_depth), width_range, divisions, div_num = division_sol[1]
    corner_center = ((max(*depths) + min(*depths)) / 2 - EE_DEPTH_OFF, np.mean(width_range))
    print("sweep depths: {}".format(depths))
    print("divisions: {}".format(divisions))
    return sweep_width, (area_width, area_depth), width_range, divisions, div_num, corner_center

class Corners(Enum):
    Left = 0
    Right = 1


CornerSequence = [Corners.Left, Corners.Right, Corners.Left, Corners.Right]
RotationSequence = [np.identity(3), np.identity(3), Rot_axis(3, np.pi), np.identity(3)]

corner_point_dirs = {Corners.Left: np.array([-1, 1, 1]),
                     Corners.Right: np.array([-1, -1, 1])}

def select_area(TABLE_HEIGHT, TABLE_DEPTH, TABLE_WIDTH):
    TABLE_HEIGHT = np.round(TABLE_HEIGHT, 3)
    len_dat = len(dataset)
    for i in range(len_dat):
        print(dataset[i][0])
        if ((TABLE_HEIGHT >= dataset[i][0]) and
                (TABLE_HEIGHT < dataset[i + 1][0] if i < len_dat - 1 else TABLE_HEIGHT < dataset[i][0] + 0.04)):
            data = dataset[i]
            break

    height_goal, depth_criteria, opt_depth_sort, optimal_list, width_list, area_list = data

    # reduce areas by sensor error margin
    SENSOR_MARGIN = 0.1
    optimal_list = [[pt1, pt2, round(width - SENSOR_MARGIN, 3), round(depth - SENSOR_MARGIN, 3)] for pt1, pt2, width, depth
                    in optimal_list]
    optimal_dict = {opt_tem[-1]: opt_tem for opt_tem in optimal_list}
    # width_list = [width-width_list for width in width_list]
    opt_depth_sort = np.round(np.subtract(opt_depth_sort, SENSOR_MARGIN), 3)

    # select area depth based on table depth
    if TABLE_DEPTH <= depth_criteria:
        area_depth_obj = TABLE_DEPTH
    else:
        if TABLE_DEPTH > max(opt_depth_sort) * 2:
            print("==========================================================")
            print("=========== TOO DEEP TABLE! CAN'T FINISH WIPING TASK =============")
            print("==========================================================")
#             raise (RuntimeError("TOO DEEP TABLE! CAN'T FINISH WIPING TASK"))
            area_depth_obj = max(opt_depth_sort)
        else:
            area_depth_obj = TABLE_DEPTH / 2

    # select optimal item for the area depth
    idx_depth = np.min(np.where(np.subtract(opt_depth_sort, area_depth_obj) >= 0)[0])
    area_depth = opt_depth_sort[idx_depth]
    area_corner1, area_corner2, area_width, _ = optimal_dict[area_depth]

    # fit table width to table
    if TABLE_WIDTH < area_width:
        area_width = TABLE_WIDTH

    # get area parameters
    print("area_corner1: {}".format(area_corner1))
    print("area_corner2: {}".format(area_corner2))
    corner_center = [(area_corner1[0] + area_corner2[0]) / 2, (area_corner1[1] + area_corner2[1]) / 2]
    corner_center = np.round(corner_center, 5)
    area = area_depth * area_width
    area = np.round(area, 5)

    if area_depth_obj == depth_criteria:
        area_depth = area_depth_obj
    else:
        if len(opt_depth_sort) == 1:
            area_depth = opt_depth_sort[0]
        else:
            for j in range(len(opt_depth_sort) - 1):
                if (area_depth_obj >= opt_depth_sort[j]) & (area_depth_obj < opt_depth_sort[j + 1]):
                    area_depth = opt_depth_sort[j]
                    break
                else:
                    area_depth = opt_depth_sort[len(opt_depth_sort) - 1]
                    break
    corner_center = [(area_corner1[0] + area_corner2[0]) / 2, (area_corner1[1] + area_corner2[1]) / 2]
    corner_center = np.round(corner_center, 5)
    area = area_depth * area_width
    area = np.round(area, 5)

    if area_depth < TABLE_DEPTH:
        num_depth = 2
    else:
        num_depth = 1

    num_width, rem = divmod(TABLE_WIDTH, area_width)
    if not rem == 0:
        num_width += 1
    num_width = int(num_width)

    num_area = num_width * num_depth

    return corner_center, area_width, area_depth, height_goal, num_width, num_depth

from pkg.planning.constraint.constraint_common import BindingTransform

class CachedCollisionCheck:
    def __init__(self, gcheck, wp_task, wp_hdl, wayframer):
        self.gcheck = gcheck
        self.wp_task, self.wp_hdl, self.wayframer = \
            wp_task, wp_hdl, wayframer
        self.cache = {}

    def get_cache_key(self, T_loal, Q_dict):
        xyzquat = T2xyzquat(T_loal)
        key = (tuple(np.round(xyzquat[0], 3)),
               tuple(np.round(xyzquat[1], 3)),
               tuple(np.round(
                   dict2list(Q_dict,
                             self.gcheck.gscene.joint_names))))
        return key

    def __call__(self, T_loal, Q_dict):
        key = self.get_cache_key(T_loal, Q_dict)
        if key in self.cache:
            return self.cache[key]
        res = self.gcheck.check(
            BindingTransform(self.wp_task, self.wp_hdl, self.wayframer,
                             T_loal=T_loal), Q_dict)
        self.cache[key] = res
        return res


SWEEP_DAT_PATH = os.path.join(os.environ["RNB_PLANNING_DIR"], "data/sweep_reach")


##
# @brief get division dictionary
# @param plane_val value along the plane normal axis
# @param ccheck CachedCollisionCheck
# @param tip_dir None, up, down
# @return {approach dir 0~3: {Tsm_key: [idx_div]}}, surface_div_centers
def get_division_dict(surface, brush_face, robot_config, plane_val, tip_dir, TOOL_DIM, ccheck):
    ## get data
    rtype = robot_config.type.name
    sweep_path = os.path.join(SWEEP_DAT_PATH, rtype if tip_dir is None else "{}-{}".format(rtype, tip_dir))
    sweep_max = np.loadtxt(sweep_path+"-max.csv", delimiter=",")
    sweep_min = np.loadtxt(sweep_path+"-min.csv", delimiter=",")

    gcheck = ccheck.gcheck
    pscene = gcheck.pscene
    gscene = pscene.gscene
    crob = pscene.combined_robot

    ## set axes
    ax_pln = 2 if tip_dir is None else 0
    ax_swp = 1
    idx_pln = np.argmin(np.abs(sweep_max[:, ax_pln] - plane_val))
    val_pln = sweep_max[idx_pln, ax_pln]
    idc_pln = np.where(sweep_max[:, ax_pln] == val_pln)[0]
    sweep_max = sweep_max[idc_pln, :]
    sweep_min = sweep_min[idc_pln, :]
    ax_step = [ax for ax in [0,1,2] if ax not in [ax_pln, ax_swp]][0] # step axis = not plane axis nor sweep axis

    ## divide surface with squares
    sqdiv_size_ref = np.max(TOOL_DIM)
    surf_offset = 0.05
    surface_dim_eff = np.subtract(surface.dims[:2], sqdiv_size_ref) # effective surface dims - except tool dimension to boundaries
    div_close = np.ceil(surface_dim_eff[0]/sqdiv_size_ref).astype(np.int)
    sqdiv_size = surface_dim_eff[0]/div_close # reduce square size to fit the closed ends
    div_num_ = np.ceil(surface_dim_eff/sqdiv_size).astype(np.int)
    surface_dim_eff = np.round(sqdiv_size*div_num_, 5) # enpand effective size of open ends
    div_num = div_num_ + 1
    surface_div_centers = [tuple(np.round(-surface_dim_eff/2+np.multiply(sqdiv_size, (i,j)), 5))
                            for i, j in product(range(div_num[0]), range(div_num[1]))]

    ## get all sweep points
    swp_points_all = []
    for step_val, min_val, max_val in zip(sweep_min[:, ax_step], sweep_min[:, ax_swp], sweep_max[:, ax_swp]):
        diff_val = max_val - min_val
        sweep_num = np.floor(diff_val / sqdiv_size).astype(np.int)
        min_val_clip = (max_val + min_val) / 2 - (sweep_num * sqdiv_size / 2) + (sqdiv_size / 2)

        swp_points = np.zeros((sweep_num, 3))
        swp_points[:, ax_swp] = min_val_clip + np.arange(sweep_num) * sqdiv_size
        swp_points[:, ax_step] = step_val
        swp_points[:, ax_pln] = plane_val
        swp_points_all.append(np.round(swp_points, 3))
    swp_points_all = np.concatenate(swp_points_all)

    ## make feasible base-div dictionary
    ROBOT_BASE = pscene.robot_chain_dict[robot_config.get_indexed_name()]["link_names"][0]
    TIP_LINK = brush_face.geometry.link_name
    Tbs = surface.get_tf(crob.home_dict)
    Tmr = gcheck.gscene.get_tf(to_link=ROBOT_BASE, from_link=robot_config.root_on, Q=crob.home_dict)
    Trm = SE3_inv(Tmr)
    if tip_dir is None:
        Rre = Rot_rpy([np.pi, np.pi/2, 0])
    elif tip_dir == "up":
        Rre = Rot_rpy([np.pi, 0, 0])
    elif tip_dir == "down":
        Rre = Rot_rpy([np.pi, np.pi, 0])
    else:
        raise(RuntimeError("Not defined"))
    div_base_dict = defaultdict(lambda: defaultdict(list))
    Tbm_in_all = []
    Tbm_fail_all = []
    Tbm_succ_all = []
    for i_div, div_center in enumerate(surface_div_centers):
            Tsc = SE3(np.identity(3), tuple(div_center)+(surface.dims[2]/2,))
            for i in range(4):
                Tct = SE3(Rot_axis(3, i * np.pi/2), [0, 0, surf_offset])
                Tst = matmul_series(Tsc, Tct)
                for swp_point in swp_points_all:
                    Tre = SE3(Rre, swp_point)
                    Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)
                    Trt = matmul_series(Tre, Tet)
                    Tsr = matmul_series(Tst, SE3_inv(Trt))
                    Tsm = np.matmul(Tsr, Trm)
                    Tbm = np.matmul(Tbs, Tsm)
                    if (np.all(np.abs(Tsm[:2,3]) < np.divide(surface.dims[:2], 2))
                        or (np.abs(Tsm[0,3]) > np.divide(surface.dims[0], 2))): # mobile loc inside surface
                        Tbm_in_all.append(Tbm)
                        continue
                    if ccheck(T_loal=Tbm, Q_dict=crob.home_dict): #check feasible
                        Tbm_succ_all.append(Tbm)
                        Tsm_xq = T2xyzquat(Tsm)
                        Tsm_key = tuple(np.round(Tsm_xq[0], 3)), tuple(np.round(Tsm_xq[1], 3))
                        div_base_dict[Tsm_key][i].append(i_div)
                    else:
                        Tbm_fail_all.append(Tbm)
    Tsm_keys = sorted(div_base_dict.keys())
    return div_base_dict, Tsm_keys, surface_div_centers, sqdiv_size, div_num, (ax_step, ax_swp, ax_pln)


## repeat selecting max-cover base
def select_max_cover_bases(div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step):
    covered_all = []
    idx_bases = []
    idc_divs = []
    while True:
        ## make feasible base-div matrix
        Tsm_key_len = len(Tsm_keys)
        div_len = len(surface_div_centers)
        div_base_mat = np.zeros((Tsm_key_len, 4, div_len), dtype=np.int)
        for i_aproc in range(4):
            for i_tsm, Tsm_key in enumerate(Tsm_keys):
                idc_div = div_base_dict[Tsm_key][i_aproc]
                idc_div = sorted(set(idc_div) - set(covered_all))
                if len(idc_div) > 0:
                    vec_stp = np.zeros(3)
                    vec_stp[ax_step] = 1
                    vec_stp = np.abs(np.matmul(Rot_axis(3, i_aproc * np.pi / 2), vec_stp))
                    ax_stp_ = np.where(vec_stp > 0.5)[0][0]
                    idc_div_rav = np.array(np.unravel_index(idc_div, div_num))
                    u_list, c_list = np.unique(idc_div_rav[ax_stp_, :], return_counts=True)
                    count_dict = {u: c for u, c in zip(u_list, c_list)}
                    for i_div in idc_div:
                        i_div_rav = np.unravel_index(i_div, div_num)
                        div_c = surface_div_centers[i_div]
                        div_base_mat[i_tsm, i_aproc, i_div] = count_dict[i_div_rav[ax_stp_]]
        ## Select best bases
        div_base_mat_max = np.max(div_base_mat, axis=1)
        div_marks = np.any(div_base_mat_max, axis=0)
        if not np.any(div_marks):  ## repeat base selecting untile no div remaining
            break
        base_covers = np.max(div_base_mat_max, axis=1)  # max sweep length
        idx_max = np.argmax(base_covers)
        covereds = np.where(div_base_mat_max[idx_max] == base_covers[idx_max])[0]
        covered_all = sorted(np.concatenate([covered_all, covereds]).astype(np.int))
        idx_bases.append(idx_max)
        idc_divs.append(covereds)
    return idx_bases, idc_divs, covered_all


def get_min_max_sweep_points(sweep_centers, sqdiv_size, TOOL_DIM_SWEEP):
    swp_min = np.min(sweep_centers, axis=0)
    swp_max = np.max(sweep_centers, axis=0)
    ax_swp = np.where(swp_min!=swp_max)[0] # sweep axis
    swp_min[ax_swp] = swp_min[ax_swp]-sqdiv_size/2+TOOL_DIM_SWEEP
    swp_max[ax_swp] = swp_max[ax_swp]+sqdiv_size/2-TOOL_DIM_SWEEP
    return swp_min, swp_max, ax_swp


from pkg.geometry.geometry import *
from pkg.planning.constraint.constraint_common import *
from pkg.planning.constraint.constraint_actor import *
from pkg.planning.constraint.constraint_subject import *

def add_sweep_task(pscene, sweep_name, surface, swp_min, swp_max, Tsm, wp_dims,
                   color_sweep=(0.6, 0.0, 0.0, 0.3), color_wp=(0.6, 0.0, 0.0, 0.5)):
    gscene = pscene.gscene
    wp_list = []
    ax_swp = np.where(swp_min!=swp_max)[0][0] # sweep axis
    ax_swp_s = np.where(np.abs(Tsm[:3,ax_swp])>0.5)[0][0]
    sweep_dim = list(wp_dims)
    sweep_dim[ax_swp_s] = np.subtract(swp_max, swp_min)[ax_swp] + wp_dims[ax_swp_s]
    sweep_dim = tuple(sweep_dim)
    gscene.create_safe(gtype=GEOTYPE.BOX, name=sweep_name, link_name="base_link",
                       dims=sweep_dim + (surface.dims[2],),
                       center=tuple(np.mean([swp_min, swp_max], axis=0)) + (0,),
                       rpy=Rot2rpy(Tsm[:3, :3]), color=color_sweep, display=True,
                       collision=False, fixed=True, parent=surface.name)

    if np.matmul(Tsm[:2,:3].transpose(), swp_min)[ax_swp_s] < np.matmul(Tsm[:2,:3].transpose(), swp_max)[ax_swp_s]:
        swp_0 = swp_max
        swp_1 = swp_min
    else:
        # swp_0 = swp_min
        # swp_1 = swp_max
        swp_0 = swp_max
        swp_1 = swp_min
    for wp_idx, wp_pos in [(0, swp_0), (1, swp_1)]:
        wp_list.append(gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_wp_{}".format(sweep_name, wp_idx), link_name="base_link",
                                          dims=tuple(wp_dims[:2])+(surface.dims[2],), center=tuple(wp_pos)+(0,),
                                          rpy=Rot2rpy(Tsm[:3,:3]), color=color_wp, display=True,
                                          collision=False, fixed=True, parent=surface.name))
    sweep_task = pscene.create_subject(oname=sweep_name, gname=sweep_name, _type=SweepLineTask,
                                       action_points_dict={wp.name: SweepFrame(wp.name, wp, [0,0,wp.dims[2]/2], [0,0,0])
                                                           for wp in wp_list})
    return sweep_task

def add_waypoint_task(pscene, name, dims, center, rpy, parent, color=(1, 1, 0, 0.5)):
    gscene = pscene.gscene
    wp = gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_wp".format(name), link_name="base_link",
                               dims=dims, center=center, rpy=rpy,
                               color=color, display=True, collision=False, fixed=True, parent=parent)
    wp_hdl = WayFrame(wp.name, wp, [0, 0, dims[2] / 2], [0, 0, 0])
    wp_task = pscene.create_subject(oname="waypoints", gname="floor_ws", _type=WayopintTask,
                                    action_points_dict={wp_hdl.name: wp_hdl})
    return wp_task, wp_hdl