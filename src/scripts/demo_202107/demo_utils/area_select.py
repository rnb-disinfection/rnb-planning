import numpy as np
from enum import Enum
from demo_config import *
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from collections import defaultdict

DATASET_DIR = os.path.join(os.environ["RNB_PLANNING_DIR"], 'data/sweep_reach')
try_mkdir(DATASET_DIR)

class Corners(Enum):
    Left = 0
    Right = 1
    
class SweepDirections(Enum):
    front="front"
    up="up"
    down="down"
    
    @classmethod
    def get_dcm_re(cls, tip_dir):
        if isinstance(tip_dir, cls):
            tip_dir = tip_dir.value
        if tip_dir is None or tip_dir == cls.front.value:
            Rre = Rot_rpy([0, np.pi/2, 0])
        elif tip_dir == cls.up.value:
            Rre = Rot_rpy([0, 0, 0])
        elif tip_dir == cls.down.value:
            Rre = Rot_rpy([0, np.pi, np.pi])
        else:
            raise (RuntimeError("Not defined"))
        return Rre
    
    @classmethod
    def get_file_name(cls, rtype, tip_dir):
        if not isinstance(rtype, str):
            rtype = rtype.name
        if isinstance(tip_dir, cls):
            tip_dir = tip_dir.value
        return rtype if tip_dir is None else "{}-{}".format(rtype, tip_dir)
    
    @classmethod
    def check_fourway(cls, tip_dir):
        return tip_dir is None or tip_dir==cls.front or tip_dir==cls.front.name

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
                             self.gcheck.gscene.joint_names), 3)))
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

    def clear(self):
        self.cache = {}


SWEEP_DAT_PATH = os.path.join(os.environ["RNB_PLANNING_DIR"], "data/sweep_reach")


##
# @brief    get division dictionary. the surface is divided into sections after subtracting max tool dimension. \n
#           so when performing sweep, each sweep lines should be extended by max tool dimension. \n
#           For computational efficiency, the surface and robot-reach data are discretized to divide the surface. \n
#           Each sweep section coveres square area with side dimension same as the max tool dimension, \n
#           but the section-to-section distance is smaller than the dimension of covered area.
# @param plane_val value along the plane normal axis
# @param ccheck CachedCollisionCheck
# @param tip_dir None, up, down
# @return {approach dir 0~3: {Tsm_key: [idx_div]}}, surface_div_centers
def get_division_dict(surface, brush_face, robot_config, plane_val, tip_dir, TOOL_DIM, ccheck, resolution,
                      sweep_margin=0, io_margin=0.2, xout_cut=False):
    gcheck = ccheck.gcheck
    pscene = gcheck.pscene
    gscene = pscene.gscene
    crob = pscene.combined_robot

    ## divide surface with squares
    rect_div_size_ref = np.max(TOOL_DIM)
    # effective surface dims - surface to divide, except most outside tool dimension
    surface_dim_eff = np.clip(np.subtract(surface.dims[:2], rect_div_size_ref), 0, 1e5)

    div_num_ = np.ceil(surface_dim_eff / rect_div_size_ref).astype(np.int)
    div_num_denom = np.copy(div_num_)
    if np.min(div_num_) == 0:
        div_num_denom[np.where(div_num_ == 0)] = 1  # to resolve numerical singularity
    rect_div_size = surface_dim_eff / div_num_denom  # reduce square size to fit the closed ends
    if np.min(rect_div_size) == 0:
        rect_div_size[np.where(rect_div_size == 0)] = np.max(rect_div_size)  # to resolve numerical singularity
    div_num = div_num_ + 1
    surface_div_centers = [tuple(np.round(-surface_dim_eff / 2 + np.multiply(rect_div_size, (i, j)), 5))
                           for i, j in product(range(div_num[0]), range(div_num[1]))]

    ## make feasible base-div dictionary
    ROBOT_BASE = pscene.robot_chain_dict[robot_config.get_indexed_name()]["link_names"][0]
    TIP_LINK = brush_face.geometry.link_name
    Tbs = surface.get_tf(crob.home_dict)
    Tmr = gcheck.gscene.get_tf(to_link=ROBOT_BASE, from_link=robot_config.root_on, Q=crob.home_dict)
    Trm = SE3_inv(Tmr)
    Rre = SweepDirections.get_dcm_re(tip_dir)
    Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
    rtype = robot_config.type.name
    sweep_path = os.path.join(SWEEP_DAT_PATH, SweepDirections.get_file_name(rtype, tip_dir))
    sweep_max = np.loadtxt(sweep_path + "-max.csv", delimiter=",")
    sweep_min = np.loadtxt(sweep_path + "-min.csv", delimiter=",")

    ## set axes and extract min max sweep range
    ax_pln = 2 if SweepDirections.check_fourway(tip_dir) else 0  # plane axis in robot coord
    ax_swp = 1  # sweep axis in robot coord
    ax_step = [ax for ax in [0, 1, 2] if ax not in [ax_pln, ax_swp]][0]  # step axis = not plane axis nor sweep axis
    if plane_val is not None:
        print("Height Reference: ", plane_val)
        idx_pln = np.argmin(np.abs(sweep_max[:, ax_pln] - plane_val))
        val_pln = sweep_max[idx_pln, ax_pln]
        idc_pln = np.where(sweep_max[:, ax_pln] == val_pln)[0]
        sweep_max = sweep_max[idc_pln, :]
        sweep_min = sweep_min[idc_pln, :]
    else:  # plane_val is not fixed - this means height is not fixed and should be decided by each point
        Tbs = surface.get_tf(crob.home_dict)
        robot_base = crob.get_robot_base_dict()[robot_config.get_indexed_name()]
        Tbr = gscene.get_tf(robot_base, crob.home_dict)
        Trs = np.matmul(SE3_inv(Tbr), Tbs)
        Hoff_et = np.matmul(Rre, Tet[:3, 3])[2]  # Rre*Pet
        div_heights_r = np.matmul(Trs[:3, :2], np.transpose(surface_div_centers)).transpose()[:, 2] + Trs[
            2, 3]  # Prt[2]
        idc_h_matchs = []
        for div_rh in div_heights_r:  # for each division heights, collect according indices
            h_diffs = np.abs(sweep_min[:, 2] + Hoff_et - div_rh)  # sweep_min: Pre
            idx_min = np.argmin(h_diffs)
            if h_diffs[idx_min] > rect_div_size_ref * 0.1:
                continue
            val_min = sweep_min[idx_min, 2]
            idc_min = np.where(sweep_min[:, 2] == val_min)[0]
            idc_h_matchs += list(idc_min)
        idc_h_matchs = sorted(set(idc_h_matchs))
        sweep_min = sweep_min[idc_h_matchs]
        sweep_max = sweep_max[idc_h_matchs]

    ## cut margins at the edge
    sweep_max[:, ax_swp] -= (np.max(TOOL_DIM) / 2 + sweep_margin)
    sweep_min[:, ax_swp] += (np.max(TOOL_DIM) / 2 + sweep_margin)
    idx_ok = np.where(sweep_max[:, ax_swp] > sweep_min[:, ax_swp])[0]
    sweep_max = sweep_max[idx_ok]
    sweep_min = sweep_min[idx_ok]

    ## apply median
    sweep_max[:, ax_swp] = moving_median(sweep_max[:, ax_swp])
    sweep_min[:, ax_swp] = moving_median(sweep_min[:, ax_swp])
    
    ## get all sweep points
    swp_points_dict = {0: [], 1: []}
    for ax_swp_s in range(2):
        div_size_swp = rect_div_size[ax_swp_s]
        div_size_nswp_grid = rect_div_size[(ax_swp_s+1)%2] / 2
        step_points_dict = defaultdict(lambda: defaultdict(list))
        for step_val, pln_val, min_val_ful, max_val_ful in zip(sweep_min[:, ax_step], sweep_min[:, ax_pln],
                                                   sweep_min[:, ax_swp], sweep_max[:, ax_swp]):
            # div sweep range with grid size = div_size/2
            min_grid, max_grid = (np.sign([min_val_ful, max_val_ful]) 
                                  * np.floor(np.abs([min_val_ful, max_val_ful])/(div_size_swp/2))).astype(np.int)
            diff_grid = max_grid - min_grid

            if diff_grid % 2 != 0:  # cut the point with smaller margin if sweep range is not divided by div_size
                if max_val_ful - (max_grid*div_size_swp/2) >= (min_grid*div_size_swp/2) - min_val_ful:
                    max_grid -= 1
                else:
                    min_grid += 1
                diff_grid = max_grid - min_grid
            assert diff_grid % 2 == 0

            min_val, max_val = np.multiply([min_grid, max_grid], div_size_swp/2)
            diff_val = max_val - min_val
            sweep_num = int(diff_grid / 2)

            swp_points = np.zeros((sweep_num, 3))
            swp_points[:, ax_swp] = min_val + np.arange(sweep_num) * div_size_swp
            lv_stp = int(np.round(step_val/div_size_nswp_grid))
            lv_pln = int(np.round(pln_val/div_size_nswp_grid))
            off_val = int(np.round((step_val-lv_stp*div_size_nswp_grid 
                                    if plane_val is not None else
                                    pln_val-lv_pln*div_size_nswp_grid) / resolution))
            swp_points[:, ax_step] = step_val if plane_val is None else div_size_nswp_grid*lv_stp+off_val*resolution
            swp_points[:, ax_pln] = plane_val if plane_val is not None else div_size_nswp_grid*lv_pln+off_val*resolution
            if len(step_points_dict[off_val][(lv_stp, lv_pln)])<len(swp_points):
                step_points_dict[off_val][(lv_stp, lv_pln)] = swp_points
            else:
                step_points_dict[off_val][(lv_stp, lv_pln)] = swp_points
        step_points_list = list(step_points_dict.values())
        if len(step_points_list)>0:
            swp_points_dict[ax_swp_s] = np.concatenate(max(step_points_list, key=lambda lv_dict: np.sum(map(len, lv_dict.values()))).values())
        else:
            swp_points_dict[ax_swp_s] = []

    ## get base-sweep combinations
    div_base_dict = defaultdict(lambda: defaultdict(list))
    Tbm_in_all = []
    Tbm_float_all = []
    Tbm_fail_all = []
    Tbm_succ_all = []
    for i_div, div_center in list(enumerate(surface_div_centers)):  # for each division
        Tsc = SE3(np.identity(3), tuple(div_center) + (surface.dims[2] / 2,))
        for i in range(4) if SweepDirections.check_fourway(tip_dir) else [0, 2]:
            Tct = SE3(Rot_axis(3, i * np.pi / 2), [0, 0, 0])
            Tst = matmul_series(Tsc, Tct)
            Rsr = matmul_series(Tst[:3, :3], Tet[:3, :3].transpose(), Rre.transpose())
            ax_swp_s = np.where(np.abs(Rsr[:3, ax_swp]) > 0.5)[0][0]
            Tbt = np.matmul(Tbs, Tst)
            for i_s, swp_point in enumerate(swp_points_dict[ax_swp_s]):  # for each valid swp points from robot
                Tre = SE3(Rre, swp_point)
                Trt = matmul_series(Tre, Tet)
                Tsr = matmul_series(Tst, SE3_inv(Trt))
                Tsm = np.matmul(Tsr, Trm)  # make base position for each valid swp points from robot
                Tbm = np.matmul(Tbs, Tsm)
                if (np.all(np.abs(Tsm[:3, 3]) < np.divide(surface.dims[:3], 2)+io_margin)
                        or (xout_cut and (np.abs(Tsm[0, 3]) > np.divide(surface.dims[0],
                                                                        2) + io_margin))):  # mobile loc inside surface
                    Tbm_in_all.append(Tbm)
                    continue
                if abs(Tbm[2, 3]) > resolution:
                    Tbm_float_all.append(Tbm)
                    continue
                Tbm[2, 3] = 0
                if ccheck(T_loal=Tbm, Q_dict=crob.home_dict):  # check feasible
                    Tbm_succ_all.append(Tbm)
                    Tsm_xq = T2xyzquat(Tsm)
                    Tsm_key = tuple(np.round(Tsm_xq[0], 3)), tuple(np.round(Tsm_xq[1], 3))
                    div_base_dict[Tsm_key][i].append(i_div)
    #                 gscene.add_highlight_axis("hl", "tbt_{}_{}".format(i_div, i),
    #                                           link_name="base_link",
    #                                           center=Tbt[:3, 3], orientation_mat=Tbt[:3, :3])
    #                 gscene.add_highlight_axis("hl", "tbm_{}_{}_{}".format(i_div, i, i_s),
    #                                           link_name="base_link",
    #                                           center=Tbm[:3, 3], orientation_mat=Tbm[:3, :3])
                else:
                    Tbm_fail_all.append(Tbm)
    #     raise(RuntimeError("error"))
    #     gscene.clear_highlight()
    #     time.sleep(1)
    Tsm_keys = sorted(div_base_dict.keys())
    return div_base_dict, Tsm_keys, surface_div_centers, div_num, (ax_step, ax_swp, ax_pln)


## repeat selecting max-cover base
def select_max_cover_bases(div_base_dict, Tsm_keys, surface_div_centers, div_num, ax_step,
                           test_fun=lambda Tsm, swp_centers:True, lazy_base_thresh=0.1):
    covered_all = []
    idx_bases = []
    idc_divs = []
    test_outputs = []
    bonus_dict = {Tsm_key: 0.0 for Tsm_key in Tsm_keys}
    while True:
        ## make feasible base-div matrix
        Tsm_key_len = len(Tsm_keys)
        div_len = len(surface_div_centers)
        div_base_mat = np.zeros((Tsm_key_len, 4, div_len), dtype=np.float32)
        for i_aproc in range(4):
            for i_tsm, Tsm_key in enumerate(Tsm_keys):
                Tsm = T_xyzquat(Tsm_key)
                if i_aproc not in div_base_dict[Tsm_key]:
                    continue
                idc_div = div_base_dict[Tsm_key][i_aproc]
                idc_div = sorted(set(idc_div) - set(covered_all))
                if len(idc_div) > 0:
                    vec_stp = np.zeros(3)
                    vec_stp[ax_step] = 1 # in robot coords
                    vec_stp = np.matmul(Tsm[:3,:3], vec_stp) # in surface coords
                    ax_stp_ = np.where(np.abs(vec_stp) > 0.5)[0][0] # in surface coords
                    idc_div_rav = np.array(np.unravel_index(idc_div, div_num))
                    u_list, c_list = np.unique(idc_div_rav[ax_stp_, :], return_counts=True) # ax_stp_: which axis is step in ravel? 0:x, 1:y
                    count_dict = {u: c for u, c in zip(u_list, c_list)}
                    for i_div in idc_div:
                        i_div_rav = np.unravel_index(i_div, div_num)
                        div_c = surface_div_centers[i_div]
                        div_base_mat[i_tsm, i_aproc, i_div] = count_dict[i_div_rav[ax_stp_]] + bonus_dict[Tsm_key]*0.1 \
                                                              + 1e-2*len(idc_div) + 1e-4*i_div_rav[ax_stp_]
        ## Select best bases
        div_base_mat_max = np.max(div_base_mat, axis=1)
        div_marks = np.any(div_base_mat_max>0, axis=0)
        if not np.any(div_marks):  ## repeat base selecting untile no div remaining
            break
        base_covers = np.max(div_base_mat_max, axis=1)  # max sweep length
        idx_max = np.argmax(base_covers)
        covereds = np.where(div_base_mat_max[idx_max] == base_covers[idx_max])[0]
        swp_centers = np.array(surface_div_centers)[covereds]

        Tsm_key = Tsm_keys[idx_max]
        xyzquat0 = tuple(Tsm_key[0]) + tuple(Tsm_key[1])
        bonus_keys = np.array(sorted(bonus_dict.keys()))
        bonus_dists = np.array(
            map(lambda xyzquat: np.linalg.norm(np.subtract(tuple(xyzquat[0]) + tuple(xyzquat[1]), xyzquat0)),
                bonus_keys))
        idc_lazy = np.where(bonus_dists <= lazy_base_thresh)[0] # get near ones
        bonus_dists, bonus_keys = bonus_dists[idc_lazy], bonus_keys[idc_lazy]
        bonus_dists, bonus_keys = zip(*sorted(zip(bonus_dists, bonus_keys.tolist()))) # sort by nearest
        if Tsm_key not in bonus_keys: # append Tsm_key if not already in
            bonus_keys = list(bonus_keys)+[Tsm_key]
            bonus_dists = list(bonus_dists)+[0]
        for Tsm_key in bonus_keys:
            Tsm_key = tuple(Tsm_key)
            Tsm = T_xyzquat(Tsm_key)
            test_output = test_fun(Tsm, swp_centers)
            if not test_output:
                Tms = SE3_inv(Tsm)
                swp_centers_m = np.round(np.matmul(Tms[:2, :2], swp_centers.transpose()) + Tms[:2, -1:], 3)
                remove_idx = np.argsort(np.sum(np.abs(swp_centers_m), axis=0).transpose().tolist())[-1]  # select farthest
                to_remove = set([covereds[remove_idx]])
                for i_aproc in range(4):
                    idc_div = div_base_dict[Tsm_key][i_aproc]
                    div_base_dict[Tsm_key][i_aproc] = sorted(set(idc_div) - to_remove)
                continue

            covered_all = sorted(np.concatenate([covered_all, covereds]).astype(np.int))
            idx_bases.append(idx_max)
            idc_divs.append(covereds)
            bonus_dict[Tsm_key] += len(covereds)
            test_outputs.append(test_output)
            break
    return idx_bases, idc_divs, covered_all, test_outputs


##
# @brief    get min/max sweep points for given sweep section centers. \n
#           it is believed that sweep centers are on one sweep line. \n
#           min/max points are expanded to cover edges. \n
#           it is believed that edge margins of max tool dimension were subtracted when sweep centers are generated \n
#           at the first place, in get_division_dict(). so the sweep length should be expanded by max tool dimension. \n
#           for other line-ends inside the suface, there will be some needless expansions but it does not cause problem.
def get_min_max_sweep_points(surface, sweep_centers, TOOL_DIM_MAX, TOOL_DIM_SWEEP, ax_swp):
    swp_min = np.min(sweep_centers, axis=0)
    swp_max = np.max(sweep_centers, axis=0)
    swp_min[ax_swp] = swp_min[ax_swp]-TOOL_DIM_MAX/2+TOOL_DIM_SWEEP/2
    swp_max[ax_swp] = swp_max[ax_swp]+TOOL_DIM_MAX/2-TOOL_DIM_SWEEP/2
    if (swp_max[ax_swp]-swp_min[ax_swp] + TOOL_DIM_SWEEP)>surface.dims[ax_swp]:
        swp_center = np.mean([swp_max[ax_swp], swp_min[ax_swp]])
        swp_min[ax_swp] = swp_center - surface.dims[ax_swp]/2+TOOL_DIM_SWEEP/2
        swp_max[ax_swp] = swp_center + surface.dims[ax_swp]/2-TOOL_DIM_SWEEP/2
    return swp_min, swp_max


from pkg.geometry.geometry import *
from pkg.planning.constraint.constraint_common import *
from pkg.planning.constraint.constraint_actor import *
from pkg.planning.constraint.constraint_subject import *

def add_sweep_task(pscene, sweep_name, surface, swp_min, swp_max, Tsm, wp_dims,
                   color_sweep=(0.6, 0.0, 0.0, 0.3), color_wp=(0.6, 0.0, 0.0, 0.5), tool_dir=1):
    gscene = pscene.gscene
    wp_list = []
    ax_swp = np.where(swp_min!=swp_max)[0][0] # sweep axis
    ax_swp_s = np.where(np.abs(Tsm[:3,ax_swp])>0.5)[0][0]
    sweep_dim = list(wp_dims)
    # print(sweep_dim)
    # print(swp_max)
    # print(swp_min)
    # print(ax_swp)
    # print(wp_dims)
    # print(ax_swp_s)
    sweep_dim[ax_swp_s] = np.subtract(swp_max, swp_min)[ax_swp] + wp_dims[ax_swp_s]
    sweep_dim = tuple(sweep_dim)

    if np.matmul(Tsm[:2,:3].transpose(), swp_min)[ax_swp_s] < np.matmul(Tsm[:2,:3].transpose(), swp_max)[ax_swp_s]:
        swp_0 = swp_max if tool_dir > 0 else swp_min
        swp_1 = swp_min if tool_dir > 0 else swp_max
    else:
        swp_0 = swp_min if tool_dir > 0 else swp_max
        swp_1 = swp_max if tool_dir > 0 else swp_min

    dir_swp_s = np.sign(swp_1-swp_0)
    theta = np.arctan2(dir_swp_s[0], -dir_swp_s[1]) # get angle for y axis
    Rsc = Rot_axis(3, theta if tool_dir else theta + np.pi)

    gscene.create_safe(gtype=GEOTYPE.BOX, name=sweep_name, link_name="base_link",
                       dims=sweep_dim + (surface.dims[2],),
                       center=tuple(np.mean([swp_0, swp_1], axis=0)) + (0,),
                       rpy=Rot2rpy(Rsc), color=color_sweep, display=True,
                       collision=False, fixed=True, parent=surface.name)
    for wp_idx, wp_pos in [(0, swp_0), (1, swp_1)]:
        wp_list.append(gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_wp_{}".format(sweep_name, wp_idx), link_name="base_link",
                                          dims=tuple(wp_dims[:2])+(surface.dims[2],), center=tuple(wp_pos)+(0,),
                                          rpy=Rot2rpy(Rsc), color=color_wp, display=True,
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

def set_base_sweep(pscene, floor_gtem, Tsm, surface, swp_centers, WP_DIMS, TOOL_DIM, Q_dict,
                   ax_swp_tool=1, ax_swp_base=1, tool_dir=1):
    Tbf = floor_gtem.get_tf(Q_dict)
    Tbs = surface.get_tf(Q_dict)
    Tbm = np.matmul(Tbs, Tsm)
    Tfm = np.matmul(SE3_inv(Tbf), Tbm)
    Tfm[2,3] = 0
    wp_task, wp_hdl = add_waypoint_task(pscene, "waypoint", WP_DIMS, Tfm[:3,3], Rot2rpy(Tfm[:3,:3]),
                                        parent=floor_gtem.name)

    TOOL_DIM_SWEEP = TOOL_DIM[ax_swp_tool]
    ax_swp_surf = np.where(np.abs(Tsm[:3,ax_swp_base])>0.5)[0][0]
    swp_min, swp_max = get_min_max_sweep_points(surface, swp_centers, np.max(TOOL_DIM), TOOL_DIM_SWEEP, ax_swp_surf)
    sweep_task = add_sweep_task(pscene, "sweep", surface, swp_min, swp_max, Tsm, wp_dims=TOOL_DIM, tool_dir=tool_dir)

def test_base_divs(ppline, floor_gtem, Tsm, surface, swp_centers, WP_DIMS, TOOL_DIM, Q_dict,
                   timeout=0.3, timeout_loop=3, verbose=False, 
                   multiprocess=True, terminate_on_first=True, 
                   show_motion=False, tool_dir=1):
    pscene = ppline.pscene
    gscene = pscene.gscene
    set_base_sweep(pscene, floor_gtem, Tsm, surface, swp_centers, WP_DIMS, TOOL_DIM, Q_dict, tool_dir=tool_dir)

    ppline.mplan.update_gscene()
    ppline.tplan.prepare()
    initial_state = pscene.initialize_state(dict2list(Q_dict, gscene.joint_names))

    ppline.search(initial_state, [(2, 1)], verbose=verbose,
                  timeout=timeout, timeout_loop=timeout_loop, multiprocess=multiprocess,
                  add_homing=False, terminate_on_first=terminate_on_first, 
                  display=show_motion, post_optimize=False)
    snode_schedule = ppline.tplan.get_best_schedule(at_home=False)
    return snode_schedule


class TestBaseDivFunc:
    def __init__(self, ppline, floor_ws, surface, WP_DIMS, TOOL_DIM, Q_dict, 
                 multiprocess=True, terminate_on_first=True,
                 show_motion=False, highlight_color=(1, 1, 0, 0.5), tool_dir=1, **kwargs):
        self.ppline, self.floor_ws, self.surface = ppline, floor_ws, surface
        self.WP_DIMS, self.TOOL_DIM, self.Q_dict = WP_DIMS, TOOL_DIM, Q_dict
        self.pscene = self.ppline.pscene
        self.gscene = self.pscene.gscene
        self.multiprocess = multiprocess
        self.terminate_on_first = terminate_on_first
        self.show_motion = show_motion
        self.tool_dir = tool_dir
        self.highlight_color = highlight_color
        self.pass_count = 0
        self.highlights = []
        self.kwargs = kwargs

    def __call__(self, Tsm, swp_centers):
        output = test_base_divs(self.ppline, self.floor_ws, Tsm, self.surface, swp_centers,
                                self.WP_DIMS, self.TOOL_DIM, self.Q_dict, 
                                multiprocess=self.multiprocess, terminate_on_first=self.terminate_on_first,
                                show_motion=self.show_motion, tool_dir=self.tool_dir, **self.kwargs)
        if output:
            # leave highlight on cleared area
            swp_fin = self.gscene.copy_from(self.gscene.NAME_DICT["sweep"],
                                            new_name="sweep_tested_{}".format(self.pass_count),
                                            color=self.highlight_color)
            swp_fin.dims = (swp_fin.dims[0], swp_fin.dims[1], swp_fin.dims[2] + 0.002)
            self.gscene.update_marker(swp_fin)
            self.highlights.append(swp_fin)
            self.pass_count += 1
        return output

    def clear(self):
        self.pass_count = 0
        for htem in self.highlights:
            self.gscene.remove(htem)
        self.highlights = []


def refine_order_plan(ppline, snode_schedule_list_in, idx_bases, idc_divs, Qcur,
                      floor_gtem, wayframer, surface, Tsm_keys, surface_div_centers,
                      WP_DIMS, TOOL_DIM, ROBOT_NAME, MOBILE_NAME, HOME_POSE_MOVE, tool_dir=1):
    pscene = ppline.pscene
    mplan = ppline.mplan
    gscene = pscene.gscene
    crob = pscene.combined_robot

    base_div_dict = {idx: triple for idx, triple in enumerate(zip(snode_schedule_list_in, idx_bases, idc_divs))}
    idx_bases_remain = deepcopy(list(base_div_dict.keys()))
    snode_schedule_list = []
    idx_bases_out = []
    idc_divs_out = []
    scene_args_list = []
    scene_kwargs_list = []
    while idx_bases_remain:
        Tbm_cur = wayframer.get_tf_handle(list2dict(Qcur, gscene.joint_names))
        idx_sorted = sorted(
            idx_bases_remain,
            key=lambda x: norm_SE3(np.matmul(SE3_inv(Tbm_cur),
                                             np.matmul(surface.get_tf(Qcur),
                                                       T_xyzquat(Tsm_keys[base_div_dict[x][1]]))
                                            )
                                  ))
            
        snode_schedule, i_b, idc_div = base_div_dict[idx_sorted[0]]
        idx_bases_remain.remove(idx_sorted[0])

        swp_centers = np.array(surface_div_centers)[idc_div]
        Tsm = T_xyzquat(Tsm_keys[i_b])
        Q_dict = list2dict(Qcur, gscene.joint_names)
        set_base_sweep(pscene, floor_gtem, Tsm, surface, swp_centers, WP_DIMS, TOOL_DIM,
                       Q_dict=Q_dict, tool_dir=tool_dir)
        scene_args_list.append((pscene, floor_gtem, Tsm, surface, swp_centers, WP_DIMS, TOOL_DIM))
        scene_kwargs_list.append(dict(Q_dict=Q_dict))

        Qcur_update = np.copy(Qcur)
        to_update_list = []
        to_update_list_all = []
        for snode_pre, snode_nxt in zip(snode_schedule[:-1], snode_schedule[1:]):
            Qpre_saved, Qnxt_saved = snode_pre.state.Q, snode_nxt.state.Q
            rpairs = crob.get_robots_in_act(snode_nxt.traj, skip_not_connected=False)
            idc_move = sorted(np.concatenate(map(lambda rpair: crob.idx_dict[rpair[0]], rpairs)))
            idc_nomove = [i for i in range(gscene.joint_num) if i not in idc_move]
            Qpre_new, Qnxt_new = np.copy(Qcur_update), np.copy(Qnxt_saved) # update Qpre with Qcur
            Qnxt_new[idc_nomove] = Qpre_new[idc_nomove] # update non-moving part of Qnxt with Qcur
            Qcur_update = np.copy(Qnxt_new)
            if (np.sum(np.abs(Qpre_new - Qpre_saved)) > 1e-6 
                    or np.sum(np.abs(Qnxt_new - Qnxt_saved)) > 1e-6): # to_update if new != saved
                to_update_list.append((snode_pre, snode_nxt, Qpre_new, Qnxt_new))
            else:
                if to_update_list:
                    to_update_list_all.append(to_update_list)
                    to_update_list = []
                    
        for to_update_list in to_update_list_all:
            for snode_pre, snode_nxt, Qpre_new, Qnxt_new in to_update_list:
                state_0 = snode_pre.state
                state_0.Q = Qpre_new
                state_0_to = state_0.copy(pscene)
                state_0_to.Q = Qnxt_new
                pscene.set_object_state(state_0)
                mplan.update_gscene()
                Traj, LastQ, error, success, binding_list = mplan.plan_transition(state_0, state_0_to, timeout=1)
                if success:
                    print("update {}th motion".format(snode_schedule.index(snode_nxt)))
                    snode_nxt.set_traj(Traj)
                else:
                    raise(RuntimeError("Fail to update {}".format(snode_schedule.index(snode_nxt))))
        snode_last = snode_schedule[-1]
        ref_state = snode_last.state.copy(pscene)
        ref_state.Q[crob.idx_dict[ROBOT_NAME]] = HOME_POSE_MOVE
        snode_schedule = snode_schedule + ppline.add_return_motion(snode_last, ref_state)
        snode_schedule_list.append(snode_schedule)
        idx_bases_out.append(i_b)
        idc_divs_out.append(idc_div)
#             ppline.play_schedule(snode_schedule) ## To visualize on-refine
        Qcur = np.copy(snode_schedule[-1].state.Q)
        Qcur[3:6] = 0
    return snode_schedule_list, idx_bases_out, idc_divs_out, scene_args_list, scene_kwargs_list

def show_base_div(gscene, surface, surface_div_centers, div_base_dict, Q):
    Tbs = surface.get_tf(Q)
    for i_tsm, (Tsm_key, div_base) in enumerate(div_base_dict.items()):
        show_base_div_lines(gscene, surface, i_tsm, surface_div_centers, Q, Tbs, Tsm_key, div_base)

##
# @param sweep_axis on base
def show_base_div_lines(gscene, surface, i_tsm, surface_div_centers, Q, Tbs, Tsm_key, div_base, sweep_axis="y"):
    Tsm = T_xyzquat(Tsm_key)
    Tbm = np.matmul(Tbs, Tsm)
    gscene.add_highlight_axis("hl", "tbm_{:03}".format(i_tsm), link_name="base_link", center=Tbm[:3,3], orientation_mat=Tbm[:3,:3])
    
    idx_sax = "xyz".index("y".lower())
    axis_swp = np.zeros(3, dtype=np.int)
    axis_swp[idx_sax] = 1
    
    i_line = 0
    for i, idc_div in div_base.items():
        Tct = SE3(Rot_axis(3, i * np.pi / 2), [0, 0, 0])
        P_list = []
        for i_div in idc_div:
            Tsc = SE3(np.identity(3), tuple(surface_div_centers[i_div]) + (surface.dims[2] / 2,))
            Tbt = matmul_series(Tbs, Tsc, Tct)
            P_list.append(Tbt[:3,3])
            gscene.add_highlight_axis("hl", "tbt_{:03}_{}_{}".format(i_tsm, i, i_div), link_name="base_link", 
                                      center=Tbt[:3,3], orientation_mat=Tbt[:3,:3], axis="x")            
        while P_list:
            P0 = P_list.pop(0)
            P_list_ = [P0]
            P_list_bak = []
            while P_list:
                P_ = P_list.pop(0)
                if all([P_[i] == P0[i] for i in range(3) if i!=idx_sax]):
                    P_list_.append(P_)
                else:
                    P_list_bak.append(P_)
            P_list = P_list_bak
            if len(P_list_)>1:
                P1, P2 = P_list_[0], P_list_[-1]
                dP = P2 - P1
                rotvec = calc_rotvec_vecs(axis_swp, dP)
                R = Rotation.from_rotvec(rotvec).as_dcm()
                show_lines(gscene, [(P1, P2)], base_link="base_link", orientation_mat=R, 
                           sweep_axis="y", key="line{:03}_{:02}".format(i_tsm, i_line))
                i_line += 1

def show_lines(gscene, lines, base_link="base_link", orientation_mat=None, sweep_axis="y", key="line"):
    if orientation_mat is None:
        orientation_mat = np.identity(3)
    for i_s, (p_min, p_max) in enumerate(lines):
        gscene.add_highlight_axis("hl", "{}_{}".format(key, i_s), link_name=base_link,
                                 center=p_min, orientation_mat=orientation_mat, axis=sweep_axis,
                                 dims=(np.linalg.norm(np.subtract(p_max, p_min)), 0.05,0.005))
                
