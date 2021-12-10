import numpy as np
from enum import Enum
from demo_config import *
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from pkg.controller.trajectory_client.trajectory_client import DEFAULT_TRAJ_FREQUENCY
from pkg.utils.traj_utils import *
from collections import defaultdict
import copy

DATASET_DIR = os.path.join(os.environ["RNB_PLANNING_DIR"], 'data/sweep_reach')
try_mkdir(DATASET_DIR)

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
    def get_Re_step_level(cls, tip_dir, sweep_ax):
        assert isinstance(sweep_ax, int), "sweep axis should be int"
        Rre = cls.get_dcm_re(tip_dir)
        Xtool = np.where(np.abs(Rre[:, 0]).astype(int))[0][0]
        Ytool = np.where(np.abs(Rre[:, 1]).astype(int))[0][0]
        Ztool = np.where(np.abs(Rre[:, 2]).astype(int))[0][0]
        level_axes = [Xtool, Ztool]
        if sweep_ax in level_axes:
            level_axes.remove(sweep_ax)
        level_ax = level_axes[0]
        step_ax = list({0, 1, 2} - {sweep_ax, level_ax})[0]
        return Rre, step_ax, level_ax

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
    def __init__(self, mplan, mobile_name, Q_ref):
        self.mplan = mplan
        self.Q_ref = Q_ref
        self.mobile_name = mobile_name
        self.idx_mb = np.array(self.mplan.combined_robot.idx_dict[mobile_name])
        self.cache = {}

    def get_Q(self, T_loal):
        return np.array(list(T_loal[:2, 3]) + [Rot2axis(T_loal[:3,:3], 3)])

    def __call__(self, T_loal):
        Qxyw = self.get_Q(T_loal)
        key = tuple(np.round(Qxyw, 3))
        if key in self.cache:
            return self.cache[key]
        Qtest = np.copy(self.Q_ref)
        Qtest[self.idx_mb[:3]] = Qxyw
        res = self.mplan.validate_trajectory([Qtest])
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
def get_division_dict(pscene, surface, brush_face, robot_config, plane_val, tip_dir, sweep_dir, TOOL_DIM, ccheck, resolution,
                      sweep_margin=0, io_margin=0.2, xout_cut=False, div_num=None):
    gscene = pscene.gscene
    crob = pscene.combined_robot

    ## divide surface with squares
    rect_div_size_ref = np.max(TOOL_DIM)
    # effective surface dims - surface to divide, except most outside tool dimension
    surface_dim_eff = np.clip(np.subtract(surface.dims[:2], rect_div_size_ref), 0, 1e5)

    if div_num is None:
        div_num_ = np.ceil(surface_dim_eff / rect_div_size_ref).astype(np.int)
    else:
        div_num_ = np.subtract(div_num, 1)
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
    Tmr = gscene.get_tf(to_link=ROBOT_BASE, from_link=robot_config.root_on, Q=crob.home_dict)
    Trm = SE3_inv(Tmr)
    Rre = SweepDirections.get_dcm_re(tip_dir)
    Tet = brush_face.get_tf_handle(crob.home_dict, from_link=TIP_LINK)  ## get data
    rtype = robot_config.type.name
    sweep_path = os.path.join(SWEEP_DAT_PATH, SweepDirections.get_file_name(rtype, tip_dir+sweep_dir))
    sweep_max = np.loadtxt(sweep_path + "-max.csv", delimiter=",")
    sweep_min = np.loadtxt(sweep_path + "-min.csv", delimiter=",")

    ax_swp = "XYZ".index(sweep_dir)  # sweep axis in robot coord
    Rre, ax_step , ax_pln = SweepDirections.get_Re_step_level(tip_dir, ax_swp)

    if ax_pln == 2:
        assert plane_val is not None, "Height reference must be provided when plane axis = 2"
        print("Height Reference: ", plane_val)
        idx_pln = np.argmin(np.abs(sweep_max[:, ax_pln] - plane_val))
        val_pln = sweep_max[idx_pln, ax_pln]
        idc_pln = np.where(sweep_max[:, ax_pln] == val_pln)[0]
        sweep_max = sweep_max[idc_pln, :]
        sweep_min = sweep_min[idc_pln, :]
    else:
        ## get all sweep points
        Tbs = surface.get_tf(crob.home_dict)
        robot_base = crob.get_robot_base_dict()[robot_config.get_indexed_name()]
        Tbr = gscene.get_tf(robot_base, crob.home_dict)
        Trs = np.matmul(SE3_inv(Tbr), Tbs)
        Hoff_et = np.matmul(Rre, Tet[:3, 3])[2]  # Rre*Pet
        div_heights_r = np.matmul(
            Trs[:3, :2], np.transpose(surface_div_centers)
        ).transpose()[:, 2] + Trs[2, 3]  # Prt[2]
        if ax_step==2:  # height is not fixed and should be decided by step
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
        elif ax_swp==2:  #  # height is not fixed and should be decided by sweep points
            sweep_min = sweep_min
            sweep_max = sweep_max
        else:
            raise(RuntimeError("Z axis is not decided to be any of sweep, step, plane axes"))


    ## cut margins at the edge
    sweep_max[:, ax_swp] -= (np.max(TOOL_DIM) / 2 + sweep_margin)
    sweep_min[:, ax_swp] += (np.max(TOOL_DIM) / 2 + sweep_margin)
    idx_ok = np.where(sweep_max[:, ax_swp] > sweep_min[:, ax_swp])[0]
    sweep_max = sweep_max[idx_ok]
    sweep_min = sweep_min[idx_ok]

    ## apply median
    sweep_max[:, ax_swp] = moving_median(sweep_max[:, ax_swp])
    sweep_min[:, ax_swp] = moving_median(sweep_min[:, ax_swp])

    swp_points_dict = {0: [], 1: []}
    for ax_swp_s in range(2):
        div_size_swp = rect_div_size[ax_swp_s]
        div_size_nswp_grid = rect_div_size[(ax_swp_s+1)%2] / 2
        step_points_dict = defaultdict(lambda: defaultdict(list))
        sweep_num_list = []
        for step_val, pln_val, min_val_ful, max_val_ful in zip(sweep_min[:, ax_step], sweep_min[:, ax_pln],
                                                   sweep_min[:, ax_swp], sweep_max[:, ax_swp]):
            if ax_swp != 2: # grid divide step with div_size, offset = 0 or div_size/2
            # div sweep range with grid size = div_size/2
                min_grid, max_grid = np.divide([min_val_ful, max_val_ful], (div_size_swp/2))
                min_grid = np.ceil(min_grid).astype(int)
                max_grid = np.floor(max_grid).astype(int)
                diff_grid = max_grid - min_grid # in div_size/2 grid

                if diff_grid % 2 != 0:  # cut the point with smaller margin if sweep range is not divided by div_size
                    if max_val_ful - (max_grid*div_size_swp/2) >= (min_grid*div_size_swp/2) - min_val_ful:
                        max_grid -= 1
                    else:
                        min_grid += 1
                    diff_grid = max_grid - min_grid
                assert diff_grid % 2 == 0

                if diff_grid < 0:
                    continue
                min_val, max_val = np.multiply([min_grid, max_grid], div_size_swp/2) # in real value
                sweep_num = int(diff_grid / 2)+1 # in div_size grid
                sweep_num_list.append(sweep_num)

                swp_points = np.zeros((sweep_num, 3))
                swp_points[:, ax_swp] = min_val + np.arange(sweep_num) * div_size_swp
            else:
                h_in_range = div_heights_r[
                    np.where(np.logical_and(min_val_ful+Hoff_et< div_heights_r,
                                            div_heights_r < max_val_ful+Hoff_et))]-Hoff_et
                if len(h_in_range)==0:
                    continue
                swp_points = np.zeros((len(h_in_range), 3))
                swp_points[:, ax_swp] = h_in_range

            diff_val = np.max(swp_points[:, ax_swp]) - np.min(swp_points[:, ax_swp])
            lv_stp = int(np.round(step_val/div_size_nswp_grid))
            lv_pln = int(np.round(pln_val/div_size_nswp_grid))
            off_val = int(np.round((step_val-lv_stp*div_size_nswp_grid
                                    if plane_val is not None else
                                    pln_val-lv_pln*div_size_nswp_grid) / resolution))
            swp_points[:, ax_step] = step_val if ax_step==2 else div_size_nswp_grid*lv_stp+off_val*resolution
            swp_points[:, ax_pln] = plane_val if plane_val is not None else div_size_nswp_grid*lv_pln+off_val*resolution
            if len(step_points_dict[off_val][(lv_stp, lv_pln)])<len(swp_points):
                step_points_dict[off_val][(lv_stp, lv_pln)] = swp_points
        step_points_list = list(step_points_dict.values())
        if len(step_points_list)>0:
            swp_points_dict[ax_swp_s] = np.concatenate(max(step_points_list, key=lambda lv_dict: np.sum(map(len, lv_dict.values()))).values())
        else:
            swp_points_dict[ax_swp_s] = []

    ## get base-sweep combinations
    div_base_dict = defaultdict(lambda: defaultdict(set))
    Tbm_in_all = []
    Tbm_float_all = []
    Tbm_fail_all = []
    Tbm_succ_all = []
    Tsm_swp_pairs = []
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
                if ccheck(T_loal=Tbm):  # check feasible
                    Tsm_swp_pairs.append((Tsm, swp_point))
                    Tbm_succ_all.append(Tbm)
                    Tsm_xq = T2xyzquat(Tsm)
                    Tsm_key = tuple(np.round(Tsm_xq[0], 3)), tuple(np.round(Tsm_xq[1], 3))
                    div_base_dict[Tsm_key][i].add(i_div)
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
    for k, v in div_base_dict.items():
        for kk, vv in v.items():
            div_base_dict[k][kk] = list(sorted(vv))
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


def add_sweep_task(pscene, sweep_name, surface, swp_min, swp_max, Tsm, ax_swp_t, wp_dims,
                   color_sweep=(0.6, 0.0, 0.0, 0.3), color_wp=(0.6, 0.0, 0.0, 0.5), tool_dir=1):
    gscene = pscene.gscene
    wp_list = []
    ax_swp_s = np.where(swp_min != swp_max)[0][0]  # sweep axis in surface
    ax_swp = np.where(np.abs(Tsm[ax_swp_s, :3]) > 0.5)[0][0]
    sweep_dim = list(wp_dims)
    sweep_dim[ax_swp_t] = np.subtract(swp_max, swp_min)[ax_swp_s] + wp_dims[ax_swp_t]
    sweep_dim = tuple(sweep_dim)

    if np.matmul(Tsm[:2, :3].transpose(), swp_min)[ax_swp] < np.matmul(Tsm[:2, :3].transpose(), swp_max)[ax_swp]:
        swp_0 = swp_max # if tool_dir > 0 else swp_min
        swp_1 = swp_min # if tool_dir > 0 else swp_max
    else:
        swp_0 = swp_min # if tool_dir > 0 else swp_max
        swp_1 = swp_max # if tool_dir > 0 else swp_min

    dir_swp_s = np.sign(swp_1 - swp_0)
    dir_swp_t = np.zeros(2)
    dir_swp_t[ax_swp_t] = tool_dir

    theta = calc_rotvec_vecs(dir_swp_t, dir_swp_s) + np.pi  # get angle for tool sweep axis
    Rsc = Rot_axis(3, theta)

    gscene.create_safe(gtype=GEOTYPE.BOX, name=sweep_name, link_name="base_link",
                       dims=sweep_dim + (surface.dims[2],),
                       center=tuple(np.mean([swp_0, swp_1], axis=0)) + (0,),
                       rpy=Rot2rpy(Rsc), color=color_sweep, display=True,
                       collision=False, fixed=True, parent=surface.name)
    for wp_idx, wp_pos in [(0, swp_0), (1, swp_1)]:
        wp_list.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_wp_{}".format(sweep_name, wp_idx), link_name="base_link",
                               dims=tuple(wp_dims[:2]) + (surface.dims[2],), center=tuple(wp_pos) + (0,),
                               rpy=Rot2rpy(Rsc), color=color_wp, display=True,
                               collision=False, fixed=True, parent=surface.name))
    sweep_task = pscene.create_subject(oname=sweep_name, gname=sweep_name, _type=SweepLineTask,
                                       action_points_dict={
                                           wp.name: SweepFrame(wp.name, wp, [0, 0, wp.dims[2] / 2], [0, 0, 0])
                                           for wp in wp_list})
    return sweep_task

def set_sweep(pscene, surface, Tsm, swp_centers, ax_swp_tool, ax_swp_base,
                   TOOL_DIM, tool_dir=1):
    TOOL_DIM_SWEEP = TOOL_DIM[ax_swp_tool]
    ax_swp_surf = np.where(np.abs(Tsm[:3,ax_swp_base])>0.5)[0][0]
    swp_min, swp_max = get_min_max_sweep_points(surface, swp_centers, np.max(TOOL_DIM), TOOL_DIM_SWEEP, ax_swp_surf)
    sweep_task = add_sweep_task(pscene, "sweep", surface, swp_min, swp_max, Tsm, ax_swp_tool, wp_dims=TOOL_DIM, tool_dir=tool_dir)

def test_base_divs(ppline, surface, Tsm, swp_centers,
                   ax_swp_tool, ax_swp_base, TOOL_DIM, Q_dict,
                   timeout=0.3, timeout_loop=3, verbose=False,
                   multiprocess=True, max_solution_count=1,
                   show_motion=False, tool_dir=1, **kwargs):
    pscene = ppline.pscene
    gscene = pscene.gscene
    set_sweep(pscene, surface, Tsm, swp_centers, ax_swp_tool, ax_swp_base,
                   TOOL_DIM, tool_dir=tool_dir)

    ppline.mplan.update_gscene()
    ppline.tplan.prepare()
    initial_state = pscene.initialize_state(dict2list(Q_dict, gscene.joint_names))

    ppline.search(initial_state, [(2,)], verbose=verbose,
                  timeout=timeout, timeout_loop=timeout_loop, multiprocess=multiprocess,
                  add_homing=False, max_solution_count=max_solution_count, 
                  display=show_motion, post_optimize=False, **kwargs)
    snode_schedule = ppline.tplan.get_best_schedule(at_home=False)
    return snode_schedule

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


class GreedyExecuter:
    def __init__(self, ppline, brush_face, tool_dim, Qhome=None, drift=None):
        self.ppline, self.brush_face, self.tool_dim = ppline, brush_face, tool_dim
        self.pscene = self.ppline.pscene
        self.gscene = self.pscene.gscene
        self.mplan = self.ppline.mplan
        self.crob = self.pscene.combined_robot
        if Qhome is not None:
            self.Qhome = Qhome
        else:
            self.Qhome = self.crob.get_real_robot_pose()
        for rconfig in self.crob.robots_on_scene:
            if rconfig.type == rconfig.type.kmb:
                self.mobile_config = rconfig
            else:
                self.robot_config = rconfig
        self.robot_name = self.robot_config.get_indexed_name()
        self.mobile_name = self.mobile_config.get_indexed_name()
        self.mobile_link = self.robot_config.root_on
        self.kmb = self.crob.robot_dict[self.mobile_name]
        self.idx_rb = self.crob.idx_dict[self.robot_name]
        self.idx_mb = self.crob.idx_dict[self.mobile_name]
        self.ccheck = CachedCollisionCheck(self.mplan, self.mobile_name, Qhome)
        self.pass_count = 0
        self.highlights = []
        self.vel_lims = 0.5
        self.acc_lims = 0.5
        if drift is None:
            self.drift = np.zeros(len(self.gscene.joint_names))
        else:
            self.drift = drift

    def get_division_dict(self, surface, tip_dir, sweep_dir, plane_val,
                          xout_cut=False, resolution=0.02, div_num=None):
        self.surface = surface
        self.tip_dir, self.sweep_dir, self.plane_val = tip_dir, sweep_dir, plane_val
        self.div_base_dict, self.Tsm_keys, self.surface_div_centers, self.div_num, \
        (self.ax_step, self.ax_swp, self.ax_pln) = \
            get_division_dict(self.pscene, self.surface, self.brush_face, self.robot_config,
                              plane_val=plane_val, tip_dir=tip_dir, sweep_dir=sweep_dir,
                              TOOL_DIM=self.tool_dim, ccheck=self.ccheck,
                              resolution=resolution, xout_cut=xout_cut, div_num=div_num)

        self.ax_swp_base = self.ax_swp
        self.Rre = SweepDirections.get_dcm_re(tip_dir)
        self.Tet = self.brush_face.get_tf_handle(self.Qhome, from_link=self.brush_face.geometry.link_name)
        self.Rrt = np.matmul(self.Rre, self.Tet[:3, :3])
        self.ax_swp_tool = np.where(np.abs(self.Rrt.transpose()[:, self.ax_swp_base]).astype(np.int))[0][0]

    def remove_covered(self, covered):
        self.covered = set(covered)
        self.remains = set(range(len(self.surface_div_centers))) - self.covered

        if self.covered and self.remains:
            div_base_dict_remains = defaultdict(lambda: defaultdict(list))
            for k, div_dict in self.div_base_dict.items():
                for i, divs in div_dict.items():
                    divs = list(set(divs) - self.covered)
                    if divs:
                        div_base_dict_remains[k][i] = divs
            self.div_base_dict = div_base_dict_remains
            self.Tsm_keys = sorted([tkey for tkey in self.Tsm_keys if tkey in self.div_base_dict])

    def set_test_kwargs(self, **kwargs):
        self.test_kwargs = kwargs
        self.test_clear()

    def test_clear(self):
        self.pass_count = 0
        for htem in self.highlights:
            self.gscene.remove(htem)
        self.highlights = []

    def test_base_divs(self, Qcur, Tsm, swp_centers, tool_dir):
        if not isinstance(Qcur, dict):
            Qcur = list2dict(Qcur, self.gscene.joint_names)
        output = test_base_divs(self.ppline, self.surface, Tsm, swp_centers,
                                self.ax_swp_tool, self.ax_swp_base, self.tool_dim, Qcur,
                                tool_dir=tool_dir, **self.test_kwargs)
        if output:
            # leave highlight on cleared area
            swp_fin = self.gscene.copy_from(self.gscene.NAME_DICT["sweep"],
                                            new_name="sweep_tested_{}".format(self.pass_count),
                                            color=(1, 1, 0, 0.5))
            swp_fin.dims = (swp_fin.dims[0], swp_fin.dims[1], swp_fin.dims[2] + 0.002)
            self.gscene.update_marker(swp_fin)
            self.highlights.append(swp_fin)
            self.pass_count += 1
        return output

    def init_base_divs(self, Qcur):
        Tms0 = self.surface.get_tf(Qcur, from_link=self.mobile_link)
        self.div_dists = np.array(
            [np.linalg.norm(np.matmul(Tms0[:3, :2], ct) + Tms0[:3, 3]) for ct in self.surface_div_centers])
        Tsm_key_len = len(self.Tsm_keys)
        div_len = len(self.surface_div_centers)
        self.div_base_mat = np.zeros((Tsm_key_len, 4, div_len), dtype=np.float32)
        for i_t, tkey in enumerate(self.Tsm_keys):
            for i_ap, idc_divs in self.div_base_dict[tkey].items():
                self.div_base_mat[i_t, i_ap, idc_divs] = 100 # - self.div_dists[idc_divs]*0.1

    def get_best_base_divs(self, Qcur):
        Tbs = self.surface.get_tf(Qcur)
        dist_scores = np.linalg.norm(
            np.array([np.matmul(Tbs, T_xyzquat(tkey))[:2,3] for tkey in self.Tsm_keys]) - Qcur[:2],
            axis=-1)*0.2
        score_mat = np.sum(self.div_base_mat, axis=-1)
        score_mat -= dist_scores[:, np.newaxis]
        max_score = np.max(score_mat)
        if max_score < 1e-5:
            return None, None, []
        i_t, i_ap = np.transpose(np.where(score_mat == max_score))[0]
        tkey = self.Tsm_keys[i_t]
        return tkey, i_ap, np.where(self.div_base_mat[i_t, i_ap, :]>0)[0]

    def mark_tested(self, tkey, i_ap, idc_succs, idc_fails):
        if len(idc_succs) > 0:
            self.div_base_mat[:, :, idc_succs] = 0
        if len(idc_fails) > 0:
            i_t = self.Tsm_keys.index(tkey)
            self.div_base_mat[i_t, i_ap, idc_fails] = 0

    def get_mobile_Q(self, tkey, Qcur):
        Tbs = self.surface.get_tf(Qcur)
        Tsm = T_xyzquat(tkey)
        Tbm = np.matmul(Tbs, Tsm)
        Qmob = list(Tbm[:2, 3]) + [Rot2axis(Tbm[:3, :3], 3), 0, 0, 0]
        return Qmob

    def force_add_return(self, snode_schedule, Qhome=None):
        # add return motion and execute
        homing_stack = []
        for i in range(1, len(snode_schedule)):
            snode_cur = snode_schedule[-i]
            initial_state = snode_schedule[0].state.copy(self.pscene)
            initial_state.Q[self.idx_rb] = Qhome[self.idx_rb]
            homing = self.ppline.add_return_motion(snode_cur,
                                                   initial_state=initial_state,
                                                   timeout=1, try_count=3)
            if len(homing) > 0:
                for hnode in homing:
                    homing_stack += list(hnode.traj)
                    homing_stack = np.array(homing_stack)
                self.homing_stack = homing_stack
                hnode_full = snode_schedule[-1].copy(self.pscene)
                hnode_full.set_traj(homing_stack)
                hnode_full.state.Q = homing_stack[-1]
                self.hnode_full = hnode_full
                snode_schedule.append(hnode_full)
                break
            else:
                homing_stack += list(reversed(snode_cur.traj))
        return snode_schedule

    def greedy_execute(self, Qcur, tool_dir, mode_switcher, offset_fun, auto_clear_subject=True, cost_cut=110, covereds=[],
                       repeat_sweep=2, adjust_once=True):
        gtimer = GlobalTimer.instance()
        Qcur = np.copy(Qcur)
        Qhome = np.copy(Qcur)
        # # MAKE LOOP BELOW
        snode_schedule_list = []
        covereds = deepcopy(covereds)
        self.mark_tested(None, None, covereds, [])
        while True:
            with gtimer.block("get_best_base_divs"):
                # get current base base
                tkey, i_ap, idc_divs = self.get_best_base_divs(Qcur)

                if tkey is None or len(idc_divs) == 0:
                    break

                # move base and get real pose
                Qmob = self.get_mobile_Q(tkey, Qcur)
                Qref = np.array(list(Qmob) + list(Qcur[self.idx_rb]))
                T_bm = self.gscene.get_tf(self.mobile_link, Qref)
                if ((not self.ccheck(T_loal=T_bm))
                        or (not self.ccheck(T_loal=self.gscene.get_tf(self.mobile_link, np.add(Qref, self.drift))))
                        or (not self.kmb.check_valid(Qref, cost_cut))):
                    self.mark_tested(tkey, i_ap, [], idc_divs)
                    TextColors.RED.println("[PLAN] Skip {} - collision base position ({} / {})".format(
                        tkey, np.round(self.kmb.coster(Qref)), cost_cut))
                    print("Drift = {}".format(np.round(self.drift, 2)))
                    continue

            print("Drift = {}".format(np.round(self.drift, 2)))
            with gtimer.block("move_base"):
                self.kmb.joint_move_make_sure(np.subtract(Qmob, (self.drift[self.idx_mb] / 2)))

            with gtimer.block("offset_fun"):
                try:
                    Qcur, Qtar = offset_fun(self, self.crob, self.mplan, self.robot_name, Qref)
                except Exception as e:
                    TextColors.RED.println("[PLAN] Error in offset fun")
                    print(e)
                    self.mark_tested(tkey, i_ap, [], idc_divs)
                    continue

#                 self.drift = np.mean([np.subtract(Qcur, Qref), self.drift], axis=0)
#                 self.drift[self.idx_mb[2]] = (self.drift[2] + np.pi) % (np.pi * 2) - np.pi
#                 self.drift[self.idx_rb] = 0
                self.drift[:] = 0
                Tbm_cur = self.gscene.get_tf(self.mobile_link, Qcur)
                Tbs = self.surface.get_tf(Qcur)
                Tsm_cur = np.matmul(SE3_inv(Tbs), Tbm_cur)
                tkey_cur_exact = T2xyzquat(Tsm_cur, decimals=2)
                tkey_cur_exact = (tkey_cur_exact[0], tkey_cur_exact[1][2:])

            if adjust_once:
                with gtimer.block("adjust_once"):
                    TextColors.BLUE.println("[PLAN] Adjust base once. {} / {}".format(tkey, tkey_cur_exact))
                    TextColors.BLUE.println("[PLAN] Qcur: {}".format(np.round(Qcur[:3], 3)))
                    TextColors.BLUE.println("[PLAN] Qref: {}".format(np.round(Qref[:3], 3)))
                    TextColors.BLUE.println("[PLAN] tar: {}".format(np.round(Qtar[:3], 3)))
                    Qmob_new = np.copy(Qmob)
                    Qmob_new[:2] = Qtar[:2]
                    self.kmb.joint_move_make_sure(Qmob_new, check_valid=0)

                with gtimer.block("update_adjusted_offset"):
                    try:
                        Qref[self.idx_rb] = Qcur[self.idx_rb]
                        Qcur, Qtar = offset_fun(self, self.crob, self.mplan, self.robot_name, Qref)

                        # self.drift = np.mean([np.subtract(Qcur, Qref), self.drift], axis=0)
                        # self.drift[self.idx_mb[2]] = (self.drift[2] + np.pi) % (np.pi * 2) - np.pi
                        # self.drift[self.idx_rb] = 0
                        self.drift[:] = 0
                        Tbm_cur = self.gscene.get_tf(self.mobile_link, Qcur)
                        Tbs = self.surface.get_tf(Qcur)
                        Tsm_cur = np.matmul(SE3_inv(Tbs), Tbm_cur)
                        tkey_cur_exact = T2xyzquat(Tsm_cur, decimals=2)
                        tkey_cur_exact = (tkey_cur_exact[0], tkey_cur_exact[1][2:])
                    except Exception as e:
                        TextColors.RED.println("[PLAN] Error in offset fun")
                        print(e)
                        continue

            with gtimer.block("update_base_offset"):
                Tdiff_list = []
                for i_t, _tkey in enumerate(self.Tsm_keys):
                    Tsm = T_xyzquat(_tkey)
                    Tdiff = np.linalg.norm(np.matmul(SE3_inv(Tsm_cur), Tsm) - np.identity(4))
                    if np.sum(self.div_base_mat[i_t][i_ap]) < 1:
                        Tdiff = 100
                    Tdiff_list.append(Tdiff)
                i_min = np.argmin(Tdiff_list)
                tkey_cur = self.Tsm_keys[i_min]
                if tkey_cur != tkey:
                    TextColors.RED.println(
                        "[PLAN] Current position is closer to other Tsm_key. \n Try switch {} ({}) -> {} ({}) / {}".format(
                            tkey, i_ap, tkey_cur, i_ap, tkey_cur_exact))
                    print("Drift = {}".format(np.round(self.drift, 2)))
                    idc_divs_cur = self.div_base_dict[tkey_cur][i_ap]
                    idc_divs_cur = list(set(idc_divs_cur) - set(covereds))
                    if len(idc_divs_cur) == 0:
                        TextColors.BLUE.println("[PLAN] Switched location has no divs. Keep current one")
                    else:
                        tkey, i_ap, idc_divs = tkey_cur, i_ap, idc_divs_cur

            with gtimer.block("planning_all"):
                snode_schedule_all = []
                idc_succs = []
                idc_fails = []

                Tsm = T_xyzquat(tkey)
                vec_stp = np.identity(3)[:, self.ax_step]  # in robot coords
                vec_stp = np.matmul(Tsm[:3, :3], vec_stp)  # in surface coords
                ax_stp_ = np.where(np.abs(vec_stp) > 0.5)[0][0]  # in surface coords
                idc_div_rav = np.array(np.unravel_index(idc_divs, self.div_num))
                u_list, c_list = np.unique(idc_div_rav[ax_stp_, :],
                                           return_counts=True)  # ax_stp_: which axis is step in ravel? 0:x, 1:y
                for uval in u_list:
                    i_line = np.where(idc_div_rav[ax_stp_] == uval)[0]
                    idc_line = list(np.asarray(idc_divs)[i_line])
                    print("[PLAN] Line idc {}".format(idc_line))
                    idc_divs_remain = copy.deepcopy(idc_line)
                    idc_select_failed = []
                    while len(idc_divs_remain) > 0:
                        snode_schedule = []
                        for idc_select in reversed(list(powerset(idc_divs_remain))):
                            if len(idc_select) == 0:
                                # no area
                                break
                            if tuple(sorted(idc_select)) in idc_select_failed:
                                continue
                            idc_divs_remain_ = list(set(idc_divs_remain) - set(idc_select))
                            if np.any(np.logical_and(np.min(idc_select) < idc_divs_remain_,
                                                     idc_divs_remain_ < np.max(idc_select))):
                                # non-connected areas
                                continue
                            print("[PLAN] Try idc {}".format(idc_select))
                            with gtimer.block("test_base_divs"):
                                snode_schedule = self.test_base_divs(Qcur, Tsm,
                                                                     swp_centers=[self.surface_div_centers[i_div]
                                                                                  for i_div in idc_select],
                                                                     tool_dir=tool_dir)
                            if len(snode_schedule) > 0:
                                for snode_pre, snode_to in zip(snode_schedule[:-1], snode_schedule[1:]):
                                    if snode_pre.state.node == (1,) and snode_to.state.node == (2,):
                                        traj = list(snode_to.traj)
                                        for _ in range(repeat_sweep):
                                            traj += list(reversed(snode_to.traj))[1:] + list(snode_to.traj)[1:]
#                                         t_all, traj = calc_safe_trajectory(1.0/DEFAULT_TRAJ_FREQUENCY, 
#                                                                                np.array(traj),
#                                                                                self.vel_lims, self.acc_lims)
                                        traj = np.array(traj)
                                        snode_to.set_traj(traj)
                                idc_divs_remain = sorted(set(idc_divs_remain) - set(idc_select))
                                idc_succs += idc_select
                                snode_schedule_all += snode_schedule
                                Qcur = snode_schedule[-1].state.Q
                                break
                            else:
                                idc_select_failed.append(tuple(sorted(idc_select)))
                        if len(snode_schedule) == 0:  # no more available case in idc_idvs_remain
                            idc_fails += idc_divs_remain
                            break

                self.mark_tested(tkey, i_ap, idc_succs, idc_fails)
                covereds += idc_succs

            with gtimer.block("execution"):
                if len(snode_schedule_all) > 0:  # no more available case in idc_idvs_remain
                    snode_schedule_all = self.force_add_return(snode_schedule_all, Qhome=Qhome)
                    Qcur = snode_schedule_all[-1].state.Q
                    self.ppline.execute_schedule(snode_schedule_all, one_by_one=True, mode_switcher=mode_switcher)
                    snode_schedule_list.append(snode_schedule_all)
                if len(snode_schedule_list) > 0:
                    if len(snode_schedule_list[-1]) > 0:
                        Qcur = snode_schedule_list[-1][-1].traj[-1]
        if auto_clear_subject:
            self.remove_sweep()
        return snode_schedule_list, Qcur, sorted(set(covereds))

    def remove_sweep(self):
        self.pscene.clear_subjects()
        for child in copy.deepcopy(self.surface.children):
            self.gscene.remove(self.gscene.NAME_DICT[child])

