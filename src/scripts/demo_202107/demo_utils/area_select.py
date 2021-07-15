import numpy as np
from enum import Enum
from demo_config import *
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *

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
    MOTION_DEPTH =TABLE_DEPTH - TOOL_DEPTH +MARGIN
    print("== MOTION_DEPTH: {} ==".format(MOTION_DEPTH))
    wipe_depth = MOTION_DEPTH/DEPTH_DIV
    print("== WIIPE_DEPTH: {} ==".format(wipe_depth))
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
                    area = sweep_width, wdepth+TOOL_DEPTH
                    divisions = (int(ceil(TABLE_WIDTH/sweep_width)), DEPTH_DIV)
                    div_num = np.multiply(*divisions)
                    division_dict[depths_test] = (sweep_width, area, range_new, divisions, div_num)
    return division_dict

class Corners(Enum):
    FrontLeft = 0
    FrontRight = 1
    BackLeft = 2
    BackRight = 3


CornerRev = {v.value: v for i, v in enumerate(Corners)}

corner_point_dirs = {Corners.FrontLeft: np.array([-1, 1, 1]),
                     Corners.FrontRight: np.array([-1, -1, 1]),
                     Corners.BackLeft: np.array([1, -1, 1]),
                     Corners.BackRight: np.array([1, 1, 1])}

corner_orientations = {Corners.FrontLeft: np.identity(3),
                       Corners.FrontRight: np.identity(3),
                       Corners.BackLeft: Rot_axis(3, np.pi),
                       Corners.BackRight: Rot_axis(3, np.pi)}

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