import numpy as np
from enum import Enum
from demo_config import *
from pkg.utils.rotation_utils import *

dataset = np.load('scripts/demo_202107/dataset.npy', allow_pickle=True)


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
            raise (RuntimeError("TOO DEEP TABLE! CAN'T FINISH WIPING TASK"))
        else:
            area_depth_obj = TABLE_DEPTH / 2

    # select optimal item for the area depth
    idx_depth = np.min(np.where(np.subtract(opt_depth_sort, area_depth_obj) > 0)[0])
    area_depth = opt_depth_sort[idx_depth]
    area_corner1, area_corner2, area_width, _ = optimal_dict[area_depth]

    # fit table width to table
    if TABLE_WIDTH < area_width:
        area_width = TABLE_WIDTH

    # get area parameters
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