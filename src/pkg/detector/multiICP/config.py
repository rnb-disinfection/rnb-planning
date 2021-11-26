import os
import sys
import numpy as np
import copy
import open3d as o3d
from enum import Enum

from ..aruco.detector import *
from ...geometry.geometry import GEOTYPE
from ...utils.rotation_utils import *

# Class dictionary for object detection & segmentation
class_dict = {'person':0, 'bicycle':1, 'car':2, 'motorcycle':3, 'airplane':4, 'bus':5, 'train':6,
              'truck':7, 'boat':8, 'traffic light':9, 'fire hydrant':10, 'stop sign':11, 'parking meter':12,
              'bench':13, 'bird':14, 'cat':15, 'dog':16, 'horse':17, 'sheep':18, 'cow':19, 'elephant':20,
              'bear':21, 'zebra':22, 'giraffe':23, 'backpack':24, 'umbrella':25, 'handbag':26, 'tie':27,
              'suitcase':28, 'frisbee':29, 'skis':30, 'snowboard':31, 'sports ball':32, 'kinte':33,
              'baseball bat':34, 'baseball glove':35, 'skateboard':36, 'surfboard':37, 'tennis racket':38,
              'bottle':39, 'wine glass':40, 'cup':41, 'fork':42, 'knife':43, 'spoon':44, 'bowl':45,
              'banana':46, 'apple':47, 'sandwich':48, 'orange':49, 'broccoli':50, 'carrot':51, 'hot dog':52,
              'pizza':53, 'donut':54, 'cake':55, 'chair':56, 'couch':57, 'potted plant':58, 'bed':59,
              'dining table':60, 'toilet':61, 'tv':62, 'laptop':63, 'mouse':64, 'remote':65, 'keyboard':66,
              'cell phone':67, 'microwave':68, 'oven':69, 'toaster':70, 'sink':71, 'refrigerator':72, 'book':73,
              'clock':74, 'vase':75, 'scissors':76, 'teddy bear':77, 'hair drier':78, 'toothbrush':79}


class ObjectInfo(list):
    ##
    # @param name  name of object
    # @param dlevel detection level
    # @param gtype geometry type
    # @param dims  geometry dimensions
    # @param color geometry visualization color
    # @param Toff
    def __init__(self, name, dlevel, gtype=None, dims=None, color=None, Toff=None,
                 scale=[1e-3,1e-3,1e-3], url=None):
        if color is None:
            color = (0.6,0.6,0.6,1)
        self.name, self.dlevel, self.gtype = name, dlevel, gtype
        self.dims, self.color = dims, color
        self.Toff = Toff
        self.scale = scale
        self.url = url

    def get_geometry_kwargs(self):
        ##
        # @brief get kwargs to create geometry item
        return dict(gtype=self.gtype, dims=self.dims, color=self.color,
                    fixed=DetectionLevel.is_fixed(self.dlevel), online=self.dlevel==DetectionLevel.ONLINE,
                    Toff=self.Toff)

def get_obj_info():
    obj_info = {
        'cup': ObjectInfo('cup', dlevel=DetectionLevel.MOVABLE, gtype=GEOTYPE.CYLINDER,
                          dims=(0.4, 0.3, 0.01), color=(0.9, 0.9, 0.9, 0.2),
                          Toff=SE3(np.identity(3), (0, 0, 0)), scale=(1.3,1.3,1.26),
                          url='/home/jhkim/Projects/rnb-planning/release/cup.stl'),

        'table': ObjectInfo('table', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX,
                            dims=(0.4, 0.3, 0.01), color=(0.9, 0.9, 0.9, 0.2),
                            Toff=SE3(np.identity(3), (0,0,0)), scale=(1e-3,1e-3,1e-3),
                            url='/home/jhkim/Projects/rnb-planning/release/table.STL'),

        'bed': ObjectInfo('bed', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX,
                          dims=(0.4,0.3,0.01), color=(0.9,0.9,0.9,0.2),
                          Toff=SE3([[0,1,0],[0,0,1],[1,0,0]], (0.455,0,1.02)), scale=(1e-3,1e-3,1e-3),
                          url='/home/jhkim/Projects/rnb-planning/release/bed.STL'),

        'closet': ObjectInfo('closet', dlevel=DetectionLevel.ENVIRONMENT, gtype=GEOTYPE.BOX,
                             dims=(0.4, 0.3, 0.01), color=(0.9, 0.9, 0.9, 0.2),
                             Toff=SE3([[1,0,0],[0,0,1],[0,-1,0]], (0.3,0,0.2725)), scale=(1e-3,1e-3,1e-3),
                             url='/home/jhkim/Projects/rnb-planning/release/top_table.STL')
    }
    return obj_info


class MaskBox:
    def __init__(self, Toff, dims, include):
        self.Toff, self.dims, self.include = Toff, dims, include

    def get_tf(self, Tparent):
        return self.Toff
        # return np.matmul(Tparent, self.Toff)


class MaskBoxRule:
    def __init__(self, target, parent, merge_rule=np.all):
        self.target = target
        self.parent = parent
        self.box_list = []
        self.merge_rule = merge_rule

    def add_box(self, mbox):
        self.box_list.append(mbox)
        return self

    def apply_rule(self, pcd_in, objectPose_dict):
        pcd_dict = {}
        for oname, To in sorted(objectPose_dict.items()):
            pcd = copy.deepcopy(pcd_in)
            points = np.asarray(pcd.points)
            points4d = np.pad(points, ((0, 0), (0, 1)), 'constant', constant_values=1)
            mask_list = []
            for mbox in self.box_list:
                if self.parent in oname:
                    print("Here")
                    T_bx = mbox.get_tf(To)
                    T_xb = SE3_inv(T_bx)
                    abs_cuts = np.divide(mbox.dims, 2)
                    points_x = np.matmul(points4d, T_xb.transpose())[:, :3]
                    if mbox.include:
                        mask = np.all(np.abs(points_x) < abs_cuts, axis=-1)
                    else:
                        mask = np.any(np.abs(points_x) > abs_cuts, axis=-1)
                    mask_list.append(mask)
            idc = np.where(self.merge_rule(mask_list, axis=0))[0]
            pcd.points = o3d.utility.Vector3dVector(points[idc])
            pcd_dict[oname.replace(self.parent, self.target)] = pcd
        return pcd_dict

def remove_bed(pcd_original, pcd_bed):
    dists = pcd_original.compute_point_cloud_distance(pcd_bed)
    dists = np.asarray(dists)

    idx = np.where(dists > 0.07)[0]
    p_inliers = []
    for i in range(len(idx)):
        p_inliers.append(pcd_original.points[idx[i]])

    return p_inliers

def check_closet_location(pcd_total, pcd_bed, T_bc, T_bo, bed_dims, floor_margin=0.1, visualize=False):
    pcd_total = pcd_total.uniform_down_sample(every_k_points=11)
    if visualize:
        o3d.visualization.draw_geometries([pcd_total])

    # Remove bed
    pcd_top_table = o3d.geometry.PointCloud()
    pcd_top_table.points = o3d.utility.Vector3dVector(remove_bed(pcd_total, pcd_bed))
    if visualize:
        o3d.visualization.draw_geometries([pcd_top_table])

    # Remove other noise
    cl, ind = pcd_top_table.remove_radius_outlier(nb_points=25, radius=0.15)
    pcd_top_table = cl
    if visualize:
        o3d.visualization.draw_geometries([pcd_top_table])

    # Determine rough location of top_table
    points = np.asarray(pcd_top_table.points)
    points_4d = []
    points_temp = []
    points_transformed = []
    for i in range(len(points)):
        points_4d.append(np.hstack([points[i], [1]]))

    for i in range(len(points_4d)):
        points_temp.append(np.matmul(T_bc, points_4d[i]))

    for i in range(len(points_temp)):
        points_transformed.append(np.matmul(np.linalg.inv(T_bo), points_temp[i]))

    points_transformed_np = np.array(points_transformed)[:,:3]

    # Remove background based on bed_vis coord
    out_x = np.where(np.abs(points_transformed_np[:,0])>bed_dims[0]/2)[0]
    out_y = np.where(np.abs(points_transformed_np[:,1])>bed_dims[1]/2+bed_dims[1])[0]
    out_z = np.where(points_transformed_np[:,2]<floor_margin)[0]
    out_all = sorted(set(out_x).union(out_y).union(out_z))
    in_all = sorted(set(np.arange(len(points_transformed_np))) - set(out_all))
    points_transformed = np.array(points_transformed)[in_all, :3]

    if visualize:
        vis_pointcloud_np(points_transformed)

    # Determine closet location by checking num of points
    check_left = 0
    check_right = 0
    for i in range(len(points_transformed)):
        if points_transformed[i][1] > 0:
            check_right += 1
        elif points_transformed[i][1] < 0:
            check_left += 1

    TOP_TABLE_MODE = "LEFT"
    if check_left > check_right:
        TOP_TABLE_MODE = "LEFT"
    else:
        TOP_TABLE_MODE = "RIGHT"

    print("CLOSET ON {}".format(TOP_TABLE_MODE))
    return TOP_TABLE_MODE


def hrule_closet(micp_closet, micp_bed, mrule_closet, Tc=None):
    if Tc is None:
        Tbc = SE3(np.identity(3), (0, 0, 0))
    else:
        Tbc = Tc
    bed_dims = (1.70,0.91,0.01)
    Tbo = np.matmul(Tbc, micp_bed.pose)
    CLOSET_LOCATION = check_closet_location(micp_closet.pcd, micp_bed.pcd, Tbc, Tbo, bed_dims)

    # bed_box
    mrule_closet.add_box(MaskBox(Toff=np.matmul(Tbo, SE3(np.identity(3), (0.02, 0, 0.5))),
                                 dims=(3, 1.6, 1.3), include=False))
    # bed_wall
    mrule_closet.add_box(MaskBox(Toff=np.matmul(Tbo, SE3(np.identity(3), (-1.27, 0, 1.5))),
                                 dims=(0.5, 0.7, 0.3), include=False))
    # floor_box
    mrule_closet.add_box(MaskBox(Toff=SE3(np.identity(3), (0, 0, 0)), dims=(15, 15, 0.4), include=False))

    if CLOSET_LOCATION == "LEFT":
        # bed_left_space
        mrule_closet.add_box(MaskBox(Toff=np.matmul(Tbo, SE3(np.identity(3), (0.02, -0.9, 1))),
                                     dims=(2.5, 1, 3), include=True))
    elif CLOSET_LOCATION == "RIGHT":
        # bed_right_space
        mrule_closet.add_box(MaskBox(Toff=np.matmul(Tbo, SE3(np.identity(3), (0.02, 0.9, 1))),
                                     dims=(2.5, 1, 3), include=True))

    return mrule_closet