import os
import sys
from enum import Enum

from ..aruco.detector import *
from ...geometry.geometry import GEOTYPE

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

detection_level_dict = {'person':0, 'bicycle':1, 'car':2, 'motorcycle':3, 'airplane':4, 'bus':5, 'train':6,
                        'truck':7, 'boat':8, 'traffic light':9, 'fire hydrant':10, 'stop sign':11, 'parking meter':12,
                        'bench':DetectionLevel.ENVIRONMENT, 'bird':14, 'cat':15, 'dog':16, 'horse':17, 'sheep':18,
                        'cow':19, 'elephant':20, 'bear':21, 'zebra':22, 'giraffe':23, 'backpack':24, 'umbrella':25,
                        'handbag':26, 'tie':27, 'suitcase':28, 'frisbee':29, 'skis':30, 'snowboard':31,
                        'sports ball':32, 'kinte':33, 'baseball bat':34, 'baseball glove':35, 'skateboard':36,
                        'surfboard':37, 'tennis racket':38, 'bottle':DetectionLevel.MOVABLE,
                        'wine glass':DetectionLevel.MOVABLE, 'cup':DetectionLevel.MOVABLE, 'fork':42, 'knife':43,
                        'spoon':44, 'bowl':45, 'banana':46, 'apple':47, 'sandwich':48, 'orange':49, 'broccoli':50,
                        'carrot':51, 'hot dog':52, 'pizza':53, 'donut':54, 'cake':55, 'chair':DetectionLevel.ENVIRONMENT,
                        'couch':DetectionLevel.ENVIRONMENT, 'potted plant':58, 'bed':DetectionLevel.ENVIRONMENT,
                        'dining table':DetectionLevel.ENVIRONMENT, 'toilet':DetectionLevel.ENVIRONMENT, 'tv':62,
                        'laptop':DetectionLevel.MOVABLE, 'mouse':DetectionLevel.MOVABLE, 'remote':DetectionLevel.MOVABLE,
                        'keyboard':DetectionLevel.MOVABLE, 'cell phone':DetectionLevel.MOVABLE, 'microwave':68, 'oven':69,
                        'toaster':70, 'sink':DetectionLevel.ENVIRONMENT, 'refrigerator':72, 'book':73,
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



