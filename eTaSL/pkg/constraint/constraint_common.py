from enum import Enum
from ..utils.rotation_utils import *
from ..geometry.geometry import *
from abc import *
__metaclass__ = type

class ConstraintType(Enum):
    Place = 0
    Frame = 1
    Vacuum = 2
    Grasp2 = 3

OPPOSITE_DICT={
    "top": "bottom",
    "bottom": "top",
    "right": "left",
    "left": "right",
    "front": "back",
    "back": "front"
}

class ActionPoint:
    ctype=None
    def __init__(self, name, _object, point, rpy, name_full=None):
        self.ghnd = GeometryHandle.instance()
        self.name = name
        self.object = _object
        self.set_point_rpy(point, rpy)
        self.name_full = \
            name_full if name_full else "{objname}_{name}".format(objname=self.object.name, name=self.name)

    def set_point_rpy(self, point, rpy):
        self.point = point
        self.rpy_point = rpy
        self.R_point = Rot_rpy(self.rpy_point)
        self.Toff_oh = SE3(self.R_point,self.point or (0,0,0))
        self.update_handle()

    def update_handle(self):
        self.Toff_lh = np.matmul(self.object.Toff, self.Toff_oh)

    def get_tf_handle(self, joint_dict, from_link='base_link'):
        return np.matmul(self.object.get_tf(joint_dict, from_link=from_link), self.Toff_oh)

    @abstractmethod
    def get_redundancy(self):
        pass

def calc_redundancy(redundancy, target):
    point_add = [0,0,0]
    rpy_add = [0,0,0]
    if redundancy:
        for k, v in redundancy.items():
            ax ="xyzuvw".index(k)
            if ax<3:
                point_add[ax] += redundancy[k]
            else:
                rpy_add[ax-3] += redundancy[k]
        if target.point is None: # WARNING: CURRENTLY ASSUME UPPER PLANE
            point_add[2] += target.object.dims[2]/2
    return point_add, rpy_add