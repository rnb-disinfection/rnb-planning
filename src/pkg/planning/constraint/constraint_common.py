from enum import Enum
from copy import deepcopy
import random
from ...geometry.geometry import *
from abc import *
__metaclass__ = type
##
# @class    ConstraintType
# @brief    Constraint type
class ConstraintType(Enum):
    Place = 0
    Frame = 1
    Vacuum = 2
    Grasp2 = 3
    Fixture = 4
    Sweep = 5

OPPOSITE_DICT={
    "top": "bottom",
    "bottom": "top",
    "right": "left",
    "left": "right",
    "front": "back",
    "back": "front"
}

##
# @class    ActionPoint
# @brief    Base class for all constraint action points
# @remark   get_redundancy should be implemented in chlid classes.
class ActionPoint:
    ctype=None
    ##
    # @param    name        action point name
    # @param    geometry    geometry of the action point (rnb-planning.src.pkg.geometry.geometry.GeometryItem)
    # @param    point       action point offset respect to parent geometry
    # @param    rpy         orientation offset respect to parent geometry
    # @param    name_full   constraint full name, mainly for eTaSL constraint definition
    def __init__(self, name, geometry, point, rpy, name_full=None):
        self.gscene = geometry.gscene
        self.name = name
        self.geometry = geometry
        self.set_point_rpy(point, rpy)
        self.name_full = \
            name_full if name_full else "{objname}_{name}".format(objname=self.geometry.name, name=self.name)

    ##
    # @param    point       action point offset respect to parent geometry
    # @param    rpy         orientation offset respect to parent geometry
    def set_point_rpy(self, point, rpy):
        self.point = point
        self.rpy_point = rpy
        self.R_point = Rot_rpy(self.rpy_point)
        self.Toff_oh = SE3(self.R_point,self.point or (0,0,0))
        self.update_handle()

    ##
    # @brief    update Transformation from link (Toff_lf), when parent geometry has moved
    def update_handle(self):
        self.Toff_lh = np.matmul(self.geometry.Toff, self.Toff_oh)

    ##
    # @brief    get handle transformation
    # @param    joint_dict  current joint values as dictionary
    # @param    from_link   refernce link
    def get_tf_handle(self, joint_dict, from_link='base_link'):
        return np.matmul(self.geometry.get_tf(joint_dict, from_link=from_link), self.Toff_oh)

    ##
    # @brief    function prototype to define redundancy of action point
    @abstractmethod
    def get_redundancy(self):
        pass


##
# @brief    calculate redundancy offset
# @param redundancy   redundancy dictionary {direction: value}
# @param action_point action point to which redundancy will be added
def calc_redundancy(redundancy, action_point):
    point_add = [0,0,0]
    rpy_add = [0,0,0]
    if redundancy:
        for k, v in redundancy.items():
            ax ="xyzuvw".index(k)
            if ax<3:
                point_add[ax] += redundancy[k]
            else:
                rpy_add[ax-3] += redundancy[k]
        if action_point.point is None: # WARNING: CURRENTLY ASSUME UPPER PLANE
            point_add[2] += action_point.geometry.dims[2]/2
    return point_add, rpy_add


##
# @brief    combine redundancy of handle and binder
# @param to_ap      handle
# @param to_binder  binder
# @return {point name: {direction: (min, max)}}
def combine_redundancy(to_ap, to_binder):
    redundancy_bd = deepcopy(to_binder.get_redundancy())
    redundancy_ap = deepcopy(to_ap.get_redundancy())
    redundancy_tot = {to_ap.name: redundancy_ap, to_binder.name: redundancy_bd}
    return redundancy_tot


##
# @brief    sample redundancy of handle and binder
# @param redundancy_tot     {point name: {direction: (min, max)}}
# @param sampler            sampling function to be applied to redundancy param (default=random.uniform)
# @return {point name: {direction: sampled value}}
def sample_redundancy(redundancy_tot, sampler=random.uniform):
    return {point: {dir: sampler(*red) for dir, red in redundancy.items()} for point, redundancy in redundancy_tot.items()}


##
# @class MotionConstraint
# @brief definition of fixture constraint
class MotionConstraint:
    def __init__(self, geometry_list, fix_surface, fix_normal, tol=1e-3):
        self.geometry_list, self.fix_surface, self.fix_normal, self.tol = geometry_list, fix_surface, fix_normal, tol


