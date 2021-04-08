from enum import Enum
from copy import deepcopy
import random
from ...geometry.geometry import *
from abc import *
from ...utils.rotation_utils import SE3_inv
from scipy.spatial.transform import Rotation

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


OPPOSITE_DICT = {
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
    ctype = None

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
        self.Toff_oh = SE3(self.R_point, self.point if self.point is not None else (0, 0, 0))
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
    point_add = [0, 0, 0]
    rpy_add = [0, 0, 0]
    if redundancy:
        for k, v in redundancy.items():
            ax = "xyzuvw".index(k)
            if ax < 3:
                point_add[ax] += redundancy[k]
            else:
                rpy_add[ax - 3] += redundancy[k]
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
    return {point: {dir: sampler(*red) for dir, red in redundancy.items()} for point, redundancy in
            redundancy_tot.items()}



##
# @brief    calculate margins for binding between two ActionPoints
# @param handle_T   transformation matrix (4x4) to global coordinate for handle
# @param binder_T   transformation matrix (4x4) to global coordinate for binder
# @param handle_redundancy  redundancy for handle {axis: (min, max)}, where axis in "xyzuvw"
# @param binder_redundancy  redundancy for binder {axis: (min, max)}, where axis in "xyzuvw"
# @return 6x2 matrix that contains margin on the mininum/maximum side, in "xyz+rotationvector" order
def get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy):
    T_rel = np.matmul(SE3_inv(binder_T), handle_T)

    # get min/max offset in the handle coordinate
    min_handle_vec = np.zeros(3)
    max_handle_vec = np.zeros(3)
    for i_ax, k_ax in enumerate("xyz"):
        if k_ax in handle_redundancy:
            min_val, max_val = handle_redundancy[k_ax]
        else:
            min_val, max_val = 0, 0
        min_handle_vec[i_ax] = min_val
        max_handle_vec[i_ax] = max_val

    pos_vec = T_rel[:3, 3]
    margin_vec = np.zeros((3, 2))
    for i_ax, k_ax in enumerate("xyz"):
        # get min/max offset in the binder coordinate
        if k_ax in binder_redundancy:
            min_val, max_val = binder_redundancy[k_ax]
        else:
            min_val, max_val = 0, 0

        # transform handle coordinate min/max to binder coordinate
        # - this reshapes object's margin to rectangular area in binder coordinate: not accurate
        min_vec, max_vec = T_rel[i_ax, :3] * min_handle_vec, T_rel[i_ax, :3] * max_handle_vec
        max_off = np.sum(np.maximum(min_vec, max_vec))
        min_off = np.sum(np.minimum(min_vec, max_vec))
        margin_vec[i_ax] = [pos_vec[i_ax] - (min_val + min_off), (max_val + max_off) - pos_vec[i_ax]]

    # get min/max offset in the handle coordinate
    handle_rot_range = np.zeros((3, 2))
    binder_rot_range = np.zeros((3, 2))
    for i_ax, k_ax in enumerate("uvw"):
        if k_ax in handle_redundancy:
            handle_rot_range[i_ax] = handle_redundancy[k_ax]
        if k_ax in binder_redundancy:
            binder_rot_range[i_ax] = binder_redundancy[k_ax]
    handle_rot_axes = np.where(np.linalg.norm(handle_rot_range, axis=-1) > 0)[0]
    binder_rot_axes = np.where(np.linalg.norm(binder_rot_range, axis=-1) > 0)[0]
    assert len(set(handle_rot_axes).union(
        binder_rot_axes)) <= 1, "Rotation redundancy is allowed for only single axis, in same axis in both binder and handle"

    rot_range = handle_rot_range + binder_rot_range
    rot_vec = Rotation.from_dcm(T_rel[:3, :3]).as_rotvec()
    margin_rot = np.stack([rot_vec - rot_range[:, 0], rot_range[:, 1] - rot_vec], axis=-1)

    return np.concatenate([margin_vec, margin_rot])

##
# @brief fit binding
# @param    obj     AbstractObject to update location to fit binding
# @param    handle  handle to match with binder
# @param    binder  binder to match with handle
# @param    Q_dict  current joint configuration in dictionary
# @return
def fit_binding(obj, handle, binder, Q_dict):
    handle_T = handle.get_tf_handle(Q_dict)
    binder_T = binder.get_tf_handle(Q_dict)
    handle_redundancy = handle.get_redundancy()
    binder_redundancy = binder.get_redundancy()
    margin_mat = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy)

    offset_loc = np.maximum(-margin_mat[:, 0], 0) + np.minimum(margin_mat[:, 1], 0)
    obj_geo = obj.geometry
    T_glink = get_tf(obj_geo.link_name, Q_dict, obj_geo.gscene.urdf_content)

    R_lgb = np.matmul(T_glink[:3, :3].transpose(), binder_T[:3, :3])    # orienation of binder from geometry link
    pos_off = np.matmul(R_lgb, offset_loc[:3])

    if np.linalg.norm(offset_loc[3:]) > 1e-2:
        R_bh = np.matmul(binder_T[:3, :3].transpose(), handle_T[:3, :3])
        rvec_bh = Rotation.from_dcm(R_bh).as_rotvec()
        R_bh_new = Rotation.from_rotvec(rvec_bh + offset_loc[3:]).as_dcm()
        R_bo = np.matmul(R_bh_new, handle.Toff_oh[:3, :3].transpose())
        R_lo = np.matmul(R_lgb, R_bo)
        orientation_new = R_lo
    else:
        orientation_new = obj_geo.orientation_mat
    obj_geo.set_offset_tf(center=np.add(obj_geo.center, pos_off), orientation_mat=orientation_new)
    obj.update_sub_points()
    return offset_loc


##
# @class MotionConstraint
# @brief definition of fixture constraint
class MotionConstraint:
    def __init__(self, geometry_list, fix_surface, fix_normal, T_tool_offset=np.identity(4), tol=1e-3):
        self.geometry_list, self.fix_surface, self.fix_normal, self.T_tool_offset, self.tol = \
            geometry_list, fix_surface, fix_normal, T_tool_offset, tol
