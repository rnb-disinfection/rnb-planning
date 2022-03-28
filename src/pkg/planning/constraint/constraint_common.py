from enum import Enum
from copy import deepcopy
import random
from ...geometry.geometry import *
from abc import *
from ...utils.utils import DummyObject
from ...utils.rotation_utils import SE3_inv, T_xyzrpy, T2xyzrpy, matmul_series
from scipy.spatial.transform import Rotation
from collections import namedtuple

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
    Waypoint = 6
    Knob = 7
    Hinge = 8


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
    redundancy = {}

    ##
    # @param    name        action point name
    # @param    geometry    geometry of the action point (rnb-planning.src.pkg.geometry.geometry.GeometryItem)
    # @param    point       action point offset respect to parent geometry
    # @param    rpy         orientation offset respect to parent geometry
    # @param    name_full   constraint full name, mainly for eTaSL constraint definition
    def __init__(self, name, geometry, point, rpy, name_full=None, key=0):
        self.gscene = geometry.gscene
        self.name = name
        self.geometry = geometry
        self.set_point_rpy(point, rpy)
        self.name_full = \
            name_full if name_full else "{objname}_{name}".format(objname=self.geometry.name, name=self.name)
        self.key = key

    ##
    # @param    point       action point offset respect to parent geometry
    # @param    rpy         orientation offset respect to parent geometry
    def set_point_rpy(self, point, rpy):
        self.point = point
        self.rpy_point = rpy
        self.R_point = Rot_rpy(self.rpy_point)
        self.Toff_gh = SE3(self.R_point, self.point if self.point is not None else (0, 0, 0))
        self.update_handle()
        self.update_redundancy()

    ##
    # @brief    update Transformation from link (Toff_lf), when parent geometry has moved
    def update_handle(self):
        self.Toff_lh = np.matmul(self.geometry.Toff, self.Toff_gh)

    ##
    # @brief    get handle transformation
    # @param    joint_dict  current joint values as dictionary
    # @param    from_link   refernce link
    def get_tf_handle(self, joint_dict, from_link='base_link'):
        return np.matmul(self.geometry.get_tf(joint_dict, from_link=from_link), self.Toff_gh)

    ##
    # @brief    return redundancy of action point
    def get_redundancy(self, Q=None):
        return self.redundancy

    ##
    # @brief (prototype) update redundancy based on current informations: point, dims
    def update_redundancy(self):
        pass

    def get_args(self):
        return {"name": self.name,
                "gname": self.geometry.name,
                "type": self.__class__.__name__,
                "point": self.point,
                "rpy": self.rpy_point,
                "name_full": self.name_full}


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

BindingChain = namedtuple("BindingChain", ["subject_name", "handle_name", "actor_name", "actor_root_gname"])

##
# @class    BindingTransform
# @brief    information holder that contains all sub-transfomations in the binding
# @remark   give either one of redundancy, T_loal. It none is given, current position is used
class BindingTransform:
    def __init__(self, obj, handle, actor, redundancy=None, T_loal=None, T_lao=None, null_bind_link="base_link"):
        self.point_add_handle, self.rpy_add_handle, self.point_add_actor, self.rpy_add_actor = [(0,0,0)]*4

        if handle is None: # for cases where handle is not important. this can cause errors
            handle = DummyObject()
            handle.name = None
            handle.Toff_lh = obj.geometry.Toff

        if actor is None: # for cases when actor is not initialized: TaskSubjects
            actor = DummyObject()
            actor.name = None
            actor.Toff_lh = np.identity(4)
            actor_root = None
            ## @brief link of actor
            self.actor_link = null_bind_link
        else:
            actor_root = actor.geometry.name
            self.actor_link = actor.geometry.link_name

        # @brief BindingChain, (subject name, handle name, actor name, actor's root geometry name)
        self.chain = BindingChain(obj.oname, handle.name, actor.name, actor_root)
        # @brief correscponding redundancy in form of {action point name: {axis: value}}
        self.redundancy = redundancy

        if T_lao is not None:
            T_laol = np.matmul(T_lao, np.linalg.inv(obj.geometry.Toff))
            T_loal = np.linalg.inv(T_laol)

        if redundancy is not None:
            self.set_redundancy(redundancy, handle, actor)
        elif T_loal is not None:
            T_handle_lh = np.matmul(T_loal, actor.Toff_lh)
            T_add_handle = np.matmul(np.linalg.inv(handle.Toff_lh), T_handle_lh)
            self.point_add_handle, self.rpy_add_handle = T2xyzrpy(T_add_handle)
        elif actor.name is not None:
            if actor.geometry.link_name == obj.geometry.link_name:
                T_add_actor = np.matmul(np.linalg.inv(actor.Toff_lh), handle.Toff_lh)
                self.point_add_actor, self.rpy_add_actor = T2xyzrpy(T_add_actor)

        self.update_transforms(obj, handle, actor)

    def set_redundancy(self, redundancy, handle, actor):
        if self.chain.handle_name in redundancy:
            self.point_add_handle, self.rpy_add_handle = calc_redundancy(redundancy[handle.name], handle)
        if self.chain.actor_name in redundancy:
            self.point_add_actor, self.rpy_add_actor = calc_redundancy(redundancy[actor.name], actor)

    def update_transforms(self, obj, handle, actor):
        ## @brief redundant transformation added on the handle side
        self.T_add_handle = T_xyzrpy((self.point_add_handle, self.rpy_add_handle))
        ## @brief redundant transformation added on the actor side
        self.T_add_actor = T_xyzrpy((self.point_add_actor, self.rpy_add_actor))
        ## @brief redundant transformation from handle to effector
        self.T_add_ah = np.matmul(self.T_add_actor, np.linalg.inv(self.T_add_handle))
        ## @brief link-to-handle transformation with redundancy
        self.T_handle_lh = np.matmul(handle.Toff_lh, self.T_add_handle)
        ## @brief link-to-actor transformation with redundancy
        self.T_actor_lh = np.matmul(actor.Toff_lh, self.T_add_actor)
        ## @brief link-object-actor-link transformation with redundancy
        self.T_loal = np.matmul(self.T_handle_lh, np.linalg.inv(self.T_actor_lh))
        ## @brief link-actor-object-link transformation with redundancy
        self.T_laol = np.linalg.inv(self.T_loal)
        ## @brief link-actor-object transformation with redundancy
        self.T_lao = np.matmul(self.T_laol, obj.geometry.Toff)

    def get_chain(self):
        return self.chain

    def get_instance_chain(self, pscene):
        subject = pscene.subject_dict[self.chain.subject_name]
        return subject, \
               subject.action_points_dict[self.chain.handle_name] \
                   if self.chain.handle_name in subject.action_points_dict else self.chain.handle_name, \
               pscene.actor_dict[self.chain.actor_name] \
                   if self.chain.actor_name in pscene.actor_dict else self.chain.actor_name

##
# @class    BindingState
# @brief    dictionary of form {subject name: BindingTransform}
class BindingState(dict):
    def __init__(self, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)

    def get_chains_sorted(self):
        return sorted([btf.get_chain() for btf in self.values()])



##
# @brief    combine redundancy of handle and binder
# @param to_ap      handle
# @param to_binder  binder
# @param Q          joint angle list
# @return {point name: {direction: (min, max)}}
def combine_redundancy(to_ap, to_binder, Q=None):
    redundancy_bd = deepcopy(to_binder.get_redundancy(Q))
    redundancy_ap = deepcopy(to_ap.get_redundancy(Q))
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
# @brief    calculate margins for binding between two ActionPoints. Negative means deviation from the allowed range
# @param handle_T   transformation matrix (4x4) to global coordinate for handle
# @param binder_T   transformation matrix (4x4) to global coordinate for binder
# @param handle_redundancy  redundancy for handle {axis: (min, max)}, where axis in "xyzuvw"
# @param binder_redundancy  redundancy for binder {axis: (min, max)}, where axis in "xyzuvw"
# @param rot_scale scaling factor applied to rotation margin
# @return 6x2 matrix that contains margin on the mininum/maximum side, in "xyz+rotationvector" order
def get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy, rot_scale=1.0):
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
    # assert len(set(handle_rot_axes).union(
    #     binder_rot_axes)) <= 1, "Rotation redundancy is allowed for only single axis, in same axis in both binder and handle"

    rot_range = handle_rot_range + binder_rot_range
    rot_vec = Rotation.from_dcm(T_rel[:3, :3]).as_rotvec()
    margin_rot = np.stack([rot_vec - rot_range[:, 0], rot_range[:, 1] - rot_vec], axis=-1)

    return np.concatenate([margin_vec, margin_rot*rot_scale])

##
# @brief fit binding
# @param    obj     AbstractObject to update location to fit binding
# @param    handle  handle to match with binder
# @param    binder  binder to match with handle
# @param    Q_dict  current joint configuration in dictionary
# @return
def fit_binding(obj, handle, binder, Q_dict, Poffset=None):
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
    if Poffset is not None:
        pos_off = np.add(pos_off, Poffset)

    if np.linalg.norm(offset_loc[3:]) > 1e-2:
        R_bh = np.matmul(binder_T[:3, :3].transpose(), handle_T[:3, :3])
        rvec_bh = Rotation.from_dcm(R_bh).as_rotvec()
        R_bh_new = Rotation.from_rotvec(rvec_bh + offset_loc[3:]).as_dcm()
        R_bg = np.matmul(R_bh_new, handle.Toff_gh[:3, :3].transpose())
        R_lg = np.matmul(R_lgb, R_bg)
        orientation_new = R_lg
    else:
        orientation_new = obj_geo.orientation_mat
    obj_geo.set_offset_tf(center=np.add(obj_geo.center, pos_off), orientation_mat=orientation_new)
    obj.update_sub_points()
    return offset_loc

##
# @brief find best matching object&handle for a actor pose
def find_match(pscene, actor, T_ba, Q_dict, margin=1e-3):
    binder_redundancy = actor.get_redundancy()
    binder_T = T_ba
    match = None
    margin_max = -1e5
    for obj in pscene.subject_dict.values():
        for handle in obj.action_points_dict.values():
            if actor.check_pair(handle):
                handle_T = handle.get_tf_handle(Q_dict)
                handle_redundancy = handle.get_redundancy()
                margin_mat = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy)
                if np.min(margin_mat>=-margin):
                    return obj, handle

##
# @class MotionConstraint
# @brief definition of fixture constraint
class MotionConstraint:
    def __init__(self, geometry_list, fix_surface, fix_normal, T_tool_offset=np.identity(4), tol=1e-3):
        self.geometry_list, self.fix_surface, self.fix_normal, self.T_tool_offset, self.tol = \
            geometry_list, fix_surface, fix_normal, T_tool_offset, tol
