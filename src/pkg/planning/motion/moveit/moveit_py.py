import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "lib/moveit_interface_py"))
import moveit_interface_py as mpc
import numpy as np
from enum import Enum

Geometry = mpc.Geometry
GeometryList = mpc.GeometryList
ObjectType = mpc.ObjectType
ConstrainedSpaceType = mpc.ConstrainedSpaceType
Trajectory = mpc.Trajectory

class ObjectOperation(Enum):
    ADD = 0
    REMOVE = 1

def make_assign_arr(type, vals, cast=lambda x:x):
    arr = type()
    for v in vals:
        arr.append(cast(v))
    return arr


def make_assign_vec(type, vals, dim, cast):
    arr = type()
    for i, v in zip(range(dim), vals):
        arr[i] = cast(v)
    return arr


def CartPose(*vals):
    return make_assign_vec(mpc.CartPose, vals, 7, float)


def Vec3(*vals):
    return make_assign_vec(mpc.Vec3, vals, 3, float)


def JointState(dim, *vals):
    return make_assign_vec(lambda: mpc.JointState(dim), vals, dim, float)


def NameList(*vals):
    return make_assign_arr(mpc.NameList, vals, str)


def spread(bp_arr, size):
    return [bp_arr[i] for i in range(size)]

class ObjectMPC:
    def __init__(self, name, type, link_name, pose=None, dims=None, touch_links=None, attach=True):
        if pose is None:
            pose = [0]*7
        if dims is None:
            dims = [0]*3
        if touch_links is None:
            touch_links = []
        self.name, self.type, self.pose, self.dims, self.link_name, self.touch_links, self.attach = \
            name, type, pose, dims, link_name, touch_links, attach

##
# @class MoveitCompactPlanner_BP
# @brief Python client of moveit-boost-python interface
class MoveitCompactPlanner_BP(mpc.Planner):
    def __init__(self, urdf_path, srdf_path, group_names, config_path):
        mpc.Planner.__init__(self)
        self.urdf_path, self.srdf_path, self.config_path = urdf_path, srdf_path, config_path
        self.group_names = group_names
        self.__group_names = NameList(*group_names)
        if not self.init_planner_from_file(urdf_path, srdf_path, self.__group_names, self.config_path):
            raise (RuntimeError("Failed to initialize planner"))
        self.joint_names_py = spread(self.joint_names, self.joint_num)
        self.attached_dict = {}
        self.__clear_all_objects = self.clear_all_objects
        self.clear_all_objects = "FORBIDDEN: Direct call for clear_all_objects does not remove attached objects"

    def set_scene(self, obj_list):
        self.clear_scene()
        res = True
        for obj in obj_list:
            res = res and self.add_object(obj)
        return res

    def add_object(self, obj):
        if obj.attach:
            self.attached_dict[obj.name] = ObjectMPC(obj.name, obj.type, obj.link_name, attach=obj.attach)
        return self.process_object_py(obj, ObjectOperation.ADD.value)

    def remove_object(self, obj):
        if obj.attach:
            del self.attached_dict[obj.name]
        return self.process_object_py(obj, ObjectOperation.REMOVE.value)

    def clear_scene(self):
        for obj in self.attached_dict.values():
            self.remove_object(obj)
        self.__clear_all_objects()

    ##
    # @brief add union manifold.
    # @param tool_offset xyzquat(xyzw) style pose of tool offset in tool link.
    def add_union_manifold_py(self, group_name, tool_link, tool_offset, geometry_list, fix_surface, fix_normal, tol):
        self.add_union_manifold(group_name, tool_link, CartPose(*tool_offset),
                                geometry_list, fix_surface, fix_normal, tol)


    ##
    # @brief search for plan that bring tool_link to goal_pose in coordinate of goal_link.
    # @param goal_pose xyzquat(xyzw) style pose of goal transformation in goal_link.
    def plan_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init, plannerconfig="RRTConnectkConfigDefault",
                timeout=1, vel_scale=0.1, acc_scale=0.1, post_opt=False,  **kwargs):
        self.clear_context_cache()
        plan = self.plan(robot_name, str(tool_link), CartPose(*goal_pose), str(goal_link),
                         JointState(self.joint_num, *Q_init), plannerconfig, timeout, vel_scale, acc_scale, post_opt)
        return np.array(
            [spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success

    ##
    # @brief search for plan that bring tool_link to goal_pose in coordinate of goal_link.
    # @param goal_state joint value list only corresponding to specified robot chain
    def plan_joint_motion_py(self, robot_name, goal_state, Q_init, plannerconfig="RRTConnectkConfigDefault", timeout=1,
                             vel_scale=0.1, acc_scale=0.1, post_opt=False,
                             **kwargs):
        self.clear_context_cache()
        plan = self.plan_joint_motion(robot_name, JointState(len(goal_state), *goal_state),
                                           JointState(self.joint_num, *Q_init), plannerconfig, timeout,
                                      vel_scale, acc_scale, post_opt)
        return np.array(
            [spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success

    ##
    # @brief search for plan that bring tool_link to goal_pose in coordinate of goal_link, with constraints
    # @param goal_pose xyzquat(xyzw) style pose of goal transformation in goal_link.
    def plan_constrained_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init,
                            plannerconfig="BKPIECEkConfigDefault", timeout=1,
                            vel_scale=0.1, acc_scale=0.1, post_opt=False,
                            cs_type=ConstrainedSpaceType.PROJECTED, allow_approximate=False, post_projection=False):
        assert goal_link=="base_link", "Constrained planning is only available in base_link currently!"
        self.clear_context_cache()
        plan = self.plan_with_constraints(robot_name, tool_link,
                                          CartPose(*goal_pose), goal_link, JointState(self.joint_num, *Q_init),
                                          plannerconfig, timeout, vel_scale, acc_scale, post_opt,
                                          cs_type, allow_approximate, post_projection)
        return np.array(
            [spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success

    def process_object_py(self, obj, action):
        return self.process_object(
            str(obj.name), obj.type, CartPose(*obj.pose), Vec3(*obj.dims), str(obj.link_name),
            NameList(*obj.touch_links), obj.attach, action)