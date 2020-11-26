import os
import sys

sys.path.append(os.path.join(os.environ["TAMP_ETASL_DIR"], "moveit_plan_compact"))
import moveit_plan_compact as mpc
import numpy as np
from enum import Enum


class ObjectType(Enum):
    BOX = 1
    SPHERE = 2
    CYLINDER = 3

class ObjectOperation(Enum):
    ADD = 0
    REMOVE = 1

def make_assign_arr(type, vals, cast):
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
    def __init__(self, name, type, link_name, pose=[0]*7, dims=[0]*3, touch_links=[], attach=True):
        self.name, self.type, self.pose, self.dims, self.link_name, self.touch_links, self.attach = \
            name, type, pose, dims, link_name, touch_links, attach


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

    def plan_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init, plannerconfig="RRTConnectkConfigDefault", timeout=0.1):
        plan = self.plan(robot_name, tool_link, CartPose(*goal_pose), goal_link,
                         JointState(self.joint_num, *Q_init), plannerconfig, timeout)
        return np.array(
            [spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success

    def process_object_py(self, obj, action):
        return self.process_object(
            obj.name, obj.type.value, CartPose(*obj.pose), Vec3(*obj.dims), obj.link_name,
            NameList(*obj.touch_links), obj.attach, action)