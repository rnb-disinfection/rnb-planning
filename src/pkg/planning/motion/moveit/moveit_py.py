import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "lib/moveit_interface_py"))
import moveit_interface_py as mpc
import numpy as np
from enum import Enum

Geometry = mpc.Geometry
GeometryList = mpc.GeometryList
ObjectType = mpc.ObjectType
Trajectory = mpc.Trajectory
Vec3List = mpc.Vec3List
ConstrainedSpaceType = mpc.ConstrainedSpaceType


##
# @class PlannerConfig
# @brief define available planner configs
class PlannerConfig:
    SBLkConfigDefault = 'SBLkConfigDefault'
    ESTkConfigDefault = 'ESTkConfigDefault'
    LBKPIECEkConfigDefault = 'LBKPIECEkConfigDefault'
    BKPIECEkConfigDefault = 'BKPIECEkConfigDefault'
    KPIECEkConfigDefault = 'KPIECEkConfigDefault'
    RRTkConfigDefault = 'RRTkConfigDefault'
    RRTConnectkConfigDefault = 'RRTConnectkConfigDefault'
    RRTstarkConfigDefault = 'RRTstarkConfigDefault'
    TRRTkConfigDefault = 'TRRTkConfigDefault'
    PRMkConfigDefault = 'PRMkConfigDefault'
    PRMstarkConfigDefault = 'PRMstarkConfigDefault'
    FMTkConfigDefault = 'FMTkConfigDefault'
    BFMTkConfigDefault = 'BFMTkConfigDefault'
    PDSTkConfigDefault = 'PDSTkConfigDefault'
    STRIDEkConfigDefault = 'STRIDEkConfigDefault'
    BiTRRTkConfigDefault = 'BiTRRTkConfigDefault'
    LBTRRTkConfigDefault = 'LBTRRTkConfigDefault'
    BiESTkConfigDefault = 'BiESTkConfigDefault'
    ProjESTkConfigDefault = 'ProjESTkConfigDefault'
    LazyPRMkConfigDefault = 'LazyPRMkConfigDefault'
    LazyPRMstarkConfigDefault = 'LazyPRMstarkConfigDefault'
    SPARSkConfigDefault = 'SPARSkConfigDefault'
    SPARStwokConfigDefault = 'SPARStwokConfigDefault'
    TrajOptDefault = 'TrajOptDefault'


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
    def __init__(self, name, type, link_name, pose=None, dims=None, touch_links=None, attach=True,
                 vertices=None, triangles=None):
        if pose is None:
            pose = [0]*7
        if dims is None:
            dims = [0]*3
        if touch_links is None:
            touch_links = []
        self.name, self.type, self.pose, self.dims, self.link_name, self.touch_links, self.attach = \
            name, type, pose, dims, link_name, touch_links, attach
        if vertices is None or triangles is None:
            self.vertices = vertices
            self.triangles = triangles
        elif triangles is None:
            raise(RuntimeError(
                "[Error] Only triangle mesh is implemented for MoveIt interpreter now. "
                "Try to convert the point cloud to a triangle mesh or "
                "implement adding point cloud object to the moveit_interface_py library in the project."))
        else:
            self.vertices = Vec3List()
            self.triangles = Vec3List()
            for vert in vertices:
                self.vertices.append(Vec3(*vert))
            for tri in triangles:
                self.triangles.append(Vec3(*tri))

##
# @class MoveitCompactPlanner_BP
# @brief Python client of moveit-boost-python interface
class MoveitCompactPlanner_BP(mpc.Planner):
    def __init__(self, urdf_path, srdf_path, group_names, chain_dict, root_dict, config_path):
        mpc.Planner.__init__(self)
        self.urdf_path, self.srdf_path, self.config_path = urdf_path, srdf_path, config_path
        self.group_names = group_names
        self.chain_dict = chain_dict
        self.root_dict = root_dict
        self.group_joint_nums = {key: len(chain["joint_names"]) for key, chain in chain_dict.items()}
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
        if obj.type == ObjectType.MESH:
            return self.add_mesh_py(obj)
        else:
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
    def plan_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init,
                plannerconfig=PlannerConfig.RRTConnectkConfigDefault,
                timeout=1, vel_scale=0.1, acc_scale=0.1, post_opt=False,  **kwargs):
        self.clear_context_cache()
        plan = self.plan(robot_name, str(tool_link), CartPose(*goal_pose), str(goal_link),
                         JointState(self.joint_num, *Q_init), plannerconfig, timeout, vel_scale, acc_scale, post_opt)
        return np.array(
            [spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), plan.success

    ##
    # @brief search for plan that bring tool_link to goal_pose in coordinate of goal_link.
    # @param goal_state joint value list only corresponding to specified robot chain
    def plan_joint_motion_py(self, robot_name, goal_state, Q_init, plannerconfig=PlannerConfig.RRTConnectkConfigDefault,
                             timeout=1, vel_scale=0.1, acc_scale=0.1, post_opt=False,
                             **kwargs):
        self.clear_context_cache()
        plan = self.plan_joint_motion(robot_name, JointState(len(goal_state), *goal_state),
                                           JointState(self.joint_num, *Q_init), plannerconfig, timeout,
                                      vel_scale, acc_scale, post_opt)
        return np.array([spread(Q, self.joint_num) for Q in spread(plan.trajectory, len(plan.trajectory))]), \
               plan.success

    ##
    # @brief search for plan that bring tool_link to goal_pose in coordinate of goal_link, with constraints
    # @param goal_pose xyzquat(xyzw) style pose of goal transformation in goal_link.
    def plan_constrained_py(self, robot_name, tool_link, goal_pose, goal_link, Q_init,
                            plannerconfig=PlannerConfig.RRTkConfigDefault, timeout=1,
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

    def add_mesh_py(self, obj):
        return self.add_mesh(
            str(obj.name), obj.type, CartPose(*obj.pose),
            obj.vertices, obj.triangles,
            str(obj.link_name), NameList(*obj.touch_links), obj.attach)

    ##
    # @brief    get inverse kinematics solution
    # @remark   currently only a manipulator fixed on global coordinate is supported. movable base is not considered.
    # @param    goal_pose tip link pose in global coordinates
    # @return   joint values only for the specificed robot
    def solve_ik_py(self, robot_name, goal_pose, timeout_single=0.01,
                    self_collision=False, fulll_collision=False):
        root_link = self.root_dict[robot_name]
        base_link = self.chain_dict[robot_name]['link_names'][0]
        assert root_link == "base_link", \
            "[ERROR] Manipulator {} is not fixed on global coordinates (base_link) !!! " \
            "currently only a manipulator fixed on global coordinate is supported. movable base ({}) is not considered.".format(robot_name, root_link)
        Q = self.solve_ik(robot_name, CartPose(*goal_pose), timeout_single,
                             self_collision, fulll_collision)
        Q = np.array(spread(Q, self.group_joint_nums[robot_name]))
        if np.sum(np.abs(Q)) < 1e-4:
            return None
        return Q

    ##
    # @brief set joint state in the planner for collision testing in solve_ik_py
    def set_joint_state_py(self, Q):
        self.planner.set_joint_state(JointState(self.joint_num, *Q))

    ##
    # @brief get joint state in the planner
    def get_joint_state_py(self):
        return np.array(spread(self.planner.get_joint_state(), self.joint_num))

