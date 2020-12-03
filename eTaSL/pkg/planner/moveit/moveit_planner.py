from moveit_py import MoveitCompactPlanner_BP, ObjectType, ObjectMPC
from ..interface import PlannerInterface
from ...utils.utils import list2dict
from ...utils.rotation_utils import SE3, SE3_inv, Rot_rpy
from ...geometry.geometry import GEOTYPE, GeometryHandle
from ...constraint.constraint_common import calc_redundancy
from ...robots_custom import write_srdf
from scipy.spatial.transform import Rotation
import numpy as np
import os

def gtype_to_otype(gtype):
    if gtype==GEOTYPE.BOX:
        return ObjectType.BOX
    elif gtype==GEOTYPE.SPHERE:
        return ObjectType.SPHERE
    elif gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return ObjectType.CYLINDER

def get_mpc_dims(gtem):
    if gtem.gtype==GEOTYPE.BOX:
        return tuple(gtem.dims)
    elif gtem.gtype==GEOTYPE.SPHERE:
        return (gtem.radius,gtem.radius,gtem.radius,)
    elif gtem.gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return (gtem.length, gtem.radius, gtem.radius)

def get_binder_links_in_order(links, robot_names):
    # links on robots should include robot names
    return [[lname for lname in links if rname in lname][0] for rname in robot_names]

class MoveitPlanner(PlannerInterface):
    NAME = "MoveIt"

    def __init__(self, joint_names, link_names, urdf_path, urdf_content, robot_names, ghnd, binder_links=None):
        self.ghnd = ghnd
        self.joint_names, self.link_names, self.urdf_path, self.urdf_content, self.robot_names\
            = joint_names, link_names, urdf_path, urdf_content, robot_names
        if binder_links:
            self.binder_links = get_binder_links_in_order(binder_links, self.robot_names)
            self.srdf_path = write_srdf(robot_names=self.robot_names, binder_links=self.binder_links,
                                        link_names=self.link_names, joint_names=self.joint_names,
                                        urdf_content=self.urdf_content, urdf_path=self.urdf_path
                                        )
        else:
            self.srdf_path = self.urdf_path.replace("urdf", 'srdf')
        self.config_path = os.path.dirname(self.urdf_path)+"/"
        self.planner = MoveitCompactPlanner_BP(self.urdf_path, self.srdf_path, self.robot_names, self.config_path)
        if not all([a==b for a,b in zip(self.joint_names, self.planner.joint_names_py)]):
            self.need_mapping = True
            self.idx_graph_to_mpc = [self.joint_names.index(jname) for jname in self.planner.joint_names_py]
            self.idx_mpc_to_graph = [self.planner.joint_names_py.index(jname) for jname in self.joint_names]
        else:
            self.need_mapping = False

    def update_gtems(self):
        self.ghnd.update()
        self.obj_list = []
        for gtem in self.ghnd:
            if gtem.collision:
                if all([not gname in gtem.name for gname in self.robot_names]):
                    self.obj_list.append(ObjectMPC(
                        gtem.name, gtype_to_otype(gtem.gtype), gtem.link_name,
                        pose=tuple(gtem.center)+tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat()),
                        dims=get_mpc_dims(gtem), touch_links=gtem.adjacent_links)
                    )

        self.planner.set_scene(self.obj_list)

    def plan_transition(self, from_state, to_state, binding_list, redundancy_dict=None, timeout=0.1,
                        group_name_handle=None, group_name_binder=None, **kwargs):
        if len(binding_list)!=1:
            raise(RuntimeError("Only single manipulator operation is implemented with moveit!"))
        self.update_gtems()

        obj_name, ap_name, binder_name = binding_list[0]
        redundancy = redundancy_dict[obj_name] if redundancy_dict else None

        binder = self.binder_dict[binder_name]
        obj = self.object_dict[obj_name]
        handle = obj.action_points_dict[ap_name]

        group_name_handle = group_name_handle or [gname for gname in self.planner.group_names if gname in handle.object.link_name]
        group_name_binder = group_name_binder or [gname for gname in self.planner.group_names if gname in binder.object.link_name]

        point_add, rpy_add = calc_redundancy(redundancy, binder)

        T_handle = handle.Toff_lh
        T_binder = np.matmul(binder.Toff_lh, SE3(Rot_rpy(rpy_add), point_add))
        if group_name_binder and not group_name_handle:
            group_name = group_name_binder[0]
            tool, T_tool = binder, T_binder
            target, T_tar = handle, T_handle
        elif group_name_handle and not group_name_binder:
            group_name = group_name_handle[0]
            tool, T_tool = handle, T_handle
            target, T_tar = binder, T_binder
        else:
            print(binder.name, obj.object.name)
            print(group_name_binder, group_name_handle)
            print("uncontrollable binding")
            # raise(RuntimeError("uncontrollable binding"))
            return [], [0]*self.planner.joint_num, None, False

        T_tar_tool = np.matmul(T_tar, SE3_inv(T_tool))
        goal_pose = tuple(T_tar_tool[:3,3]) \
                    +tuple(Rotation.from_dcm(T_tar_tool[:3,:3]).as_quat())

        if self.need_mapping:
            from_Q = from_state.Q[self.idx_graph_to_mpc]
        else:
            from_Q = from_state.Q

        trajectory, success = self.planner.plan_py(
            group_name, tool.object.link_name, goal_pose, target.object.link_name, tuple(from_Q), timeout=timeout)

        if success:
            if self.need_mapping:
                trajectory = trajectory[:,self.idx_mpc_to_graph]
            Q_last = trajectory[-1]
            Q_last_dict = list2dict(Q_last, self.joint_names)
            T_tar, T_tool = target.get_tf_handle(Q_last_dict), tool.get_tf_handle(Q_last_dict)
            T_tar = np.matmul(T_tar, SE3(Rot_rpy(rpy_add), point_add))
            error = np.linalg.norm(T_tar-T_tool)
        else:
            Q_last, error = [0]*self.planner.joint_num, None
        return trajectory, Q_last, error, success


    def init_online_plan(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        raise(RuntimeError("online operation not implemented with moveit"))

    def step_online_plan(self, i_q, pos, wp_action=False):
        raise(RuntimeError("online operation not implemented with moveit"))

    def update_online(self, obsPos_dict):
        raise(RuntimeError("online operation not implemented with moveit"))

    def update_target_joint(self, idx_cur, traj, joint_cur):
        raise(RuntimeError("online operation not implemented with moveit"))
