from moveit_py import MoveitCompactPlanner_BP, ObjectType, ObjectMPC, Geometry, GeometryList, CartPose, Vec3, make_assign_arr
from ..interface import MotionInterface
from ....utils.utils import list2dict
from ....utils.rotation_utils import SE3, SE3_inv, Rot_rpy
from ....geometry.geometry import GEOTYPE, GeometryScene
from ....geometry.builder.xacro_customizer import write_srdf
from ...constraint.constraint_common import calc_redundancy
from scipy.spatial.transform import Rotation
import numpy as np
import os

def gtype_to_otype(gtype):
    if gtype==GEOTYPE.BOX:
        return ObjectType.BOX
    if gtype==GEOTYPE.SPHERE:
        return ObjectType.SPHERE
    if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return ObjectType.CYLINDER
    raise(NotImplementedError("Not implemented constraint shape - {}".format(gtype.name)))

def gtype_to_ctype(gtype):
    if gtype==GEOTYPE.BOX:
        return ObjectType.PLANE
    if gtype==GEOTYPE.SPHERE:
        return ObjectType.SPHERE
    if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return ObjectType.CYLINDER
    raise(NotImplementedError("Not implemented constraint shape - {}".format(gtype.name)))

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

def make_constraint_item(gtem, use_box=False):
    cartpose = tuple(gtem.center) + tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat())
    return Geometry(gtype_to_otype(gtem.gtype) if use_box else gtype_to_ctype(gtem.gtype),
                    CartPose(*cartpose), Vec3(*gtem.dims))

def make_constraint_list(gtem_list, use_box=False):
    return make_assign_arr(GeometryList, [make_constraint_item(gtem, use_box) for gtem in gtem_list])

##
# @class MoveitPlanner
# @brief Moveit motion planner
# @remark online planning is not supported
class MoveitPlanner(MotionInterface):
    NAME = "MoveIt"

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param manipulator_names moveit manipulator group names
    def __init__(self, pscene, manipulator_names=None):
        MotionInterface.__init__(self, pscene)
        self.robot_names = self.combined_robot.robot_names
        self.manipulator_names = manipulator_names or self.robot_names
        binder_links = [v.geometry.link_name for v in pscene.binder_dict.values()]
        if binder_links:
            self.binder_links = get_binder_links_in_order(binder_links, self.robot_names)
            self.srdf_path = write_srdf(robot_names=self.robot_names, binder_links=self.binder_links,
                                        link_names=self.link_names, joint_names=self.joint_names,
                                        urdf_content=self.urdf_content, urdf_path=self.urdf_path
                                        )
        self.config_path = os.path.dirname(self.urdf_path)+"/"
        self.planner = MoveitCompactPlanner_BP(self.urdf_path, self.srdf_path, self.robot_names, self.config_path)
        if not all([a==b for a,b in zip(self.joint_names, self.planner.joint_names_py)]):
            self.need_mapping = True
            self.idx_graph_to_mpc = [self.joint_names.index(jname) for jname in self.planner.joint_names_py]
            self.idx_mpc_to_graph = [self.planner.joint_names_py.index(jname) for jname in self.joint_names]
        else:
            self.need_mapping = False

    ##
    # @brief update changes in geometric scene and load collision boundaries to moveit planner
    def update_gcene(self):
        self.gscene.update()
        self.obj_list = []
        for gtem in self.gscene:
            if gtem.collision:
                if all([not gname in gtem.name for gname in self.manipulator_names]):
                    self.obj_list.append(ObjectMPC(
                        gtem.name, gtype_to_otype(gtem.gtype), gtem.link_name,
                        pose=tuple(gtem.center)+tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat()),
                        dims=get_mpc_dims(gtem), touch_links=gtem.adjacent_links)
                    )

        self.planner.set_scene(self.obj_list)

    ##
    # @brief moveit planning implementation
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param binding_list   list of bindings to pursue
    # @param redundancy_dict    redundancy in dictionary format {object name: {axis: value}}
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_dict=None, timeout=0.1,
                        group_name_handle=None, group_name_binder=None, **kwargs):
        if len(binding_list)!=1:
            raise(RuntimeError("Only single manipulator operation is implemented with moveit!"))
        self.update_gcene()

        obj_name, ap_name, binder_name = binding_list[0]
        redundancy = redundancy_dict[obj_name] if redundancy_dict else None

        binder = self.pscene.binder_dict[binder_name]
        obj = self.pscene.object_dict[obj_name]
        handle = obj.action_points_dict[ap_name]
        point_add, rpy_add = calc_redundancy(redundancy, binder)
        T_handle = handle.Toff_lh
        T_binder = np.matmul(binder.Toff_lh, SE3(Rot_rpy(rpy_add), point_add))

        if len(self.planner.group_names)==1:
            group_name_handle = self.planner.group_names if handle.geometry.link_name in self.urdf_content.parent_map else []
            group_name_binder = self.planner.group_names if binder.geometry.link_name in self.urdf_content.parent_map else []
        else:
            group_name_handle = group_name_handle or [gname for gname in self.planner.group_names if gname in handle.geometry.link_name]
            group_name_binder = group_name_binder or [gname for gname in self.planner.group_names if gname in binder.geometry.link_name]

        if group_name_binder and not group_name_handle:
            group_name = group_name_binder[0]
            tool, T_tool = binder, T_binder
            target, T_tar = handle, T_handle
        elif group_name_handle and not group_name_binder:
            group_name = group_name_handle[0]
            tool, T_tool = handle, T_handle
            target, T_tar = binder, T_binder
        else:
            print(binder.name, obj.geometry.name)
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
            group_name, tool.geometry.link_name, goal_pose, target.geometry.link_name, tuple(from_Q), timeout=timeout)

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


from itertools import permutations
from ....geometry.builder.xacro_customizer import save_converted_chain

def transfer_ctem(gscene, gscene_new):
    link_chain = gscene_new.urdf_content.get_chain(gscene_new.urdf_content.get_root(), "stem", joints=False, links=True)
    for gtem in gscene:
        if gtem.collision:
            gtem_new = gscene_new.create_safe(gtype=gtem.gtype, name=gtem.name, link_name=gtem.link_name,
                                 dims=gtem.dims, center=gtem.center, rpy=gtem.rpy, color=gtem.color, display=gtem.display,
                                 collision=gtem.collision, fixed=gtem.fixed, soft=gtem.soft, online=gtem.online, K_col=gtem.K_col)
            if gtem_new.link_name in link_chain:
#                 print("transfered: {}".format(gtem_new.name))
                joint_child = gscene_new.urdf_content.joint_map[gscene_new.urdf_content.child_map[gtem_new.link_name][0][0]]
                T_jp = SE3(Rot_rpy(joint_child.origin.rpy), joint_child.origin.xyz)
                Toff_new = np.matmul(T_jp, gtem_new.Toff)
                gtem_new.set_offset_tf(Toff_new[:3,3], Toff_new[:3,:3])
#             else:
#                 print("as-is: {}".format(gtem_new.name))
#         else:
#             print("no-collision: {}".format(gtem.name))

def get_dual_planner_dict(GRIPPER_REFS, gscene, urdf_content, urdf_path, link_names, robot_names):
    glist = GRIPPER_REFS.keys()
    dual_chain_list = list(permutations(glist,2))
    dual_mplan_dict = {}
    for grip_root_name, grip_end_name in dual_chain_list:
        grip_root = GRIPPER_REFS[grip_root_name]
        grip_end = GRIPPER_REFS[grip_end_name]
        grip_root_link = urdf_content.link_map[grip_root['link_name']]
        grip_end_link = urdf_content.link_map[grip_end['link_name']]
        rname_new = "{}_{}".format(grip_root_name, grip_end_name)
        urdf_content_new, urdf_path_new, srdf_path_new, new_joints = save_converted_chain(urdf_content, urdf_path, rname_new, grip_root_link.name, grip_end_link.name)
        gscene_new = GeometryScene(urdf_content_new)
        transfer_ctem(gscene, gscene_new)
        dual_mplan_dict[(grip_root_name, grip_end_name)] = MoveitPlanner(gscene=gscene_new, joint_names=new_joints, link_names=link_names, urdf_path=urdf_path_new, urdf_content=urdf_content_new,
                                                                         robot_names=[rname_new], manipulator_names=robot_names)
    return dual_mplan_dict