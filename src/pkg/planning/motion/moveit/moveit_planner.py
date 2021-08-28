from moveit_py import MoveitCompactPlanner_BP, ObjectType, ObjectMPC, \
    Geometry, GeometryList, CartPose, Vec3, make_assign_arr, JointState, Trajectory
from ..interface import MotionInterface
from ....utils.utils import list2dict
from ....utils.rotation_utils import SE3, SE3_inv, Rot_rpy, T2xyzquat
from ....geometry.geometry import GEOTYPE, GeometryScene
from ...constraint.constraint_common import calc_redundancy
from scipy.spatial.transform import Rotation
import numpy as np
import os
from enum import Enum

def gtype_to_otype(gtype):
    if gtype==GEOTYPE.BOX:
        return ObjectType.BOX
    if gtype==GEOTYPE.PLANE:
        return ObjectType.PLANE ## Currently Errornous
    if gtype==GEOTYPE.SPHERE:
        return ObjectType.SPHERE
    if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return ObjectType.CYLINDER
    raise(NotImplementedError("Not implemented constraint shape - {}".format(gtype.name)))

def get_mpc_dims(gtem):
    if gtem.gtype==GEOTYPE.BOX:
        return tuple(gtem.dims)
    elif gtem.gtype==GEOTYPE.PLANE:
        return tuple(gtem.dims)
    elif gtem.gtype==GEOTYPE.SPHERE:
        return (gtem.radius,gtem.radius,gtem.radius,)
    elif gtem.gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        return (gtem.length, gtem.radius, gtem.radius)


##
# @brief make moveit constraint geometry item
def make_constraint_item(gtem):
    cartpose = tuple(gtem.center) + tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat())
    return Geometry(gtype_to_otype(gtem.gtype), CartPose(*cartpose), Vec3(*gtem.dims))

##
# @brief make list of moveit constraint geometry list
def make_constraint_list(gtem_list):
    return make_assign_arr(GeometryList, [make_constraint_item(gtem) for gtem in gtem_list])

##
# @class MoveitPlanner
# @brief Moveit motion planner
# @remark online planning is not supported
class MoveitPlanner(MotionInterface):
    NAME = "MoveIt"
    JOINT_MOTION = 1
    TASK_MOTION = 2

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param enable_dual    boolean flag to enable dual arm manipulation (default=True)
    # @param    motion_filters list of child-class of rnb-planning.src.pkg.planning.motion.filtering.filter_interface.MotionFilterInterface
    def __init__(self, pscene, motion_filters=[], enable_dual=True):
        MotionInterface.__init__(self, pscene, motion_filters)
        config_path = os.path.dirname(self.urdf_path)+"/"
        self.robot_names = self.combined_robot.robot_names
        chain_dict = pscene.robot_chain_dict
        binder_links = [chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}
        srdf_path = write_srdf(robot_names=self.robot_names, chain_dict=chain_dict,
                                    link_names=self.link_names, joint_names=self.joint_names,
                                    urdf_content=self.urdf_content, urdf_path=self.urdf_path
                               )
        self.planner = MoveitCompactPlanner_BP(self.urdf_path, srdf_path, self.robot_names, config_path)
        if not all([a==b for a,b in zip(self.joint_names, self.planner.joint_names_py)]):
            self.need_mapping = True
            self.idx_pscene_to_mpc = np.array([self.joint_names.index(jname) for jname in self.planner.joint_names_py])
            self.idx_mpc_to_pscene = np.array([self.planner.joint_names_py.index(jname) for jname in self.joint_names])
        else:
            self.need_mapping = False

        self.enable_dual = enable_dual
        if self.enable_dual:
            self.dual_planner_dict = get_dual_planner(pscene, binder_links)

    ##
    # @brief update changes in geometric scene and load collision boundaries to moveit planner
    # @param dual_key key of target dual planner: root_robot_name_end_robot_name
    def update_gscene(self, dual_key=None, only_self_collision=False):
        self.gscene.update()
        if only_self_collision:
            return
        if dual_key is None:
            obj_list = []
            for gtem in self.gscene:
                if gtem.collision:
                    if all([not mname in gtem.name for mname in self.robot_names]):
                        obj_list.append(ObjectMPC(
                            gtem.name, gtype_to_otype(gtem.gtype), gtem.link_name,
                            pose=tuple(gtem.center)+tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat()),
                            dims=get_mpc_dims(gtem), touch_links=gtem.adjacent_links)
                        )
            self.planner.set_scene(obj_list)
        else:
            if self.enable_dual:
                dual_planner = self.dual_planner_dict[dual_key]
                transfer_ctem(self.gscene, dual_planner.gscene)
                dual_planner.gscene.update()
                obj_list = []
                for gtem in dual_planner.gscene:
                    if gtem.collision:
                        if all([not mname in gtem.name for mname in self.robot_names]):
                            obj_list.append(ObjectMPC(
                                gtem.name, gtype_to_otype(gtem.gtype), gtem.link_name,
                                pose=tuple(gtem.center)+tuple(Rotation.from_dcm(gtem.orientation_mat).as_quat()),
                                dims=get_mpc_dims(gtem), touch_links=gtem.adjacent_links)
                            )
                dual_planner.planner.set_scene(obj_list)

    ##
    # @brief moveit planning implementation
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param binding_list   list of bindings to pursue
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_values=None, timeout=1,
                       timeout_joint=None, timeout_constrained=None, verbose=False, only_self_collision=False, **kwargs):
        timeout_joint = timeout_joint if timeout_joint is not None else timeout
        timeout_constrained = timeout_constrained if timeout_constrained is not None else timeout
        self.planner.clear_context_cache()
        self.planner.clear_manifolds()
        if self.enable_dual:
            for dual_planner in self.dual_planner_dict.values():
                dual_planner.planner.clear_context_cache()
                dual_planner.planner.clear_manifolds()

        if len(binding_list)>1:
            raise(RuntimeError("Only single manipulator operation is implemented with moveit!"))

        self.update_gscene(only_self_collision=only_self_collision)

        motion_type = 0
        if len(binding_list) == 0: # joint motion case
            motion_type = MoveitPlanner.JOINT_MOTION
            if from_state.Q is None or to_state.Q is None:
                raise(RuntimeError("No motion goal is defined!"))
            diffs = np.abs(from_state.Q - to_state.Q)
            joint_groups =[]
            for rname in self.robot_names:
                idx_rbt = self.combined_robot.idx_dict[rname]
                if np.sum(diffs[idx_rbt])>1e-3:
                    joint_groups.append(rname)
            if len(joint_groups)==0:
                return np.array([from_state.Q, to_state.Q]), to_state.Q, 0, True
            dual = False
            if len(joint_groups)==1:
                group_name = joint_groups[0]
                idx_rbt = self.combined_robot.idx_dict[group_name]
                planner = self.planner
                if self.need_mapping:
                    from_Q = from_state.Q[self.idx_pscene_to_mpc]
                    to_Q = to_state.Q[[idx for idx in self.idx_pscene_to_mpc if idx in idx_rbt]]
                else:
                    from_Q = from_state.Q
                    to_Q =  to_state.Q[idx_rbt]
            else:
                raise(RuntimeError("multi-robot joint motion not implemented!"))
            if verbose:
                print("try joint motion") ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp
            trajectory, success = planner.plan_joint_motion_py(
                group_name, tuple(to_Q), tuple(from_Q), timeout=timeout_joint, **kwargs)
            if verbose:
                print("joint motion tried: {}".format(success)) ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp

        else: # task motion case
            motion_type = MoveitPlanner.TASK_MOTION
            obj_name, ap_name, binder_name, binder_geometry_name = binding_list[0]

            binder = self.pscene.actor_dict[binder_name]
            obj = self.pscene.subject_dict[obj_name]
            handle = obj.action_points_dict[ap_name]
            point_add_handle, rpy_add_handle = redundancy_values[(obj_name, handle.name)]
            point_add_binder, rpy_add_binder = redundancy_values[(obj_name, binder.name)]
            T_handle = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
            T_binder = np.matmul(binder.Toff_lh, SE3(Rot_rpy(rpy_add_binder), point_add_binder))

            group_name_handle = self.binder_link_robot_dict[handle.geometry.link_name] if handle.geometry.link_name in self.binder_link_robot_dict else None
            group_name_binder = self.binder_link_robot_dict[binder.geometry.link_name] if binder.geometry.link_name in self.binder_link_robot_dict else None

            dual = False
            if group_name_binder and not group_name_handle:
                group_name = group_name_binder
                tool, T_tool = binder, T_binder
                target, T_tar = handle, T_handle
                point_add_tool, rpy_add_tool = point_add_binder, rpy_add_binder
                point_add_tar, rpy_add_tar = point_add_handle, rpy_add_handle
            elif group_name_handle and not group_name_binder:
                group_name = group_name_handle
                tool, T_tool = handle, T_handle
                target, T_tar = binder, T_binder
                point_add_tool, rpy_add_tool = point_add_handle, rpy_add_handle
                point_add_tar, rpy_add_tar = point_add_binder, rpy_add_binder
            else:
                if not self.enable_dual:
                    raise(RuntimeError("dual arm motion is not enabled"))
                dual = True
                group_name = "{}_{}".format(group_name_binder, group_name_handle)
                self.update_gscene(group_name, only_self_collision=only_self_collision)
                tool, T_tool = handle, T_handle
                target, T_tar = binder, T_binder
                point_add_tool, rpy_add_tool = point_add_handle, rpy_add_handle
                point_add_tar, rpy_add_tar = point_add_binder, rpy_add_binder

            T_tar_tool = np.matmul(T_tar, SE3_inv(T_tool))
            goal_pose = tuple(T_tar_tool[:3,3]) \
                        +tuple(Rotation.from_dcm(T_tar_tool[:3,:3]).as_quat())

            if dual:
                dual_planner = self.dual_planner_dict[group_name]
                planner = dual_planner.planner
                from_Q = from_state.Q[dual_planner.idx_pscene_to_mpc]*dual_planner.joint_signs
            else:
                planner = self.planner
                if self.need_mapping:
                    from_Q = from_state.Q[self.idx_pscene_to_mpc]
                else:
                    from_Q = from_state.Q

            i_stem = self.pscene.subject_name_list.index(obj_name)
            binding_from = from_state.binding_state[i_stem]
            binding_to = to_state.binding_state[i_stem]
            constraints = obj.make_constraints(binding_from, binding_to)
            if constraints:
                for motion_constraint in constraints:
                    self.add_constraint(group_name, tool.geometry.link_name, tool.Toff_lh, motion_constraint=motion_constraint)
                if verbose:
                    print("try constrained motion") ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp
                trajectory, success = planner.plan_constrained_py(
                    group_name, tool.geometry.link_name, goal_pose, target.geometry.link_name, tuple(from_Q),
                    timeout=timeout_constrained, **kwargs)
                if verbose:
                    print("constrained motion tried: {}".format(success)) ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp
            else:
                if verbose:
                    print("try transition motion") ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp
                trajectory, success = planner.plan_py(
                    group_name, tool.geometry.link_name, goal_pose, target.geometry.link_name, tuple(from_Q),
                    timeout=timeout, **kwargs)

                if verbose:
                    print("transition motion tried: {}".format(success)) ## <- DO NOT REMOVE THIS: helps multi-process issue with boost python-cpp

        if success:
            if dual:
                trajectory = (trajectory*dual_planner.joint_signs)[:,dual_planner.idx_mpc_to_pscene]
            else:
                if self.need_mapping:
                    trajectory = trajectory[:,self.idx_mpc_to_pscene]
                for rname in self.combined_robot.robot_names:
                    if rname != group_name: # fix non-manipulating arm - projection in constrained motion sometimes generates motion in non-using arm
                        trajectory[:, self.combined_robot.idx_dict[rname]] = \
                            from_state.Q[self.combined_robot.idx_dict[rname]]
            Q_last = trajectory[-1]
            Q_last_dict = list2dict(Q_last, self.joint_names)
            if motion_type == MoveitPlanner.JOINT_MOTION:
                error = np.sum(np.abs(to_state.Q - Q_last))
            elif motion_type == MoveitPlanner.TASK_MOTION:
                T_tar, T_tool = target.get_tf_handle(Q_last_dict), tool.get_tf_handle(Q_last_dict)
                T_tar = np.matmul(T_tar, SE3(Rot_rpy(rpy_add_tar), point_add_tar))
                T_tool = np.matmul(T_tool, SE3(Rot_rpy(rpy_add_tool), point_add_tool))

                # T_handle = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
                # T_binder = np.matmul(binder.Toff_lh, SE3(Rot_rpy(rpy_add_binder), point_add_binder))
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

    ##
    # @brief add motion constraint for moveit planner
    # @param group_name manipulator chain group name
    # @param tool_link tool link name
    # @param tool_offset_T tool offset 4x4 transformation matrix in tool link coordinate
    # @param motion_constraint rnb-planning.src.pkg.planning.constraint.constraint_common.MotionConstraint
    # @param use_box boolean flag for using box, to convert box to plane, set this value False (default=True)
    def add_constraint(self, group_name, tool_link, tool_offset_T, motion_constraint):
        xyzquat = T2xyzquat(np.matmul(tool_offset_T, motion_constraint.T_tool_offset))
        self.planner.add_union_manifold_py(group_name=group_name, tool_link=tool_link, tool_offset=xyzquat[0]+xyzquat[1],
                                           geometry_list=make_constraint_list(motion_constraint.geometry_list),
                                           fix_surface=motion_constraint.fix_surface,
                                           fix_normal=motion_constraint.fix_normal, tol=motion_constraint.tol)

    ##
    # @brief check collision in a given trajectory
    def validate_trajectory(self, trajectory):
        self.planner.clear_context_cache()
        self.planner.clear_manifolds()
        self.update_gscene()
        if self.need_mapping:
            trajectory = trajectory[:, self.idx_pscene_to_mpc]
        traj_c = Trajectory()
        for Q in trajectory:
            traj_c.append(JointState(self.joint_num, *Q))
        return self.planner.validate_trajectory(traj_c)


from itertools import permutations

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

##
# @class DualMoveitPlanner
# @brief hold temporal dual arm moveit planner information for internal use
class DualMoveitPlanner:
    def __init__(self, gscene, planner, idx_pscene_to_mpc, idx_mpc_to_pscene, joint_signs):
        self.gscene, self.planner, self.idx_pscene_to_mpc, self.idx_mpc_to_pscene, self.joint_signs = \
                                                gscene, planner, idx_pscene_to_mpc, idx_mpc_to_pscene, joint_signs

def get_dual_planner(pscene, binder_links):
    robot_names = pscene.combined_robot.robot_names
    gscene = pscene.gscene
    blink_dict = {rname: lname for rname, lname in zip(robot_names, binder_links)}
    config_path = os.path.dirname(gscene.urdf_path)+"/"
    dual_chain_list = list(permutations(robot_names,2))
    dual_planner_dict = {}
    for root_robot, end_robot in dual_chain_list:
        rname_new = "{}_{}".format(root_robot, end_robot)
        urdf_content_new, urdf_path_new, srdf_path_new, new_joints, new_links = \
            save_converted_chain(pscene.gscene.urdf_content, pscene.gscene.urdf_path,
                                 rname_new, blink_dict[root_robot], blink_dict[end_robot])
        gscene_new = GeometryScene(urdf_content_new, urdf_path_new, new_joints, new_links, rviz=False)
        transfer_ctem(gscene, gscene_new)
        planner = MoveitCompactPlanner_BP(urdf_path_new, srdf_path_new, [rname_new], config_path)
        dual_planner_dict[rname_new] = \
            DualMoveitPlanner(gscene_new, planner,
                              idx_pscene_to_mpc=np.array([pscene.combined_robot.joint_names.index(jname) for jname in planner.joint_names_py]),
                              idx_mpc_to_pscene=np.array([planner.joint_names_py.index(jname) for jname in pscene.combined_robot.joint_names]),
                              joint_signs=np.concatenate([-np.ones_like(pscene.combined_robot.idx_dict[root_robot]),
                                                          np.ones_like(pscene.combined_robot.idx_dict[end_robot])]))


    return dual_planner_dict


import os
import urdf_parser_py
from urdf_parser_py.urdf import URDF
from ....utils.joint_utils import get_parent_joint, get_link_adjacency_map
from ....utils.rotation_utils import Rot2rpy
from xml.dom import minidom

def __get_chain(link_name_cur, urdf_content, base_link=None):
    chain = []
    while link_name_cur != base_link and link_name_cur in urdf_content.parent_map:
        parent_joint = get_parent_joint(link_name_cur, urdf_content)
        chain = [(parent_joint, link_name_cur)] + chain
        link_name_cur = urdf_content.joint_map[parent_joint].parent
    return chain

## @brief save converted chain - for use in dual motion planning in moveit
def save_converted_chain(urdf_content, urdf_path, robot_new, base_link, end_link):
    urdf_path_new = os.path.join(os.path.dirname(urdf_path),
                                 os.path.basename(urdf_path).split(".")[0] + "_{}.urdf".format(robot_new))
    urdf_content_new = URDF.from_xml_string(URDF.to_xml_string(urdf_content))
    Tinv_joint_next = np.identity(4)
    chain_base = __get_chain(base_link, urdf_content)

    for linkage in reversed(chain_base):
        jname, lname = linkage
        joint = urdf_content_new.joint_map[jname]
        link = urdf_content_new.link_map[lname]
        xyz_bak, rpy_bak = joint.origin.xyz, joint.origin.rpy
        j_xyz, j_rpy = Tinv_joint_next[:3, 3], Rot2rpy(Tinv_joint_next[:3, :3])
        joint.parent, joint.child = joint.child, joint.parent
        joint.origin.xyz = j_xyz.tolist()
        joint.origin.rpy = j_rpy.tolist()

        if joint.limit:
            joint.limit.lower, joint.limit.upper = -joint.limit.upper, -joint.limit.lower
        if joint.safety_controller:
            joint.safety_controller.soft_lower_limit, joint.safety_controller.soft_upper_limit = \
                joint.limit.lower, joint.limit.upper
        for gtem in link.collisions + link.visuals:
            if gtem.origin is None:
                gtem.origin = urdf_parser_py.urdf.Pose([0, 0, 0], [0, 0, 0])
            Tg_new = np.matmul(Tinv_joint_next, SE3(Rot_rpy(gtem.origin.rpy), gtem.origin.xyz))
            gtem.origin.xyz, gtem.origin.rpy = Tg_new[:3, 3].tolist(), Rot2rpy(Tg_new[:3, :3]).tolist()
        T_joint = SE3(Rot_rpy(rpy_bak), xyz_bak)
        Tinv_joint_next = SE3_inv(T_joint)
    urdf_content_new.add_link(urdf_parser_py.urdf.Link(name="stem"))
    urdf_content_new.add_joint(urdf_parser_py.urdf.Joint(
        name="stem_joint_base_link", joint_type="fixed", parent="stem", child=joint.child,
        origin=urdf_parser_py.urdf.Pose(Tinv_joint_next[:3, 3].tolist(), Rot2rpy(Tinv_joint_next[:3, :3]).tolist())))
    joint.child = "stem"

    f = open(urdf_path_new, "w")
    f.write(URDF.to_xml_string(urdf_content_new))
    f.close()
    urdf_content_new = URDF.from_xml_file(urdf_path_new)
    new_chain = __get_chain(end_link, urdf_content_new)
    new_joints = [linkage[0] for linkage in new_chain if
                  linkage[0] and urdf_content_new.joint_map[linkage[0]].type != "fixed"]
    new_links = sorted(urdf_content_new.link_map.keys())

    srdf_path_new = write_srdf(robot_names=[robot_new], urdf_content=urdf_content_new, urdf_path=urdf_path_new,
                               link_names=new_links, joint_names=new_joints,
                               chain_dict={robot_new: {'tip_link': end_link, 'joint_names': new_joints}},
                               base_link=base_link)
    return urdf_content_new, urdf_path_new, srdf_path_new, new_joints, new_links

## @brief write srdf for a joint group - for use in moveit motion planning
def write_srdf(robot_names, urdf_content, urdf_path, link_names, joint_names, chain_dict, base_link="base_link"):
    root = minidom.Document()

    xml = root.createElement('robot')
    xml.setAttribute('name', urdf_content.name)

    for rname in robot_names:
        grp = root.createElement("group")
        grp.setAttribute('name', rname)

        chain = root.createElement("chain")
        chain.setAttribute('base_link', base_link)
        chain.setAttribute('tip_link', chain_dict[rname]['tip_link'])
        grp.appendChild(chain)
        xml.appendChild(grp)

        grp_stat = root.createElement("group_state")
        grp_stat.setAttribute('name', "all-zeros")
        grp_stat.setAttribute('group', rname)
        for jname in chain_dict[rname]['joint_names']:
            jstat = root.createElement("joint")
            jstat.setAttribute('name', jname)
            jstat.setAttribute('value', "0")
            grp_stat.appendChild(jstat)


        xml.appendChild(grp_stat)
    vjoint = root.createElement("virtual_joint")
    vjoint.setAttribute('name', "fixed_base")
    vjoint.setAttribute('type', "fixed")
    vjoint.setAttribute('parent_frame', "world")
    vjoint.setAttribute('child_link', base_link)
    xml.appendChild(vjoint)

    link_adjacency_map, link_adjacency_map_ext = get_link_adjacency_map(urdf_content)
    for idx1 in range(len(link_names)):
        lname1 = link_names[idx1]
        for lname2 in link_names[idx1:]:
            if lname1 == lname2:
                continue
            if lname2 in link_adjacency_map[lname1]:
                dcol = root.createElement("disable_collisions")
                dcol.setAttribute('link1', lname1)
                dcol.setAttribute('link2', lname2 )
                dcol.setAttribute('reason', 'Adjacent')
                xml.appendChild(dcol)


    root.appendChild(xml)

    xml_str = root.toprettyxml(indent ="\t")

    save_path_file = urdf_path.replace("urdf", "srdf")

    with open(save_path_file, "w") as f:
        f.write(xml_str)
    return save_path_file