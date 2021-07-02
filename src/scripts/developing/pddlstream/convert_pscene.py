import os
import sys
sys.path.insert(1, os.environ["PDDL_STREAM_DIR"])
sys.path.insert(1, os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))

import pybullet
from examples.pybullet.utils.pybullet_tools.utils import get_configuration, link_from_name, add_fixed_constraint, INFO_FROM_BODY, CLIENT, ModelInfo
from examples.pybullet.utils.pybullet_tools.utils import create_box, create_capsule, create_cylinder, create_plane, create_sphere, create_shape_array, create_body
from examples.pybullet.utils.pybullet_tools.utils import set_point, set_euler, set_default_camera, draw_global_system, add_data_path, load_pybullet
from examples.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, get_pose, set_pose, Pose, \
    Point, stable_z, \
    BLOCK_URDF, SMALL_BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, HideOutput, wait_for_user, KUKA_IIWA_URDF, \
    LockRenderer, has_gui, draw_pose, is_darwin, disable_preview, CLIENTS,CLIENT, p
from examples.pybullet.utils.pybullet_tools.utils import get_moving_links, get_links, are_links_adjacent, get_moving_pairs, get_link_names

from pkg.geometry.geotype import *
from pkg.planning.constraint.constraint_subject import AbstractObject
from pkg.utils.rotation_utils import *
from pkg.utils.joint_utils import *
from pkg.utils.utils import *
from shutil import copyfile
import rospkg
rospack = rospkg.RosPack()

from pkg.planning.constraint.constraint_common import combine_redundancy, sample_redundancy, calc_redundancy
from primitives_pybullet import update_grasp_info, GraspInfo, pairwise_collision, BodyPose, sample_placement, BodyGrasp
import random

SAMPLE_GRASP_COUNT_DEFAULT = 10


##
# @brief    add axis marker to handle
def add_handle_axis(hl_key, handle, Toff=None, color=None, dims=(0.10, 0.01, 0.01)):
    gscene = handle.geometry.gscene
    hobj = handle.geometry
    Toff_lh = handle.Toff_lh
    if Toff is not None:
        Toff_lh = np.matmul(Toff_lh, Toff)
    axis = "xyz"
    gscene.add_highlight_axis(hl_key, hobj.name, hobj.link_name, Toff_lh[:3, 3], Toff_lh[:3, :3], color=color,
                                   axis=axis, dims=dims)

##
# @brief return offset from actor coordinate to object coordinate
def sample_redundancy_offset(subject, actor, drop_downward_dir=None, show_state=False,
                             binding_sampler=random.choice, redundancy_sampler=random.uniform):
    if show_state:
        subject.geometry.gscene.clear_highlight()
    for i_s in range(100):
        assert i_s < 100, "Set drop_downward_dir to the direction you want to keep upward on the actor coordinate, default is y-axis"
            
        handle = binding_sampler([ap for ap in subject.action_points_dict.values() if actor.check_type(ap)])
        redundancy_tot = combine_redundancy(handle, actor)
        redundancy = sample_redundancy(redundancy_tot, sampler=redundancy_sampler)
        point_add_handle, rpy_add_handle = calc_redundancy(redundancy[handle.name], handle)
        point_add_actor, rpy_add_actor = calc_redundancy(redundancy[actor.name], actor)
        T_handle_gh = np.matmul(handle.Toff_gh,
                                SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_ah = T_xyzrpy((point_add_actor, rpy_add_actor))
        T_ahg = np.matmul(T_ah, SE3_inv(T_handle_gh))
        if subject.geometry == handle.geometry:
            T_ao = T_ahg
        else:
            T_hgo = np.matmul(SE3_inv(handle.geometry.Toff), subject.geometry.Toff)
            T_ao = np.matmul(T_ahg, T_hgo)

        kwargs = {}
        pass_this = False
        if drop_downward_dir is not None:
            dropvec = np.matmul(T_ao[:3,:3].transpose(), drop_downward_dir)
            if dropvec[2] < 0:
                kwargs, pass_this = dict(color=(1,0,0,0.5), dims=(0.07, 0.005, 0.005)), True

        if show_state:
            T_ha = np.matmul(SE3(Rot_rpy(rpy_add_handle), point_add_handle), SE3_inv(T_ah))
            if actor.geometry.link_name == "base_link":
                add_handle_axis("sro_{}".format(i_s), actor, Toff=SE3_inv(T_ha), **kwargs)
            else:
                add_handle_axis("sro_{}".format(i_s), handle, Toff=T_ha, **kwargs)
        if pass_this:
            time.sleep(0.1)
            continue
        time.sleep(0.5)
        break
    return T_ao


def get_stable_gen_rnb(body_subject_map, body_actor_map, home_dict, fixed=[], show_state=False,
                       binding_sampler=random.choice, redundancy_sampler=random.uniform):
    def gen(body, surface):
        rnb_style = False
        if body in body_subject_map and surface in body_actor_map:
            rnb_style = True
            subject = body_subject_map[body]
            actor = body_actor_map[surface]
        fail_count = 0
        while True:
            with GlobalTimer.instance().block("get_stable_{}_{}".format(body, surface)):
                if rnb_style:
                    T_ao = sample_redundancy_offset(subject, actor, show_state=show_state,
                                                    binding_sampler=binding_sampler,
                                                    drop_downward_dir=[0, 1, 0],
                                                    redundancy_sampler=redundancy_sampler)
                    T_pose = np.matmul(actor.get_tf_handle(home_dict), T_ao)
                    pose = T2xyzquat(T_pose)
                else:
                    pose = None
                    break

                set_pose(body, pose)
                if (pose is None) or any(pairwise_collision(body, b) for b in fixed):
                    # fail_count += 1
                    # if fail_count>10:
                    #     yield None
                    continue
                fail_count=0
                body_pose = BodyPose(body, pose)
            yield (body_pose,)
    return gen



def get_grasp_gen_rnb(body_subject_map, robot, tool_link_name, actor,
                      sample_count=SAMPLE_GRASP_COUNT_DEFAULT, show_state=False,
                      binding_sampler=random.choice, redundancy_sampler=random.uniform,
                      approach_pose=Pose(0.05 * Point(z=-1))):
    tool_link = link_from_name(robot, tool_link_name)
    def gen(body):
        subject = body_subject_map[body]
        for _ in range(sample_count):
            with GlobalTimer.instance().block("sample_grasps_{}".format(body)):
                T_ao = sample_redundancy_offset(subject, actor, show_state=show_state,
                                                binding_sampler=binding_sampler,
                                                drop_downward_dir=[0, 1, 0],
                                                redundancy_sampler=redundancy_sampler)
                T_lo = np.matmul(actor.Toff_lh, T_ao)
                point, euler = T2xyzrpy(T_lo)
                body_grasp = BodyGrasp(body, Pose(point=point, euler=euler),
                                       approach_pose, robot, tool_link)
            yield (body_grasp,)
    return gen

def sample_grasps(body_subject_map, body, actor, sample_count=SAMPLE_GRASP_COUNT_DEFAULT, show_state=False,
                  binding_sampler=random.choice, redundancy_sampler=random.uniform):
    with GlobalTimer.instance().block("sample_grasps_{}".format(body)):
        subject = body_subject_map[body]
        grasps = []
        for _ in range(sample_count):
            T_ao = sample_redundancy_offset(subject, actor, show_state=show_state,
                                            binding_sampler=binding_sampler, 
                                            redundancy_sampler=redundancy_sampler)
            T_lo = np.matmul(actor.Toff_lh, T_ao)
            point, euler = T2xyzrpy(T_lo)
            grasps.append(Pose(point=point, euler=euler))
        return grasps


def get_disabled_collisions(gscene, body):
    all_links = get_links(body)
    link_names = get_link_names(body, all_links)
    link_name_map = {lname:l_no for l_no, lname in zip(all_links, link_names)}
    disabled_collsions = []
    for lname, adjacents in gscene.link_adjacency_map.items():
        if lname in link_name_map:
            lno = link_name_map[lname]
            disabled_collsions += [
                (lno, link_name_map[x])
                for x in adjacents
                if x in link_name_map and x != lname]
    return disabled_collsions


def copy_meshes(gscene):
    upath_ext_split = gscene.urdf_path.split(".")
    urdf_pybullet_path = ".".join(upath_ext_split[:-1])+"_pybullet."+upath_ext_split[-1]
    path_to = os.path.dirname(urdf_pybullet_path)
    copyfile_replace(gscene.urdf_path, urdf_pybullet_path, "package://", "./",
                     line_callback=lambda line, string_from, string_to: copyfile_callback(line, string_from, path_to))
    return urdf_pybullet_path


def copyfile_callback(line, string_from="package://", path_to="./"):
    i_s = line.find(string_from)
    i_e = line.rfind('"')
    if i_e<=0:
        i_e = line.rfind("'")
    file_pkg = line[i_s+len(string_from):i_e]
    file_split = file_pkg.split('/')
    pkg = file_split[0]
    path_pkg = rospack.get_path(pkg)
    path_cur = path_to
    for folder in file_split[:-1]:
        path_cur = os.path.join(path_cur, folder)
        try_mkdir(path_cur)
    try: copyfile(os.path.join(path_pkg, *file_split[1:]), os.path.join(path_to, file_pkg))
    except: pass

def load_model_abs(abs_path, pose=None, **kwargs):
    # TODO: error with loadURDF when loading MESH visual and CYLINDER collision
    add_data_path()
    # with LockRenderer():
    print("[Pybullet] Load urdf from {}".format(abs_path))
    body = load_pybullet(abs_path, **kwargs)
    if pose is not None:
        set_pose(body, pose)
    return body


def add_gtem_to_pybullet(gtem, robot_body=0L):
    T_base = gtem.get_tf(list2dict(get_configuration(robot_body), gtem.gscene.joint_names))
    if gtem.gtype == GEOTYPE.BOX:
        bid = create_box(*gtem.dims, color=gtem.color, collision=gtem.collision)
    if gtem.gtype == GEOTYPE.CYLINDER:
        bid = create_cylinder(radius=np.mean(gtem.dims[0:1]), height=gtem.dims[2], color=gtem.color,
                              collision=gtem.collision)
    if gtem.gtype == GEOTYPE.CAPSULE:
        bid = create_capsule(radius=np.mean(gtem.dims[0:1]), height=gtem.dims[2], color=gtem.color,
                             collision=gtem.collision)
    if gtem.gtype == GEOTYPE.SPHERE:
        bid = create_sphere(radius=np.mean(gtem.dims), color=gtem.color, collision=gtem.collision)
    if gtem.gtype == GEOTYPE.PLANE:
        bid = create_plane(color=gtem.color, collision=gtem.collision)
        print("[WARNING] plane geometry not supported yet")

    set_point(bid, T_base[:3, 3])
    set_euler(bid, Rot2rpy(T_base[:3, :3]))
    return bid


##
# @remark root parent first in gtem_list
def add_gtem_fam_to_pybullet(root_name, gtem_list, fixed_base=False, robot_body=0L, base_link="base_link"):
    geoms, poses, colors = [], [], []
    pose_base = None
    for gtem in gtem_list:
        if not gtem.collision:
            continue
        color = gtem.color
        geom = {}
        if gtem.name == root_name:
            T_base = gtem.get_tf(list2dict(get_configuration(robot_body), gtem.gscene.joint_names))
            pose_base = (T_base[:3, 3], Rotation.from_dcm(T_base[:3, :3]).as_quat())
            T_child = SE3(np.identity(3), (0,)*3)
            pose = (T_child[:3, 3], Rotation.from_dcm(T_child[:3, :3]).as_quat())
        else:
            pose = (gtem.center_child, Rotation.from_dcm(gtem.orientation_mat_child).as_quat())
        if gtem.gtype == GEOTYPE.BOX:
            geom['shapeType'] = pybullet.GEOM_BOX
            geom['halfExtents'] = np.divide(gtem.dims, 2).tolist()
        if gtem.gtype == GEOTYPE.CAPSULE:
            geom['shapeType'] = pybullet.GEOM_CAPSULE
            geom['length'] = gtem.dims[2]
            geom['radius'] = np.mean(gtem.dims[0:1])
        if gtem.gtype == GEOTYPE.CYLINDER:
            geom['shapeType'] = pybullet.GEOM_CYLINDER
            geom['length'] = gtem.dims[2]
            geom['radius'] = np.mean(gtem.dims[0:1])
        if gtem.gtype == GEOTYPE.SPHERE:
            geom['shapeType'] = pybullet.GEOM_SPHERE
            geom['radius'] = np.mean(gtem.dims)
        if gtem.gtype == GEOTYPE.PLANE:
            #         geom['shapeType'] = pybullet.GEOM_PLANE
            #         geom['halfExtents'] = np.divide(gtem.dims, 2).tolist()
            print("[WARNING] plane geometry not supported yet")
            continue
        geoms.append(geom)
        poses.append(pose)
        colors.append(color)

    assert pose_base is not None, "root geometry not included in gtem_list"
    cid, vid = create_shape_array(geoms, poses, colors)
    bid = create_body(cid, vid)
    set_pose(bid, pose_base)
    return bid


def make_body_subject_map(pscene, body_names):
    gname_subject_map = {subj.geometry.get_root(): subj for subj in pscene.subject_dict.values()}
    body_subject_map = {}
    body_subject_map.update({bid: gname_subject_map[gname] for bid, gname in body_names.items() if
                             gname in gname_subject_map})
    return body_subject_map


def make_body_actor_map(pscene, body_names):
    gname_actor_map = {subj.geometry.get_root(): subj for subj in pscene.actor_dict.values()}
    body_actor_map = {}
    body_actor_map.update({bid: gname_actor_map[gname] for bid, gname in body_names.items() if
                             gname in gname_actor_map})
    return body_actor_map


def pscene_to_pybullet(pscene, urdf_pybullet_path, tool_name = None, name_exclude_list=[]):
    assert tool_name is not None, "tool_name should be passed to pscene_to_pybullet"
    set_default_camera()
    draw_global_system()
    robot_body = load_model_abs(urdf_pybullet_path, fixed_base=True)

    gscene = pscene.gscene
    gfam_dict = {}
    for gtem in gscene:
        if any([te in gtem.name for te in name_exclude_list]):
            continue
        if gtem.parent is None:
            gfam_dict[gtem.name] = [gscene.NAME_DICT[gname] for gname in gtem.get_family()
                                    if gscene.NAME_DICT[gname].collision]

    bid_dict = {}
    for root_name, gfam in gfam_dict.items():
        if len(gfam) == 0:
            continue
        if len(gfam) == 1:
            bid = add_gtem_to_pybullet(gfam[0], robot_body=robot_body)
        else:
            bid = add_gtem_fam_to_pybullet(root_name, gfam, robot_body=robot_body)
        if bid >= 0:
            bid_dict[root_name] = bid

    body_names = {v: k for k, v in bid_dict.items()}
    movable_bodies = []
    for subject in pscene.subject_dict.values():
        body_i = bid_dict[subject.geometry.get_root()]
        if isinstance(subject, AbstractObject):
            movable_bodies.append(body_i)
        else:
            print("[WARING] non-object subject not implemented for now")

    return robot_body, body_names, movable_bodies

def bps2traj(body_paths):
    traj = []
    for bp in body_paths:
        traj += list(bp.path)
    return np.array(traj)

def play_pddl_plan(pscene, gripper, initial_state, body_names, plan, SHOW_PERIOD=0.01):
    gscene = pscene.gscene
    pscene.set_object_state(initial_state)
    gscene.update_markers_all()
    gscene.show_pose(initial_state.Q)

    for action in plan:
        if action.name == "move_free":
            traj = bps2traj(action.args[-1].body_paths)
            gscene.show_motion(traj, period=SHOW_PERIOD)

        if action.name == "pick":
            bid = action.args[0]
            tar_obj = body_names[bid]
            traj = bps2traj(action.args[-1].body_paths[:1])
            traj_rev = bps2traj(action.args[-1].body_paths[-1:])
            gscene.show_motion(np.array(traj), period=SHOW_PERIOD)
            T_obj = T_xyzquat(action.args[1].value)
            q_e = np.array(traj[-1])
            T_bgl = np.matmul(gripper.geometry.get_tf(list2dict(q_e, gscene.joint_names)), SE3_inv(gripper.geometry.Toff))
            T_lgo = np.matmul(SE3_inv(T_bgl), T_obj)
            obj_pscene = pscene.subject_dict[tar_obj]
            obj_pscene.set_state(binding=(tar_obj, None, gripper.geometry.name, gripper.name),
                             state_param=(gripper.geometry.link_name, T_lgo))
            gscene.show_motion(np.array(traj_rev), period=SHOW_PERIOD)

        if action.name == "move_holding":
            traj = bps2traj(action.args[-1].body_paths)
            gscene.show_motion(traj, period=SHOW_PERIOD)

        if action.name == "place":
            bid = action.args[0]
            tar_obj = body_names[bid]
            traj = bps2traj(action.args[-1].body_paths[:1])
            traj_rev = bps2traj(action.args[-1].body_paths[-1:])
            q_s = traj[0]
            T_obj = T_xyzquat(action.args[1].value)
            T_obj = np.matmul(SE3_inv(get_tf("base_link", list2dict(q_s, gscene.joint_names), gscene.urdf_content)), T_obj)
            obj_pscene = pscene.subject_dict[tar_obj]
            gscene.show_motion(np.array(traj), period=SHOW_PERIOD)
            obj_pscene.set_state(binding=(tar_obj, None, None, None),
                             state_param=("base_link", T_obj))
            gscene.show_motion(np.array(traj_rev), period=SHOW_PERIOD)

