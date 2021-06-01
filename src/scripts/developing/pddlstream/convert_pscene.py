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

from pkg.geometry.geotype import *
from pkg.planning.constraint.constraint_subject import AbstractObject
from pkg.utils.rotation_utils import *
from pkg.utils.utils import *
from shutil import copyfile
import rospkg
rospack = rospkg.RosPack()

from pkg.planning.constraint.constraint_common import combine_redundancy, sample_redundancy, calc_redundancy
from primitives_pybullet import update_grasp_info, GraspInfo
import random

SAMPLE_GRASP_COUNT_DEFAULT = 10

def sample_grasps(body_subject_map, body, actor, sample_count=SAMPLE_GRASP_COUNT_DEFAULT, binding_sampler=random.choice, redundancy_sampler=random.uniform):
    subject = body_subject_map[body]
    grasps = []
    for _ in range(sample_count):
        handle = binding_sampler([ap for ap in subject.action_points_dict.values() if actor.check_type(ap)])
        redundancy_tot = combine_redundancy(handle, actor)
        redundancy = sample_redundancy(redundancy_tot, sampler=redundancy_sampler)
        point_add_handle, rpy_add_handle = calc_redundancy(redundancy[handle.name], handle)
        point_add_actor, rpy_add_actor = calc_redundancy(redundancy[actor.name], actor)
        T_handle_oh = np.matmul(handle.Toff_oh,
                                SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh,
                               SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_lo = np.matmul(T_actor_lh, SE3_inv(T_handle_oh))
        point, euler = T2xyzrpy(T_lo)
        grasps.append(Pose(point=point, euler=euler))
    return grasps


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
        T_child = gtem.get_tf(list2dict(get_configuration(robot_body), gtem.gscene.joint_names))
        pose = (T_child[:3, 3], Rotation.from_dcm(T_child[:3, :3]).as_quat())
        if gtem.name == root_name:
            pose_base = pose
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

body_subject_map = {}

def set_body_subject_map(pscene, body_names):
    gname_subject_map = {subj.geometry.get_root(): subj for subj in pscene.subject_dict.values()}
    body_subject_map.clear()
    body_subject_map.update({bid: gname_subject_map[gname] for bid, gname in body_names.items() if
                             gname in gname_subject_map})


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

    set_body_subject_map(pscene, body_names)
    actor = pscene.actor_dict[tool_name]
    update_grasp_info({tool_name: GraspInfo(
        lambda body: sample_grasps(body_subject_map=body_subject_map, body=body, actor=actor),
        approach_pose=Pose(0.1 * Point(z=1)))})

    return robot_body, body_names, movable_bodies