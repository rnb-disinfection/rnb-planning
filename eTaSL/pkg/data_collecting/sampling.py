import numpy as np
import time
from ..geometry.geometry import GEOTYPE

OBJ_GEN_LIST = [lambda L_MAX: (GEOTYPE.BOX, tuple(np.random.random((3,))*L_MAX), (0.0, 0.0, 0.9, 0.2)),
                lambda L_MAX: (GEOTYPE.CYLINDER, tuple((np.random.random((2,))*L_MAX)[[0,0,1]]), (0.0, 0.9, 0.0, 0.3))]

def sample_Trbt(Nrbt, robot_names, _wdh_rbt_min, _wdh_rbt_max, _min_dist_robot):
    Trbt = []
    while len(Trbt)<Nrbt:
        _T = (np.random.random(3)*np.subtract(_wdh_rbt_max, _wdh_rbt_min)+_wdh_rbt_min, (0,0,np.random.uniform(0,np.pi*2)))
        if not Trbt:
            Trbt.append(_T)
        else:
            if all([np.linalg.norm(_T[0]-_Told[0]) > _min_dist_robot for _Told in Trbt]):
                Trbt.append(_T)
    return {k:v for k,v in zip(robot_names, Trbt)}

def show_workspace(graph, Nwdh, CENTER, L_CELL, thickness=1e-2, alpha=0.1):
    Nw, Nd, Nh = Nwdh
    Ws, Ds, Hs = WDH = tuple(np.multiply((Nw, Nd, Nh), L_CELL))
    graph.add_geometry(graph.ghnd.create_safe(
        name="workspace", link_name="base_link", gtype=GEOTYPE.BOX,
        center=CENTER, rpy=(0, 0, 0), dims=WDH,
        color=(1, 1, 1, alpha), display=True, collision=False, fixed=True))
    time.sleep(0.01)
    for iw in range(int(Ws / L_CELL) + 1):
        for id in range(int(Ds / L_CELL) + 1):
            graph.add_geometry(graph.ghnd.create_safe(
                name="grid_xy_{}_{}".format(iw, id), link_name="base_link", gtype=GEOTYPE.BOX,
                center=(iw * L_CELL, id * L_CELL, CENTER[2]), rpy=(0, 0, 0), dims=(thickness, thickness, Hs),
                color=(0, 0, 0, alpha), display=True, collision=False, fixed=True))
            time.sleep(0.01)
    for id in range(int(Ds / L_CELL) + 1):
        for ih in range(int(Hs / L_CELL) + 1):
            graph.add_geometry(graph.ghnd.create_safe(
                name="grid_yz_{}_{}".format(id, ih), link_name="base_link", gtype=GEOTYPE.BOX,
                center=(CENTER[0], id * L_CELL, ih * L_CELL,), rpy=(0, 0, 0), dims=(Ws, thickness, thickness),
                color=(0, 0, 0, alpha), display=True, collision=False, fixed=True))
            time.sleep(0.01)
    for iw in range(int(Ws / L_CELL) + 1):
        for ih in range(int(Hs / L_CELL) + 1):
            graph.add_geometry(graph.ghnd.create_safe(
                name="grid_xz_{}_{}".format(iw, ih), link_name="base_link", gtype=GEOTYPE.BOX,
                center=(iw * L_CELL, CENTER[1], ih * L_CELL,), rpy=(0, 0, 0), dims=(thickness, Hs, thickness),
                color=(0, 0, 0, alpha), display=True, collision=False, fixed=True))
            time.sleep(0.01)

import os
import sys

sys.path.append(os.path.join(os.environ["TAMP_ETASL_DIR"], "openGJK/lib"))
import openGJKlib as oGJK
from ..geometry.geometry import DEFAULT_VERT_DICT, GEOTYPE, GeometryHandle
from ..utils.rotation_utils import Rot_rpy
from ..utils.utils import list2dict
import numpy as np

POINT_DEFAULT = np.array([[0,0,0]])
SEG_DEFAULT = np.array([[0,0,1.0],[0,0,-1.0]])/2
BOX_DEFAULT = np.array([[[(i,j,k) for k in range(2)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5

DEFAULT_VERT_DICT = {
    GEOTYPE.SPHERE: POINT_DEFAULT,
    GEOTYPE.CAPSULE: SEG_DEFAULT,
    GEOTYPE.BOX: BOX_DEFAULT,
    GEOTYPE.MESH: BOX_DEFAULT,
    GEOTYPE.CYLINDER: SEG_DEFAULT
}

def get_vertex_rows(gtype, xyz, rpy, dims):
    return np.matmul(Rot_rpy(rpy), (DEFAULT_VERT_DICT[gtype]*dims).transpose()).transpose()+xyz

def get_vertex_rows_T(gtype, T, dims):
    return np.matmul(T[:3,:3], (DEFAULT_VERT_DICT[gtype]*dims).transpose()).transpose()+T[:3,3]

def getPointList(point_rows_np):
    pl = oGJK.PointList()
    for v in point_rows_np:
        pl.append(oGJK.Point3(*v))
    return pl

def get_links(graph, Q_s):
    links = []
    for rname in graph.combined_robot.robot_names:
        links += [gtem for gtem in graph.ghnd if rname in gtem.name and gtem.collision]

    link_verts = []
    link_rads = []
    for link in links:
        T_link = link.get_tf(list2dict(Q_s, graph.combined_robot.joint_names))
        link_verts.append(get_vertex_rows_T(link.gtype, T_link, link.dims))
        if link.gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER, GEOTYPE.SPHERE]:
            link_rads.append(link.radius)
        else:
            link_rads.append(0)
    link_verts = np.array(link_verts)

    link_ctems = []
    for vert in link_verts:
        link_ctems.append(getPointList(vert))
    return links, link_verts, link_ctems, link_rads

def get_colliding_cells(Nwdh, L_CELL, ctems_ref, link_rads):
    Nw, Nd, Nh = Nwdh
    cell_mat=[]
    for i in range(Nw):
        dline = []
        for j in range(Nd):
            hline = []
            for k in range(Nh):
                center = np.add((i*L_CELL, j*L_CELL, k*L_CELL), L_CELL/2)
                hline.append(get_vertex_rows(GEOTYPE.BOX, center, (0,0,0), (L_CELL,L_CELL,L_CELL)))
            dline.append(hline)
        cell_mat.append(dline)

    dist_mat=np.zeros((Nw, Nd, Nh, len(ctems_ref)))
    coll_boxes = []
    for iw in range(Nw):
        for id in range(Nd):
            for ih in range(Nh):
                cell = getPointList(cell_mat[iw][id][ih])
                for il, ctem in zip(range(len(ctems_ref)), ctems_ref):
                    dist_mat[iw, id, ih, il] = oGJK.gjk_cpp(cell, ctem)-link_rads[il]
                if any(dist_mat[iw, id, ih] < 0):
                    coll_boxes.append([iw, id, ih])

    return coll_boxes

def get_distance_list(ctem_sub, ctems_ref, rad_sub, rads_ref):
    return  [
        oGJK.gjk_cpp(
            ctem_sub, ctem_ref)-(rad_sub+rad_ref) for ctem_ref, rad_ref in zip(ctems_ref, rads_ref)]


def get_distance_mat(ctems_subj, ctems_ref, rads_subj, rads_ref):
    return np.array(
        [get_distance_list(ctem_sub, ctems_ref, rad_subj, rads_ref)
         for ctem_sub, rad_subj in zip(ctems_subj, rads_subj)]
    )

def get_reachable_cells(Nwdh, L_CELL, reach_center_dict, MAX_REACH_DICT, robot_names=None):
    if robot_names is None:
        robot_names = sorted(reach_center_dict.keys())
    Nw, Nd, Nh = Nwdh
    centers = np.zeros(Nwdh+(3,))
    for iw in range(Nw):
        centers[iw,:,:,0] = (iw+0.5)*L_CELL
    for id in range(Nd):
        centers[:,id,:,1] = (id+0.5)*L_CELL
    for ih in range(Nh):
        centers[:,:,ih,2] = (ih+0.5)*L_CELL
    reachable_bool_dict = {k: np.linalg.norm(np.subtract(centers, rcenter), axis=-1) < MAX_REACH_DICT[k] for k, rcenter in reach_center_dict.items()}
    reachable_bool_list = [reachable_bool_dict[k] for k in robot_names]
    reachable_bool_all = np.any(reachable_bool_list, axis=0)
    free_boxes = []
    for iw in range(Nw):
        for id in range(Nd):
            for ih in range(Nh):
                if reachable_bool_all[iw, id, ih]:
                    free_boxes.append([iw, id, ih])
    return free_boxes

import random

def sample_obs_goal_boxes(free_boxes, Ns, Nt):
    obs_boxes = random.sample(free_boxes, Ns)
    goal_boxes = random.sample(free_boxes, Nt)
    return obs_boxes, goal_boxes

def remove_geometries_by_prefix(graph, ID):
    for gtem in [gtem for gtem in graph.ghnd if gtem.name.startswith(ID)]:
        graph.remove_geometry(gtem)

# draw cells
def draw_cells(graph, ID, cells, L_CELL, color, link_name="base_link"):
    remove_geometries_by_prefix(graph, ID)
    gtem_list=[]
    for cell in cells:
        gtem = graph.ghnd.create_safe(
            name="{}_{}_{}_{}".format(ID, *cell), link_name=link_name, gtype=GEOTYPE.BOX,
            center=tuple(np.multiply(cell, L_CELL)+L_CELL/2), rpy=(0, 0, 0), dims=(L_CELL, L_CELL, L_CELL),
            color=color, display=True, collision=False, fixed=True)
        gtem_list.append(gtem)
        graph.add_geometry(gtem)

def make_colliding_pairs(geometry_items1, geometry_items2=None, min_distance_map=None):
    collision_pairs = []
    for idx1, ctem1 in zip(range(len(geometry_items1)), geometry_items1):
        if geometry_items2 is None:
            idx2_off = idx1+1
            geometry_items_tmp = geometry_items1[idx2_off:]
        else:
            idx2_off = 0
            geometry_items_tmp = geometry_items2

        for idx2, ctem2 in zip(range(len(geometry_items_tmp)), geometry_items_tmp):
            if ctem2.link_name in ctem1.adjacent_links or ctem1.link_name in ctem2.adjacent_links:
                continue
            else:
                if min_distance_map is not None:
                    min_link_dist = min_distance_map[ctem1.link_name][ctem2.link_name]
                    min_col_dist = min_link_dist - (np.linalg.norm(ctem1.get_off_max()) + np.linalg.norm(ctem2.get_off_max()))
                    if min_col_dist > 0:
                        continue
                collision_pairs.append((idx1, idx2+idx2_off))
    return collision_pairs

def sample_joint(graph, Q_s_loaded=None):
    joint_lims = []
    for jname in graph.combined_robot.joint_names:
        joint = graph.urdf_content.joint_map[jname]
        joint_lims.append([joint.limit.upper, joint.limit.lower])
    joint_lims=np.array(joint_lims)
    Qmax, Qmin = joint_lims[:,0],joint_lims[:,1]
    colliding_pairs=None
    while True:
        Q_s = Q_s_loaded or np.random.uniform(Qmin, Qmax, size=Qmax.shape)
        links, link_verts, link_ctems, link_rads = get_links(graph, Q_s)
        colliding_pairs = colliding_pairs or make_colliding_pairs(links)
        valid_config = True
        for cid1, cid2 in colliding_pairs:
             if oGJK.gjk_cpp(link_ctems[cid1], link_ctems[cid2])-(link_rads[cid1]+link_rads[cid2]) < 0:
                    valid_config = False
                    print("link collision detected: {} - {}".format(links[cid1].name,links[cid2].name))
                    break
        if valid_config:
            break
    return Q_s, links, link_verts, link_ctems, link_rads

from ..constants import DIR_RPY_DICT
from ..utils.rotation_utils import SE3, Rot_axis, SE3_inv, Rot2rpy

def sample_grasp(obj, WIDTH_RANGE, DEPTH_RANGE, DIM_MAX, fit_dim=False):
    R_ygrip = np.matmul(
        np.matmul(Rot_rpy(random.choice(DIR_RPY_DICT.values())), Rot_rpy((0, 0, random.uniform(0, np.pi * 2)))),
        Rot_rpy((np.pi / 2, 0, 0)))
    zdir = R_ygrip[:, 2]
    ax_grip = np.argmax(np.abs(R_ygrip[:, 1]))
    if fit_dim:
        dims_bak = obj.dims
        gwidth = np.random.uniform(*WIDTH_RANGE)
        dim_grip = dims_bak[ax_grip]
        dims_new = np.multiply(dims_bak, gwidth / dim_grip)
        if max(dims_new) > DIM_MAX:
            dims_new = dims_new / max(dims_new) * DIM_MAX
    else:
        dims_new = dims_bak = obj.dims
    if obj.gtype==GEOTYPE.BOX:
        grip_range = np.array(dims_new) / 2
        grip_ref = np.sign(-zdir) * grip_range
        grip_ref[ax_grip] = 0
        if random.random()>0.5:
            ax_end, ax_move =  (ax_grip+1)%3, (ax_grip+2)%3
        else:
            ax_end, ax_move =  (ax_grip+2)%3, (ax_grip+1)%3
        grip_ref[ax_end] = grip_ref[ax_end]
        grip_ref[ax_move] = random.uniform(-dims_new[ax_move]/2, dims_new[ax_move]/2)
        P_zo = SE3(np.identity(3), grip_ref)
        depth = random.uniform(*DEPTH_RANGE)
        P_zg = SE3(np.identity(3), [0, 0, -depth])
        T_ygrip = np.matmul(P_zo, np.matmul(SE3(R_ygrip, [0] * 3), P_zg))
    elif obj.gtype in [GEOTYPE.CYLINDER, GEOTYPE.CAPSULE]:
        if ax_grip == 2:
            theta = random.uniform(np.pi - np.pi / 3, np.pi + np.pi / 3)
            R_z = SE3(Rot_axis(2, theta), [0, 0, 0])
            r_ref = dims_new[0] / 2
            P_r = SE3(np.identity(3), [0, 0, r_ref])
            depth = random.uniform(*DEPTH_RANGE)
            P_z = SE3(np.identity(3), [0, 0, -depth])
            T_off = np.matmul(np.matmul(np.matmul(R_z, P_r), SE3_inv(R_z)), P_z)
            T_ygrip = np.matmul(SE3(R_ygrip, [0] * 3), T_off)
        else:
            DEPTH_RANGE= (min(DEPTH_RANGE[0]+dims_new[0] / 2, DEPTH_RANGE[1]), DEPTH_RANGE[1])
            h_half = dims_new[2] / 2
            P_zo = SE3(np.identity(3),
                       np.random.uniform(np.array([0, 0, -h_half]), np.array([0, 0, h_half]), size=(3,)))
            depth = random.uniform(*DEPTH_RANGE)
            P_zg = SE3(np.identity(3), [0, 0, -depth])
            T_ygrip = np.matmul(P_zo, np.matmul(SE3(R_ygrip, [0] * 3), P_zg))
    return T_ygrip, dims_new, dims_bak

def sample_putpoint(tar):
    R_zplace = np.matmul(
        Rot_rpy(random.choice(DIR_RPY_DICT.values())),
        Rot_rpy((0, 0, random.uniform(0, np.pi * 2))))
    ax_z = np.argmax(np.abs(R_zplace[:, 2]))
    ax_xy = np.argmax(np.abs(R_zplace[:, :2]), axis=0)
    dim_xy = np.array(tar.dims)[ax_xy]
    move_axis = np.zeros((3,))
    move_axis[ax_xy] = np.random.random((2,)) - 0.5
    P_xy = np.multiply(tar.dims, move_axis)
    T_xy = SE3(np.identity(3), P_xy)
    P_zplace = [0, 0] + [tar.dims[ax_z] / 2]
    T_zplace = np.matmul(T_xy, np.matmul(SE3(R_zplace, [0, ] * 3), SE3(np.identity(3), P_zplace)))
    T_lp = np.matmul(tar.Toff, T_zplace)
    return T_lp

def sample_putobject(tar, T_lp, L_MAX):
    geo_gen = random.choice(OBJ_GEN_LIST)
    gtype, dims, color = geo_gen(L_MAX)
    Rz = Rot_rpy(random.choice(DIR_RPY_DICT.values()))
    ax_z = np.argmax(np.abs(Rz[:, 2]))
    Tzoff = SE3(Rz, [0, 0, dims[ax_z] / 2])
    T_lo = np.matmul(T_lp, Tzoff)
    ontarget = GeometryHandle.instance().create_safe(
        name="ontarget", link_name=tar.link_name, gtype=gtype,
        center=T_lo[:3, 3], rpy=Rot2rpy(T_lo[:3, :3]), dims=dims,
        color=(1,0,0,0.5), display=True, collision=False, fixed=False)
    return ontarget, T_lo

def gtem_to_dict(gtem):
    return {"name":gtem.name, "gtype": gtem.gtype.name, "link_name":gtem.link_name,
            "center":gtem.center, "rpy":gtem.rpy, "dims":gtem.dims,
            "color":gtem.color, "display":gtem.display,
            "collision": gtem.collision, "fixed": gtem.fixed, "soft": gtem.soft}

def dict_to_gtem(gdict):
    return GeometryHandle.instance().create_safe(**gdict)



########################### pick sampling functions @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
def sample_pick(GRIPPER_REFS, obj_list, L_MAX):
    rname, gripper = random.choice(GRIPPER_REFS.items())
    depth_range = gripper['depth_range']
    width_range = gripper['width_range']
    obj = random.choice(obj_list)
    color_bak = obj.color
    obj.color = (1,0,0,0.5)
    T_og, dims_new, dims_bak = sample_grasp(
        obj, WIDTH_RANGE=width_range, DEPTH_RANGE=depth_range, DIM_MAX=L_MAX, fit_dim=True)
    obj.dims = dims_new
    T_lg = SE3(np.identity(3), gripper['tcp_ref'])
    T_lgo = np.matmul(T_lg, SE3_inv(T_og))
    inhand = GeometryHandle.instance().create_safe(
        name="inhand", link_name=gripper["link_name"], gtype=obj.gtype,
        center=T_lgo[:3, 3], rpy=Rot2rpy(T_lgo[:3, :3]), dims=obj.dims,
        color=(1, 0, 0, 0.5), display=True, collision=False, fixed=False)
    return rname, inhand, obj, None, dims_bak, color_bak

def sample_place(GRIPPER_REFS, obj_list, L_CELL):
    rname, gripper = random.choice(GRIPPER_REFS.items())
    depth_range = gripper['depth_range']
    width_range = gripper['width_range']
    tar = random.choice(obj_list)
    color_bak = tar.color
    T_lp = sample_putpoint(tar)
    ontarget, T_lo = sample_putobject(tar, T_lp, L_CELL)
    T_ygrip, dims_new, dims_bak = sample_grasp(
        ontarget, WIDTH_RANGE=width_range, DEPTH_RANGE=depth_range, DIM_MAX=L_CELL, fit_dim=False)
    T_glo = np.matmul(SE3(np.identity(3),gripper['tcp_ref']), SE3_inv(T_ygrip))
    inhand = GeometryHandle.instance().create_safe(
        name="inhand", link_name=gripper["link_name"], gtype=ontarget.gtype,
        center=T_glo[:3,3], rpy=Rot2rpy(T_glo[:3,:3]), dims=ontarget.dims,
        color=(1,0,0,0.5), display=True, collision=True, fixed=False)
    return rname, inhand, ontarget, None, dims_bak, color_bak


def log_manipulation(SAMPLED_DATA, key, rname1, obj1, obj2, rname2, dims_bak, color_bak):
    SAMPLED_DATA["ACTION"][key] = {"rname1": rname1, "obj1": gtem_to_dict(obj1),
                                   "obj2": gtem_to_dict(obj2), "rname2": rname2,
                                   "dims_bak":dims_bak, "color_bak":color_bak}

def load_manipulation(SAMPLED_DATA, key):
    rname1, obj1, obj2, rname2, dims_bak, color_bak = [
        SAMPLED_DATA["ACTION"][key][prm] for prm in ["rname1", "obj1", "obj2", "rname2", "dims_bak", "color_bak"]]
    return rname1, dict_to_gtem(obj1), dict_to_gtem(obj2), rname2, dims_bak, color_bak

def show_manip_coords(graph, GRIPPER_REFS, key, rname1, obj1, obj2, rname2, axis_len=0.05):
    ## show target objects
    graph.remove_geometry(obj1)
    graph.add_geometry(obj1)
    graph.remove_geometry(obj2)
    graph.add_geometry(obj2)

    gripper1 = GRIPPER_REFS[rname1] if rname1 else None
    show_grip_axis(graph, key, gripper1, obj1, obj2, axis_len)
    gripper2 = GRIPPER_REFS[rname2] if rname2 else None
    show_grip_axis(graph, key, gripper2, obj2, obj1, axis_len)

def show_grip_axis(graph, key, gripper, obj1, obj2, axis_len=0.5):
    T_lo, T_lo2 = obj1.Toff, obj2.Toff
    graph.add_highlight_axis(key, "{}_grip".format(obj1.name), obj1.link_name, T_lo[:3, 3],
                             T_lo[:3, :3], color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))
    if gripper:
        T_lg = SE3(np.identity(3), gripper['tcp_ref'])
        glink =gripper["link_name"]
        graph.add_highlight_axis(key, "{}_tcp".format(glink), glink, T_lg[:3, 3],
                                 T_lg[:3, :3], color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))
        T_go = np.matmul(SE3_inv(T_lg), T_lo)
        T_lo2g= np.matmul(T_lo2, SE3_inv(T_go))
        graph.add_highlight_axis(key, "{}_grip".format(obj2.name), obj2.link_name, T_lo2g[:3, 3],
                                 T_lo2g[:3, :3], color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))


def reset_rendering(graph, key, obj_keep_list, obj_virtual_list, dims_bak=None, color_bak=None):
    graph.clear_highlight(key)
    for obj_keep in obj_keep_list:
        obj_keep.dims, obj_keep.color = dims_bak, color_bak
        graph.remove_geometry(obj_keep)
        graph.add_geometry(obj_keep)
    for obj_virtual in obj_virtual_list:
        graph.remove_geometry(obj_virtual)

########################### place sampling functions @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@