import numpy as np
import time
from ..geometry.geometry import GEOTYPE
from ..utils.utils import Logger
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

def show_workspace(graph, Nwdh, CENTER, L_CELL, thickness=1e-2, alpha=0.1, WDH_s=(0,0,0), WDH_e=None):
    Nw, Nd, Nh = Nwdh
    Ws, Ds, Hs = WDH = tuple(np.multiply((Nw, Nd, Nh), L_CELL))
    WDH_e = WDH_e or (int(Ws / L_CELL) + 1, int(Ds / L_CELL) + 1, int(Hs / L_CELL) + 1)
    graph.add_marker(graph.ghnd.create_safe(
        name="workspace", link_name="base_link", gtype=GEOTYPE.BOX,
        center=CENTER, rpy=(0, 0, 0), dims=WDH,
        color=(1, 1, 1, alpha), display=True, collision=False, fixed=True))
    time.sleep(0.01)
    for iw in range(WDH_s[0],WDH_e[0]):
        for id in range(WDH_s[1],WDH_e[1]):
            graph.add_marker(graph.ghnd.create_safe(
                name="grid_xy_{}_{}".format(iw, id), link_name="base_link", gtype=GEOTYPE.BOX,
                center=(iw * L_CELL, id * L_CELL, CENTER[2]), rpy=(0, 0, 0), dims=(thickness, thickness, Hs),
                color=(0, 0, 0, alpha), display=True, collision=False, fixed=True))
            time.sleep(0.01)
    for id in range(WDH_s[1],WDH_e[1]):
        for ih in range(WDH_s[2],WDH_e[2]):
            graph.add_marker(graph.ghnd.create_safe(
                name="grid_yz_{}_{}".format(id, ih), link_name="base_link", gtype=GEOTYPE.BOX,
                center=(CENTER[0], id * L_CELL, ih * L_CELL,), rpy=(0, 0, 0), dims=(Ws, thickness, thickness),
                color=(0, 0, 0, alpha), display=True, collision=False, fixed=True))
            time.sleep(0.01)
    for iw in range(WDH_s[0],WDH_e[0]):
        for ih in range(WDH_s[2],WDH_e[2]):
            graph.add_marker(graph.ghnd.create_safe(
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

def get_cell(center, L_CELL, Nwdh):
    return tuple(np.clip(np.round(np.divide(center, L_CELL) - 0.5).astype(np.int),[0,0,0], np.subtract(Nwdh, 1)))

def get_centers(Nwdh, L_CELL):
    Nw, Nd, Nh = Nwdh
    centers = np.zeros(Nwdh+(3,))
    for iw in range(Nw):
        centers[iw,:,:,0] = (iw+0.5)*L_CELL
    for id in range(Nd):
        centers[:,id,:,1] = (id+0.5)*L_CELL
    for ih in range(Nh):
        centers[:,:,ih,2] = (ih+0.5)*L_CELL
    return centers
    

def get_reachable_cells(Nwdh, L_CELL, reach_center_dict, MAX_REACH_DICT, robot_names=None):
    if robot_names is None:
        robot_names = sorted(reach_center_dict.keys())
    Nw, Nd, Nh = Nwdh
    centers = get_centers(Nwdh, L_CELL)
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
    for cell in cells:
        gtem = graph.ghnd.create_safe(
            name="{}_{}_{}_{}".format(ID, *cell), link_name=link_name, gtype=GEOTYPE.BOX,
            center=tuple(np.multiply(cell, L_CELL)+L_CELL/2), rpy=(0, 0, 0), dims=(L_CELL, L_CELL, L_CELL),
            color=color, display=True, collision=False, fixed=True)
        graph.add_marker(gtem)

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
    for _ in range(100):
        Q_s = np.random.uniform(Qmin, Qmax, size=Qmax.shape) if Q_s_loaded is None else Q_s_loaded
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

def sample_putobject(tar, T_lp, L_MAX, ghnd):
    geo_gen = random.choice(OBJ_GEN_LIST)
    gtype, dims, color = geo_gen(L_MAX)
    Rz = Rot_rpy(random.choice(DIR_RPY_DICT.values()))
    ax_z = np.argmax(np.abs(Rz[:, 2]))
    Tzoff = SE3(Rz, [0, 0, dims[ax_z] / 2])
    T_lo = np.matmul(T_lp, Tzoff)
    ontarget = ghnd.create_safe(
        name="ontarget", link_name=tar.link_name, gtype=gtype,
        center=T_lo[:3, 3], rpy=Rot2rpy(T_lo[:3, :3]), dims=dims,
        color=(1,0,0,0.5), display=True, collision=False, fixed=False)
    return ontarget, T_lo

def gtem_to_dict(gtem):
    return {"name":gtem.name, "gtype": gtem.gtype.name, "link_name":gtem.link_name,
            "center":gtem.center, "rpy":gtem.rpy, "dims":gtem.dims,
            "color":gtem.color, "display":gtem.display,
            "collision": gtem.collision, "fixed": gtem.fixed, "soft": gtem.soft}

def dict_to_gtem(gdict, ghnd):
    return ghnd.create_safe(**gdict)



########################### pick sampling functions @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
def sample_pick(GRIPPER_REFS, obj_list, L_CELL, ghnd):
    rname, gripper = random.choice(GRIPPER_REFS.items())
    depth_range = gripper['depth_range']
    width_range = gripper['width_range']
    obj = random.choice(obj_list)
    color_bak = obj.color
    obj.color = (1,0,0,1)
    T_og, dims_new, dims_bak = sample_grasp(
        obj, WIDTH_RANGE=width_range, DEPTH_RANGE=depth_range, DIM_MAX=L_CELL, fit_dim=True)
    obj.dims = dims_new
    T_lg = SE3(np.identity(3), gripper['tcp_ref'])
    T_lgo = np.matmul(T_lg, SE3_inv(T_og))
    inhand = ghnd.create_safe(
        name="inhand", link_name=gripper["link_name"], gtype=obj.gtype,
        center=T_lgo[:3, 3], rpy=Rot2rpy(T_lgo[:3, :3]), dims=obj.dims,
        color=(1, 0, 0, 0.5), display=True, collision=False, fixed=False)
    return rname, inhand, obj, None, dims_bak, color_bak

def sample_place(GRIPPER_REFS, obj_list, L_CELL, ghnd):
    rname, gripper = random.choice(GRIPPER_REFS.items())
    depth_range = gripper['depth_range']
    width_range = gripper['width_range']
    tar = random.choice(obj_list)
    color_bak = tar.color
    T_lp = sample_putpoint(tar)
    ontarget, T_lo = sample_putobject(tar, T_lp, L_CELL, ghnd)
    T_ygrip, dims_new, dims_bak = sample_grasp(
        ontarget, WIDTH_RANGE=width_range, DEPTH_RANGE=depth_range, DIM_MAX=L_CELL, fit_dim=False)
    T_glo = np.matmul(SE3(np.identity(3),gripper['tcp_ref']), SE3_inv(T_ygrip))
    inhand = ghnd.create_safe(
        name="inhand", link_name=gripper["link_name"], gtype=ontarget.gtype,
        center=T_glo[:3,3], rpy=Rot2rpy(T_glo[:3,:3]), dims=ontarget.dims,
        color=(1,0,0,1), display=True, collision=True, fixed=False)
    return rname, inhand, ontarget, None, dims_bak, color_bak

def sample_handover(src, tar, L_CELL, ghnd):
    gtype, dims, color = random.choice(OBJ_GEN_LIST)(L_CELL)
    handed = ghnd.create_safe(gtype=gtype, name="handed_in_src", link_name=src[1]['link_name'],
                                    center=(0,0,0), rpy=(0,0,0), dims=dims, color=(0,1,1,1),
                                    display=True, collision=True, fixed=False)
    Ttar_ygrip, dims_new, dims_bak = sample_grasp(
        handed, WIDTH_RANGE=tar[1]['width_range'], DEPTH_RANGE=tar[1]['depth_range'],
        DIM_MAX=L_CELL, fit_dim=True)
    handed.dims = dims_new
    Tsrc_ygrip, dims_new, dims_bak = sample_grasp(
        handed, WIDTH_RANGE=src[1]['width_range'], DEPTH_RANGE=src[1]['depth_range'],
        DIM_MAX=L_CELL, fit_dim=False)
    T_slo = np.matmul(SE3(np.identity(3),src[1]['tcp_ref']), SE3_inv(Tsrc_ygrip))
    handed.set_offset_tf(center=T_slo[:3,3], orientation_mat=T_slo[:3,:3])
    T_tlo = np.matmul(SE3(np.identity(3),tar[1]['tcp_ref']), SE3_inv(Ttar_ygrip))
    intar = ghnd.create_safe(gtype=handed.gtype, name="handed_in_tar", link_name=tar[1]['link_name'],
                                   center=T_tlo[:3,3], rpy=Rot2rpy(T_tlo[:3,:3]), dims=handed.dims, color=(1,0,0.5,0.5),
                                   display=True, collision=False, fixed=False)
    return src[0], handed, intar, tar[0], (0.1,)*3, (0.5,)*4

def log_manipulation(SAMPLED_DATA, key, rname1, obj1, obj2, rname2, dims_bak, color_bak):
    SAMPLED_DATA["ACTION"][key] = {"rname1": rname1, "obj1": gtem_to_dict(obj1),
                                   "obj2": gtem_to_dict(obj2), "rname2": rname2,
                                   "dims_bak":dims_bak, "color_bak":color_bak}

def load_manipulation(SAMPLED_DATA, key, ghnd):
    rname1, obj1, obj2, rname2, dims_bak, color_bak = [
        SAMPLED_DATA["ACTION"][key][prm] for prm in ["rname1", "obj1", "obj2", "rname2", "dims_bak", "color_bak"]]
    return rname1, dict_to_gtem(obj1, ghnd), dict_to_gtem(obj2, ghnd), rname2, dims_bak, color_bak

def show_manip_coords(graph, GRIPPER_REFS, key, rname1, obj1, obj2, rname2, axis_len=0.05):
    ## show target objects
    graph.remove_marker(obj1)
    graph.add_marker(obj1)
    graph.remove_marker(obj2)
    graph.add_marker(obj2)

    gripper1 = GRIPPER_REFS[rname1] if rname1 else None
    show_grip_axis(graph, key, gripper1, obj1, obj2, axis_len)
    gripper2 = GRIPPER_REFS[rname2] if rname2 else None
    show_grip_axis(graph, key, gripper2, obj2, obj1, axis_len)

def show_grip_axis(graph, key, gripper, obj1, obj2, axis_len=0.5):
    T_lo, T_lo2 = obj1.Toff, obj2.Toff
    graph.add_highlight_axis(key, "{}_grip".format(obj1.name), obj1.link_name, T_lo[:3, 3], T_lo[:3, :3],
                             color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))
    if gripper:
        T_lg = SE3(np.identity(3), gripper['tcp_ref'])
        glink =gripper["link_name"]
        graph.add_highlight_axis(key, "{}_tcp".format(glink), glink, T_lg[:3, 3], T_lg[:3, :3],
                                 color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))
        T_go = np.matmul(SE3_inv(T_lg), T_lo)
        T_lo2g= np.matmul(T_lo2, SE3_inv(T_go))
        graph.add_highlight_axis(key, "{}_grip".format(obj2.name), obj2.link_name, T_lo2g[:3, 3], T_lo2g[:3, :3],
                                 color=None, axis="xyz", dims=(axis_len, axis_len / 10, axis_len / 10))


def reset_rendering(graph, key, obj_keep_list, obj_virtual_list, dims_bak=None, color_bak=None, vis=True, sleep=False):
    graph.clear_highlight(key)
    for obj_keep in obj_keep_list:
        graph.ghnd.NAME_DICT[obj_keep.name].dims = dims_bak
        graph.ghnd.NAME_DICT[obj_keep.name].color = color_bak
        graph.remove_marker(obj_keep, vis=vis, sleep=sleep)
        graph.add_marker(obj_keep, vis=vis)
    for obj_virtual in obj_virtual_list:
        graph.remove_geometry(obj_virtual, sleep=sleep)

########################### place sampling functions @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
from ..constraint_graph import State

def get_pick_states(graph, GRIPPER_REFS, rname, inhand, obj, tar, Q_s):
    T_lgo, T_bo = inhand.Toff, obj.Toff
    bname = GRIPPER_REFS[rname]["bname"]
    T_lg = graph.binder_dict[bname].Toff_lh
    T_bgl = np.matmul(T_bo, SE3_inv(T_lgo))
    T_bg = np.matmul(T_bgl, T_lg)
    state_s = State((("virtual", "point", "base"),), {"virtual":T_bg}, Q_s, graph)
    graph.set_object_state(state_s)
    state_g = state_s.copy(graph)
    state_g.node = (("virtual", "point", bname),)
    return state_s, state_g

def test_pick(graph, GRIPPER_REFS, rname, inhand, obj, tar, Q_s, mplan, **kwargs):
    mplan.update(graph)
    state_s, state_g = get_pick_states(graph, GRIPPER_REFS, rname, inhand, obj, tar, Q_s)
    mplan.update(graph)
    return mplan.plan_transition(state_s, state_g, state_g.node, **kwargs)

def get_place_states(graph, GRIPPER_REFS, rname, inhand, ontarget, tar, Q_s):
    T_lgo, T_bo = inhand.Toff, ontarget.Toff
    bname = GRIPPER_REFS[rname]["bname"]
    T_lg = graph.binder_dict[bname].Toff_lh
    T_bgl = np.matmul(T_bo, SE3_inv(T_lgo))
    T_bg = np.matmul(T_bgl, T_lg)
    state_s = State((("virtual", "point", "base"),), {"virtual":T_bg}, Q_s, graph)
    graph.set_object_state(state_s)
    state_g = state_s.copy(graph)
    state_g.node = (("virtual", "point", bname),)
    return state_s, state_g

def test_place(graph, GRIPPER_REFS, rname, inhand, ontarget, tar, Q_s, mplan, **kwargs):
    mplan.update(graph)
    state_s, state_g = get_place_states(graph, GRIPPER_REFS, rname, inhand, ontarget, tar, Q_s)
    mplan.update(graph)
    return mplan.plan_transition(state_s, state_g, state_g.node, **kwargs)

from ..planner.moveit.moveit_planner import transfer_ctem
from ..utils.utils import list2dict, dict2list

def get_handover_states(graph, GRIPPER_REFS, src, handed, intar, tar, Q_s):
    Q_s_new = np.array(list(reversed(-Q_s[graph.combined_robot.idx_dict[src]])) + list(Q_s[graph.combined_robot.idx_dict[tar]]))
    T_lso, T_lto = handed.Toff, intar.Toff
    sbname = GRIPPER_REFS[src]["bname"]
    tbname = GRIPPER_REFS[tar]["bname"]
    T_lt = graph.binder_dict[tbname].Toff_lh
    T_lstl = np.matmul(T_lso, SE3_inv(T_lto))
    T_lst = np.matmul(T_lstl, T_lt)
    state_s = State((("virtual", "point", sbname),), {"virtual":T_lst}, Q_s_new, graph)
    graph.set_object_state(state_s)
    state_g = state_s.copy(graph)
    state_g.node = (("virtual", "point", tbname),)
    return state_s, state_g

def test_handover(graph, GRIPPER_REFS, src, handed, intar, tar, Q_s, mplan,
                  N=250, dt=0.04, vel_conv=0.5e-2, err_conv=1e-3, **kwargs):
    graph.ghnd.update()
    state_s, state_g = get_handover_states(graph, GRIPPER_REFS, src, handed, intar, tar, Q_s)
    graph.ghnd.update()
    transfer_ctem(graph.ghnd, mplan.ghnd)
    mplan.update(graph)
    transfer_ctem(graph.ghnd, mplan.ghnd)
    trajectory, Q_last, error, success = mplan.plan_transition(state_s, state_g, state_g.node,
                          N=N, dt=dt, vel_conv=vel_conv, err_conv=err_conv,
                          **kwargs)
    if success:
        trajectory = np.array([dict2list(list2dict(traj, mplan.joint_names), graph.joint_names) for traj in trajectory])
        trajectory[:, graph.combined_robot.idx_dict[src]] *= -1
    return trajectory, Q_last, error, success

def test_full_mp(dcol, GRIPPER_REFS, Q_s, dual_mplan_dict, mplan, ID=None, UPDATE_DAT=True, VISUALIZE=False, timeout=1):
    graph = dcol.graph
    elog = Logger()
    acquired = False
    for skey in sorted(dcol.snode_dict.keys()):
        try:
            dcol.dict_lock.acquire()
            acquired = True
            snode = dcol.snode_dict[skey]
            dcol.dict_lock.release()
            acquired = False
            rname, inhand, obj, tar, dims_bak, color_bak, succ, _ = load_manipulation_from_dict(snode,
                                                                                                graph.ghnd)
        except Exception as e:
            if acquired:
                dcol.dict_lock.release()
            if not elog.log(str(e)):
                break
            continue
        try:
            success_now = False
            if rname and tar:  # handover case
                remove_map = [[], [0, 1]]
                action_type = "HANDOVER"
                trajectory, Q_last, error, success_now = test_handover(graph, GRIPPER_REFS, rname, inhand,
                                                                       obj, tar, Q_s,
                                                                       dual_mplan_dict[(rname, tar)], timeout=timeout)
            elif inhand.collision:  # place case
                remove_map = [[], [0, 1]]
                action_type = "PLACE"
                trajectory, Q_last, error, success_now = test_place(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                    tar, Q_s, mplan, timeout=timeout)
            elif obj.collision:  # pick case
                remove_map = [[1], [0]]
                action_type = "PICK"
                trajectory, Q_last, error, success_now = test_pick(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                   tar, Q_s, mplan, timeout=timeout)
            else:
                remove_map = [[], [0,1]]
                action_type = "None"
                raise (RuntimeError("non-implemented case"))

            check_result = succ == success_now
            print("DATA CHECK {}: ({}->{})".format("SUCCESS" if check_result else "FAILURE", succ, success_now))
            if UPDATE_DAT:
                dcol.dict_lock.acquire()
                acquired = True
                snode = dcol.snode_dict[skey]
                if "succ_count" not in snode:
                    snode["succ_count"] = 0
                snode["succ_count"] += success_now
                dcol.snode_dict[skey] = snode
                dcol.dict_lock.release()
                acquired = False

            if VISUALIZE and success_now:
                show_manip_coords(graph, GRIPPER_REFS, action_type, rname, inhand, obj, rname2=tar)
                graph.show_motion(trajectory, period=0.1)
        except Exception as e:
            if acquired:
                dcol.dict_lock.release()
            if not elog.log(str(e)):
                break
        finally:
            remove1 = [[inhand, obj][iii] for iii in remove_map[0]]
            remove2 = [[inhand, obj][iii] for iii in remove_map[1]]
            reset_rendering(graph, action_type, remove1, remove2, dims_bak, color_bak, sleep=True,
                            vis=VISUALIZE)
            
    print("============================ TERMINATE {} ===================================".format(ID))

def load_manipulation_from_dict(dict_log, ghnd):
    rname1, obj1, obj2, rname2, dims_bak, color_bak, success = [
        dict_log[prm] for prm in ["rname1", "obj1", "obj2", "rname2", "dims_bak", "color_bak", "success"]]
    trajectory = dict_log['trajectory'] if 'trajectory' in dict_log else []
    return rname1, dict_to_gtem(obj1, ghnd), dict_to_gtem(obj2, ghnd), rname2, dims_bak, color_bak, success, trajectory


from multiprocessing import Process, Lock, cpu_count
from multiprocessing.managers import SyncManager
import random

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)

class DataCollector:
    def __init__(self, graph, GRIPPER_REFS, S_F_RATIO=2.0):
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.dict_lock = self.manager.Lock()
        self.graph = graph
        self.ghnd = graph.ghnd
        self.GRIPPER_REFS = GRIPPER_REFS
        self.S_F_RATIO = S_F_RATIO

    def pick_search(self, ID, obj_list, Q_s, mplan, L_CELL, timeout=1, N_search=100, N_retry=5):
        graph, GRIPPER_REFS = self.graph, self.GRIPPER_REFS
        fail_count = 0
        succ_count = 0
        elog = Logger()
        acquired = False
        for i in range(N_search):
            try:
                rname, inhand, obj, _, dims_bak, color_bak = sample_pick(GRIPPER_REFS, obj_list, L_CELL, self.ghnd)
                for _ in range(N_retry):
                    trajectory, Q_last, error, success = test_pick(graph, GRIPPER_REFS, rname, inhand, obj, None, Q_s,
                                                                   mplan, timeout=timeout)
                    if success: break
                print("{}: {} - {}".format(ID, "SUCCESS" if success else "FAILURE", i))
                if success or fail_count < succ_count * self.S_F_RATIO:
                    self.dict_lock.acquire()
                    acquired = True
                    idx = self.snode_counter.value
                    self.snode_dict[idx] = {
                        "rname1": rname, "obj1": gtem_to_dict(inhand),
                        "obj2": gtem_to_dict(obj), "rname2": None, "dims_bak": dims_bak, "color_bak": color_bak,
                        "success": success, "trajectory": trajectory}
                    self.snode_counter.value = self.snode_counter.value + 1
                    self.dict_lock.release()
                    acquired = False
                    if success:
                        succ_count += 1
                    else:
                        fail_count += 1
                    print(
                        "=========== {} {} {} =========== - {}".format(rname, ID, "SUCCESS" if success else "FAILURE", idx))
            except Exception as e:
                if not elog.log(str(e)):
                    break
                if acquired:
                    self.dict_lock.release()
            finally:
                reset_rendering(graph, "PICK", [obj], [inhand], dims_bak, color_bak, vis=False)
        print("=============== TERMINATE {} ==============".format(ID))

    def place_search(self, ID, obj_list, Q_s, mplan, L_CELL, timeout=1, N_search=100, N_retry=1):
        graph, GRIPPER_REFS = self.graph, self.GRIPPER_REFS
        fail_count = 0
        succ_count = 0
        elog = Logger()
        acquired = False
        for i in range(N_search):
            try:
                rname, inhand, ontarget, _, dims_bak, color_bak = sample_place(GRIPPER_REFS, obj_list, L_CELL, self.ghnd)
                for _ in range(N_retry):
                    trajectory, Q_last, error, success = test_place(graph, GRIPPER_REFS, rname, inhand, ontarget, None, Q_s,
                                                                    mplan, timeout=timeout)
                    if success: break
                print("{}: {} - {}".format(ID, "SUCCESS" if success else "FAILURE", i))
                if success or fail_count < succ_count * self.S_F_RATIO:
                    self.dict_lock.acquire()
                    acquired = True
                    idx = self.snode_counter.value
                    self.snode_dict[idx] = {
                        "rname1": rname, "obj1": gtem_to_dict(inhand),
                        "obj2": gtem_to_dict(ontarget), "rname2": None, "dims_bak": dims_bak, "color_bak": color_bak,
                        "success": success, "trajectory": trajectory}
                    self.snode_counter.value = self.snode_counter.value + 1
                    self.dict_lock.release()
                    acquired = False
                    if success:
                        succ_count += 1
                    else:
                        fail_count += 1
                    print(
                        "=========== {} {} {} =========== - {}".format(rname, ID, "SUCCESS" if success else "FAILURE", idx))
            except Exception as e:
                if not elog.log(str(e)):
                    break
                if acquired:
                    self.dict_lock.release()
            finally:
                reset_rendering(graph, "PLACE", [], [ontarget, inhand], dims_bak, color_bak, vis=False)
        print("=============== TERMINATE {} ==============".format(ID))

    def handover_search(self, ID, obj_list, Q_s, mplan_dict, L_CELL, timeout=1, N_search=100, N_retry=1):
        graph, GRIPPER_REFS = self.graph, self.GRIPPER_REFS
        fail_count = 0
        succ_count = 0
        elog = Logger()
        acquired = False
        for i in range(N_search):
            try:
                src, tar = random.sample(GRIPPER_REFS.items(), 2)
                mplan = mplan_dict[(src[0], tar[0])]
                src, handed, intar, tar, dims_bak, color_bak = sample_handover(src, tar, L_CELL, mplan.ghnd)
                for _ in range(N_retry):
                    trajectory, Q_last, error, success = test_handover(graph, GRIPPER_REFS, src, handed, intar, tar, Q_s,
                                                                       mplan, timeout=timeout)
                    if success: break
                print("{}: {} - {}".format(ID, "SUCCESS" if success else "FAILURE", i))
                if success or fail_count < succ_count * self.S_F_RATIO:
                    self.dict_lock.acquire()
                    acquired = True
                    idx = self.snode_counter.value
                    self.snode_dict[idx] = {
                        "rname1": src, "obj1": gtem_to_dict(handed),
                        "obj2": gtem_to_dict(intar), "rname2": tar, "dims_bak": dims_bak, "color_bak": color_bak,
                        "success": success, "trajectory": trajectory}
                    self.snode_counter.value = self.snode_counter.value + 1
                    self.dict_lock.release()
                    acquired = False
                    if success:
                        succ_count += 1
                    else:
                        fail_count += 1
                    print(
                        "=========== {}-{} {} {} =========== - {}".format(src, tar, ID, "SUCCESS" if success else "FAILURE",
                                                                          idx))
            except Exception as e:
                if not elog.log(str(e)):
                    break
                if acquired:
                    self.dict_lock.release()
            finally:
                reset_rendering(graph, "HANDOVER", [], [handed, intar], dims_bak, color_bak, vis=False)
        print("=============== TERMINATE {} ==============".format(ID))

    def search_loop_mp(self, Q_s, obj_list, mplan, search_fun, L_CELL, N_agents=None, timeout=1, N_search=100,
                       N_retry=1):
        if N_agents is None:
            N_agents = cpu_count()
        self.N_agents = N_agents
        print("Use {}/{} agents".format(N_agents, cpu_count()))
        self.snode_counter = self.manager.Value('i', 0)
        self.snode_dict = self.manager.dict()
        self.proc_list = [Process(
            target=search_fun,
            args=(id_agent, obj_list, Q_s, mplan, L_CELL),
            kwargs={'timeout': timeout, 'N_search': N_search, 'N_retry': N_retry})
            for id_agent in range(N_agents)]
        for proc in self.proc_list:
            proc.start()

        for proc in self.proc_list:
            proc.join(timeout=60)
        print("================== FINISHED ( {} / {} ) =======================".format(self.snode_counter.value,
                                                                                       N_agents * N_search))
        print(self.snode_counter.value)

    def check_loop_mp(self, test_fun, GRIPPER_REFS, Q_s, dual_mplan_dict, mplan, N_retry=None, timeout=1.0):
        if N_retry is None:
            N_retry = cpu_count()
        self.N_retry = N_retry
        print("Use {}/{} agents to retry {} times".format(N_retry, cpu_count(), N_retry))
        self.proc_list = [Process(
            target=test_fun, args=(self,),
            kwargs={"ID": id_agent, "GRIPPER_REFS": GRIPPER_REFS,
                    "Q_s": Q_s, "dual_mplan_dict": dual_mplan_dict, "mplan": mplan, "timeout": timeout})
            for id_agent in range(N_retry)]
        for proc in self.proc_list:
            proc.start()

        for proc in self.proc_list:
            proc.join(timeout=120)
        print("================== FINISHED =======================")

    def play_all(self, graph, GRIPPER_REFS, key, test_fun, Q_s, period=0.05, remove_map=[[1], [0]]):
        for k in range(self.snode_counter.value):
            rname, inhand, obj, tar, dims_bak, color_bak, succ, trajectory = load_manipulation_from_dict(
                self.snode_dict[k], graph.ghnd)
            show_manip_coords(graph, GRIPPER_REFS, key, rname, inhand, obj, rname2=tar)
            graph.show_motion(trajectory, period=period)
            remove1 = [[inhand, obj][iii] for iii in remove_map[0]]
            remove2 = [[inhand, obj][iii] for iii in remove_map[1]]
            reset_rendering(graph, key, remove1, remove2, dims_bak, color_bak, sleep=True, vis=True)
            print("DONE: {}".format(k))

def get_merge_pairs(ghnd, BASE_LINK):
    merge_pairs = []
    for idx1, ctem1 in zip(range(len(ghnd)), ghnd):
        if ctem1.link_name == BASE_LINK:
            continue
        for ctem2 in ghnd[idx1+1:]:
            if ctem2.link_name == BASE_LINK:
                continue
            if ctem1!=ctem2 and ctem1.link_name == ctem2.link_name and ctem1.gtype==ctem2.gtype and ctem1.collision and ctem2.collision:
                if any(np.subtract(ctem1.rpy, ctem2.rpy)>1e-5):
                    continue # not considering now
                offs = np.subtract(ctem2.center, ctem1.center)    
                dims = np.divide(np.add(ctem1.dims, ctem2.dims),2)
                dims_rot = np.matmul(ctem1.orientation_mat,dims)
                if all(np.abs(offs)<dims_rot):
                    merge_pairs.append((ctem1.name, ctem2.name))
    return merge_pairs

def merge_paired_ctems(ghnd, merge_pairs, VISUALIZE=False, graph=None):
    for mpair in merge_pairs:
        ctem1, ctem2 = ghnd.NAME_DICT[mpair[0]], ghnd.NAME_DICT[mpair[1]]
        offs = np.subtract(ctem2.center, ctem1.center)    
        dims = np.divide(np.add(ctem1.dims, ctem2.dims),2)
        new_center = tuple(np.add(ctem1.center, offs/2))
        new_dims = tuple(dims+np.abs(np.matmul(ctem1.orientation_mat.transpose(), offs)))
        ctem_new = ghnd.create_safe(gtype=GEOTYPE.BOX, name=ctem1.name+"_"+ctem2.name.split("_")[-1],
                                    link_name=ctem1.link_name, dims=new_dims, center=new_center, rpy=ctem1.rpy, 
                                    color=ctem1.color, display=ctem1.display,
                                    collision=ctem1.collision, fixed=ctem1.fixed, soft=ctem1.soft, 
                                    online=ctem1.online, K_col=ctem1.K_col)
        if graph is not None:
            graph.remove_geometry(ctem1)
            graph.remove_geometry(ctem2)
            graph.add_marker(ctem_new, vis=VISUALIZE)
        else:
            ghnd.remove(ctem1)
            ghnd.remove(ctem2)
        
        
import cvxpy


def select_minial_combination(diff_mat):
    selection = cvxpy.Variable(shape=diff_mat.shape, boolean=True)
    line_constraints = [
        cvxpy.sum(sel_line) == 1 for sel_line in selection
    ]
    col_constraints = [
        cvxpy.sum(selection[:, icol]) <= 1 for icol in range(diff_mat.shape[1])
    ]
    total_diff = cvxpy.sum(cvxpy.multiply(diff_mat, selection))
    problem = cvxpy.Problem(cvxpy.Minimize(total_diff),
                            line_constraints + col_constraints)
    problem.solve()
    if problem.status == "optimal":
        return np.round(selection.value).astype(np.bool)
    else:
        raise (RuntimeError("Non-optimal"))




def rearrange_cell_array(cell_array, idxset, L_CELL, Nwdh, ctem_TFs_cur, centers):
    cell_center = cell_array[idxset[0]]
    near_range = np.clip(
        ((cell_center[0]-1,cell_center[0]+1),(cell_center[1]-1,cell_center[1]+1),(cell_center[2]-1,cell_center[2]+1)),
        [[0,0]]*3, np.transpose([Nwdh]*2)-1)
    cells_near = get_centers(tuple(near_range[:,1]-near_range[:,0]+1), 1) - 0.5 + near_range[:, 0]
    center_coord = centers[cell_center[0]][cell_center[1]][cell_center[2]]
    centers_local = (cells_near-cell_center) * L_CELL
    centers_global = centers_local + center_coord
    idx_near = []
    for cell in cells_near.reshape((-1, 3)):
        idx_near += np.where(np.all(cell_array == cell, axis=-1))[0].tolist()
    idx_near = sorted(idx_near)
    diff_mat = np.linalg.norm(ctem_TFs_cur[idx_near][:, :3, 3].reshape((-1, 1, 3)) - centers_global.reshape((1, -1, 3)),
                              axis=-1)
    minimal_combs = select_minial_combination(diff_mat)
    cell_idxes = np.where(minimal_combs)[1]
    cells_new = cells_near.reshape((-1, 3))[cell_idxes].astype(np.int)
    cell_array[idx_near] = cells_new
    return cell_array


def rearrange_cell_array_bak(cell_array, idxset, L_CELL, ctem_TFs_cur, centers):
    cell_center = cell_array[idxset[0]]
    cells_near = get_centers((3, 3, 3), 1) - 1.5 + cell_center
    center_coord = centers[cell_center[0]][cell_center[1]][cell_center[2]]
    centers_local = get_centers((3, 3, 3), L_CELL) - L_CELL * 1.5
    centers_global = centers_local + center_coord
    idx_near = []
    for cell in cells_near.reshape((-1, 3)):
        idx_near += np.where(np.all(cell_array == cell, axis=-1))[0].tolist()
    idx_near = sorted(idx_near)
    diff_mat = np.linalg.norm(ctem_TFs_cur[idx_near][:, :3, 3].reshape((-1, 1, 3)) - centers_global.reshape((1, -1, 3)),
                              axis=-1)
    minimal_combs = select_minial_combination(diff_mat)
    cell_idxes = np.where(minimal_combs)[1]
    cells_new = cells_near.reshape((-1, 3))[cell_idxes].astype(np.int)
    cell_array[idx_near] = cells_new
    return cell_array

def get_cell_data(obj, L_CELL, Nwdh, Tlink_dict, chain_dict, gtype=None, cell=None):
    Tobj = np.matmul(Tlink_dict[obj.link_name], obj.Toff)
    cell, verts_loc = get_cell_verts(gtype or obj.gtype, Tobj, obj.dims, L_CELL, Nwdh, cell=cell)
    return cell, verts_loc, chain_dict[obj.link_name]

def get_cell_verts(gtype, Tobj, dims, L_CELL, Nwdh, cell=None):
    center = Tobj[:3,3]
    cell = np.array(get_cell(center, L_CELL, Nwdh)) if cell is None else cell
    verts_dim = np.multiply(DEFAULT_VERT_DICT[gtype], dims)
    verts = (np.matmul(Tobj[:3,:3], verts_dim.transpose())+Tobj[:3,3:4]).transpose()
    verts_loc = (verts - (cell*L_CELL+L_CELL/2))
    verts_loc = verts_loc.flatten()
    if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
        verts_loc = np.concatenate([verts_loc, dims[0:1]], axis=-1)
    return cell, verts_loc

from ..utils.utils import load_pickle

def load_scene_data(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION, joint_num, get_deviation=False):
    N_vtx_box = 3 * 8
    N_mask_box = 1
    N_joint_box = joint_num
    N_label_box = N_vtx_box + N_mask_box + N_joint_box
    N_vtx_cyl = 3 * 2 + 1
    N_mask_cyl = 1
    N_joint_cyl = joint_num
    N_label_cyl = N_vtx_cyl + N_mask_cyl + N_joint_cyl
    N_vtx_init = 3 * 8
    N_mask_init = 1
    N_joint_init = joint_num
    N_label_init = N_vtx_init + N_mask_init + N_joint_init
    N_vtx_goal = 3 * 8
    N_mask_goal = 1
    N_joint_goal = joint_num
    N_label_goal = N_vtx_goal + N_mask_goal + N_joint_goal
    N_joint_label = 6 * joint_num
    N_cell_label = N_label_box + N_label_cyl + N_label_init + N_label_goal + N_joint_label
    N_BEGIN_CYL = N_vtx_box + N_mask_box + N_joint_box
    N_BEGIN_INIT = N_BEGIN_CYL + N_vtx_cyl + N_mask_cyl + N_joint_cyl
    N_BEGIN_GOAL = N_BEGIN_INIT + N_vtx_init + N_mask_init + N_joint_init

    print("load: {}".format((CONVERTED_PATH, DATASET, WORLD, SCENE)))
    scene_pickle = load_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, "scene.pkl"))
    scene_data = scene_pickle[b'scene_data']
    ctem_names = scene_pickle[b'ctem_names']
    ctem_cells = scene_pickle[b'ctem_cells']

    act_dat = load_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION))
    init_box_dat = act_dat[b'init_box_dat']
    goal_box_dat = act_dat[b'goal_box_dat']
    ctem_dat_list = act_dat[b'ctem_dat_list']
    skey = int(act_dat[b'skey'])
    success = act_dat[b'success']
    ### put init, goal item data
    cell, verts, chain = init_box_dat
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_INIT:N_BEGIN_INIT + N_vtx_init] = verts
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_INIT + N_vtx_init:N_BEGIN_INIT + N_vtx_init + N_mask_init] = 1
    scene_data[cell[0], cell[1], cell[2],
    N_BEGIN_INIT + N_vtx_init + N_mask_init:N_BEGIN_INIT + N_vtx_init + N_mask_init + N_joint_init] = chain
    cell_init = cell

    cell, verts, chain = goal_box_dat
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_GOAL:N_BEGIN_GOAL + N_vtx_goal] = verts
    scene_data[cell[0], cell[1], cell[2], N_BEGIN_GOAL + N_vtx_goal:N_BEGIN_GOAL + N_vtx_goal + N_mask_goal] = 1
    scene_data[cell[0], cell[1], cell[2],
    N_BEGIN_GOAL + N_vtx_goal + N_mask_goal:N_BEGIN_GOAL + N_vtx_goal + N_mask_goal + N_joint_goal] = chain
    cell_goal = cell

    ### add/replace collilsion object
    for cname, ctype, cell, verts, chain in ctem_dat_list:
        if ctype == b'BOX':
            N_BEGIN_REP, N_vtx, N_mask, N_joint = 0, N_vtx_box, N_mask_box, N_joint_box
        elif ctype == b'CYLINDER':
            N_BEGIN_REP, N_vtx, N_mask, N_joint = N_BEGIN_CYL, N_vtx_cyl, N_mask_cyl, N_joint_cyl
        else:
            raise (RuntimeError("Non considered shape key"))
        scene_data[cell[0], cell[1], cell[2], N_BEGIN_REP:N_BEGIN_REP + N_vtx] = verts
        scene_data[cell[0], cell[1], cell[2], N_BEGIN_REP + N_vtx:N_BEGIN_REP + N_vtx + N_mask] = 1
        scene_data[cell[0], cell[1], cell[2],
        N_BEGIN_REP + N_vtx + N_mask:N_BEGIN_REP + N_vtx + N_mask + N_joint] = chain
    if get_deviation:
        return scene_data, success, skey, cell_init, cell_goal
    else:
        return scene_data, success, skey

def get_box_diplay(ghnd, name, cell_dat, N_BEGIN, joint_num, center, color=(0.8,0.0,0.0,0.5), dim_offset=(0,0,0)):
    N_vtx_box = 3*8
    verts = cell_dat[N_BEGIN:N_BEGIN+N_vtx_box]
    mask = bool(cell_dat[N_BEGIN+N_vtx_box])
    chain = cell_dat[N_BEGIN+N_vtx_box+1:N_BEGIN+N_vtx_box+1+joint_num]
    if mask:
        verts_res = verts.reshape((-1,3))
        center_loc = np.mean(verts_res, axis=0)
        dim_x = float(np.linalg.norm(verts_res[4,:]-verts_res[0,:]))
        dim_y = float(np.linalg.norm(verts_res[2,:]-verts_res[0,:]))
        dim_z = float(np.linalg.norm(verts_res[1,:]-verts_res[0,:]))
        dims = (dim_x, dim_y, dim_z)
        verts_ref = BOX_DEFAULT*dims
        verts_ctd = verts_res-center_loc
        R = np.matmul(np.linalg.pinv(verts_ref), verts_ctd).transpose()
        rpy = tuple(Rot2rpy(R))
        center = tuple(np.add(center, center_loc))
        box = ghnd.create_safe(name=name, gtype=GEOTYPE.BOX, link_name="base_link",
                               dims=tuple(np.add(dims, dim_offset)), center=center, rpy=rpy,
                               collision=False, display=True, color=color)
    else:
        box = None
    return box, mask, chain

from ..utils.rotation_utils import calc_zvec_R

def get_cyl_diplay(ghnd, name, cell_dat, N_BEGIN, joint_num, center, color=(0.8,0.0,0.0,0.5), dim_offset=(0,0,0)):
    N_vtx_cyl = 3*2+1
    verts = cell_dat[N_BEGIN:N_BEGIN+N_vtx_cyl]
    mask = bool(cell_dat[N_BEGIN+N_vtx_cyl])
    chain = cell_dat[N_BEGIN+N_vtx_cyl+1:N_BEGIN+N_vtx_cyl+1+joint_num]
    if mask:
        verts_res = verts[:-1].reshape((-1,3))
        center_loc = np.mean(verts_res, axis=0)
        dim_z = float(np.linalg.norm(verts_res[1,:]-verts_res[0,:]))
        dims = (verts[-1], verts[-1], dim_z)
        verts_ref = SEG_DEFAULT*dims
        verts_ctd = verts_res-center_loc
        vec=(verts_ctd[1] -verts_ctd[0])/dims[2]
        rpy = Rot2rpy(calc_zvec_R(vec))
        center = tuple(np.add(center, center_loc))
        cyl = ghnd.create_safe(name=name, gtype=GEOTYPE.CYLINDER, link_name="base_link",
                               dims=tuple(np.add(dims, dim_offset)), center=center, rpy=rpy,
                               collision=False, display=True, color=color)
    else:
        cyl = None
    return cyl, mask, chain

def get_twist_tems(ghnd, cell_dat, center, chain, idx_chain, joint_num, L_CELL, load_limits=True):
    i_j = np.where(chain)[0][idx_chain]
    if load_limits:
        N_joint_limits = 3 * joint_num
        N_joint_label = 6 * joint_num + N_joint_limits
        xi = cell_dat[-N_joint_label:-N_joint_limits].reshape((-1,6))
    else:
        N_joint_label = 6 * joint_num
        xi = cell_dat[-N_joint_label:].reshape((-1,6))
    print("xi: {}".format(xi.shape))
    wv = xi[i_j]
    print("wv: {}".format(wv))
    __w = wv[:3]
    __v = wv[3:]
    w_abs = np.linalg.norm(__w)
    v_abs = np.linalg.norm(__v)
    q_abs = v_abs/w_abs
    w_nm = __w/w_abs
    v_nm = __v/v_abs
    q_nm = np.cross(v_nm, w_nm)
    __q = q_nm*q_abs
    joint_point = center - __q
    rpy = Rot2rpy(np.matmul(calc_zvec_R(q_nm), Rot_axis(2,np.pi/2)))
    dims = (0.01,3,3)
    ptem = ghnd.create_safe(name="joint_plane_{}".format(i_j), gtype=GEOTYPE.BOX, link_name="base_link", center = joint_point, rpy=rpy, dims=dims, collision=False, display=True, color=(0.5,0.5,0.5,0.5))
    atem = ghnd.create_safe(name="joint_dir_{}".format(i_j), gtype=GEOTYPE.ARROW, link_name="base_link",
                            center = center, rpy=rpy, dims=(q_abs,0.01,0.01,),
                            collision=False, display=True, color=(1,0,0,1))
    rpy_v = Rot2rpy(np.matmul(calc_zvec_R(v_nm), Rot_axis(2,np.pi/2)))
    vtem = ghnd.create_safe(name="joint_vel_{}".format(i_j), gtype=GEOTYPE.ARROW, link_name="base_link",
                            center = center, rpy=rpy_v, dims=(v_abs,0.01,0.01,),
                            collision=False, display=True, color=(0,0,1,1))
    btem = ghnd.create_safe(name="cell_{}".format(i_j), gtype=GEOTYPE.BOX, link_name="base_link",
                            center = center, rpy=(0,0,0), dims=(L_CELL,L_CELL,L_CELL,),
                            collision=False, display=True, color=(0.7,0.7,0.6,0.2))
    return ptem, vtem, atem, btem


def select_minial_combination_fast(diff_mat):
    diff_shape = diff_mat.shape
    if diff_shape[0] > diff_shape[1]:
        raise("Items more than cells")
    selection = np.zeros((diff_shape[0],), dtype=np.int)
    for _ in range(len(selection)):
        min_loc = np.unravel_index(np.argmin(diff_mat), diff_shape)
        selection[min_loc[0]] = min_loc[1]
        diff_mat[min_loc[0],:] = 1e5
        diff_mat[:,min_loc[1]] = 1e5
    return selection

def rearrange_cell_array_fast(cell_array, idxset, L_CELL, Nwdh, ctem_TFs_cur, centers):
    cell_center = cell_array[idxset[0]]
    near_range = np.clip(
        ((cell_center[0]-1,cell_center[0]+1),(cell_center[1]-1,cell_center[1]+1),(cell_center[2]-1,cell_center[2]+1)),
        [[0,0]]*3, np.transpose([Nwdh]*2)-1)
    cells_near = get_centers(tuple(near_range[:,1]-near_range[:,0]+1), 1) - 0.5 + near_range[:, 0]
    center_coord = centers[cell_center[0]][cell_center[1]][cell_center[2]]
    centers_local = (cells_near-cell_center) * L_CELL
    centers_global = centers_local + center_coord
    idx_near = []
    for cell in cells_near.reshape((-1, 3)):
        idx_near += np.where(np.all(cell_array == cell, axis=-1))[0].tolist()
    idx_near = sorted(idx_near)
    diff_mat = np.linalg.norm(ctem_TFs_cur[idx_near][:, :3, 3].reshape((-1, 1, 3)) - centers_global.reshape((1, -1, 3)),
                              axis=-1)
    cell_idxes = select_minial_combination_fast(diff_mat)
    cells_new = cells_near.reshape((-1, 3))[cell_idxes].astype(np.int)
    cell_array[idx_near] = cells_new
    return cell_array
