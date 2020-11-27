import numpy as np
import time
from ..geometry.geometry import GEOTYPE

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
from ..geometry.geometry import DEFAULT_VERT_DICT, GEOTYPE
from ..utils.rotation_utils import Rot_rpy
from ..utils.utils import list2dict
import numpy as np

POINT_DEFAULT = np.array([[0,0,0]])
SEG_DEFAULT = np.array([[0,0,1.0],[0,0,-1.0]])/2
BOX_DEFAULT = np.array([[[(i,j,k) for k in range(2)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5

DEFAULT_VERT_DICT = {
    GEOTYPE.SPHERE: POINT_DEFAULT,
    GEOTYPE.SEGMENT: SEG_DEFAULT,
    GEOTYPE.BOX: BOX_DEFAULT,
    GEOTYPE.MESH: BOX_DEFAULT
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

def get_collision_mat(Nwdh, L_CELL, crob, ghnd, Q_s):
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
    links = []
    for rname in crob.robot_names:
        links += [gtem for gtem in ghnd if rname in gtem.name and gtem.collision]

    link_verts = []
    link_rads = []
    for link in links:
        T_link = link.get_tf(list2dict(Q_s, crob.joint_names))
        link_verts.append(get_vertex_rows_T(link.gtype, T_link, link.dims))
        if link.gtype in [GEOTYPE.SEGMENT, GEOTYPE.SPHERE]:
            link_rads.append(link.dims[0])
        else:
            link_rads.append(0)
    link_verts = np.array(link_verts)
    coll_mat=np.zeros((Nw, Nd, Nh, len(link_verts)))

    link_ctems = []
    for vert in link_verts:
        link_ctems.append(getPointList(vert))

    for iw in range(Nw):
        for id in range(Nd):
            for ih in range(Nh):
                cell = getPointList(cell_mat[iw][id][ih])
                for il, ctem in zip(range(len(link_ctems)), link_ctems):
                    coll_mat[iw, id, ih, il] = oGJK.gjk_cpp(cell, ctem)-link_rads[il]
    return coll_mat
