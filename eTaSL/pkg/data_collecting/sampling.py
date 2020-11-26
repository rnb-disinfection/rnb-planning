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
