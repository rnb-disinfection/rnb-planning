import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))

from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
from pkg.utils.nonholo_utils import *

class NonHolonomicPlanner:
    def __init__(self, mplan, mobile_name, min_radi=MIN_RADI_DEFAULT):
        self.mplan = mplan
        self.pscene = mplan.pscene
        self.crob = self.pscene.combined_robot
        self.gscene = self.pscene.gscene
        self.mobile_name = mobile_name
        self.idx_mobile = self.crob.idx_dict[mobile_name]
        self.min_radi = min_radi

    def extend_tree(self, tree_cur, Qref, Xnew, update_gscene=False,
                    connect_dist=1.0, step_size=STEP_SIZE_DEFAULT):
        Qnew = Qref.copy()
        Qnew[self.idx_mobile[:2]] = Xnew[:2]
        Qnew[self.idx_mobile[2]] = (Xnew[2] + np.pi) % (2 * np.pi) - np.pi
        nodes = tree_cur["nodes"]
        edges = tree_cur["edges"]
        connect_N = int(connect_dist / step_size)
        if not self.mplan.validate_trajectory([Qnew], update_gscene=update_gscene):  # validate
            return None
        Xnear = sorted(nodes, key=lambda x: dist_nonholo(Xnew, x, min_radi=self.min_radi))[0]
        y12, y21, theta, R, T1, T2 = get_nonholo_trajargs(Xnear, Xnew)
        T_list = interpolate_nonholo(y12, y21, theta, R, T1, T2, min_radi=self.min_radi, step_size=step_size)
        if not T_list:
            return None
        traj = []
        for T in T_list[:connect_N]:
            Q_ = Qref.copy()
            Q_[self.idx_mobile[:2]] = T[:2, 2]
            Q_[self.idx_mobile[2]] = np.arctan2(T[1, 0], T[0, 0])
            traj.append(Q_)
        traj = np.array(traj)
        if self.mplan.validate_trajectory(traj, update_gscene=update_gscene):
            nodes.append(tuple(traj[-1, :3].tolist()))
            i_n = len(nodes) - 1
            edge = (nodes.index(Xnear), i_n, traj)
            edges.append(edge)
            return edge, len(T_list) == len(traj)

    def backtrack_tree(self, tree, i_node, invert=False):
        edges = tree["edges"]
        edge_list = []
        while i_node > 0:
            for edge in edges:
                if edge[1] == i_node:
                    if invert:
                        edge_list.append((edge[1], edge[0],
                                          np.array(list(reversed(edge[2])))))
                    else:
                        edge_list = [edge] + edge_list
                    i_node = edge[0]
        return edge_list

    def search(self, Qinit, Xgoal, timeout=1, step_size=STEP_SIZE_DEFAULT, connect_dist=1, update_gscene=True):
        if update_gscene:
            self.mplan.update_gscene()

        Qinit = np.array(Qinit)
        Xinit = tuple(Qinit[self.idx_mobile[:3]])
        t_s = time.time()
        mins = self.mplan.combined_robot.get_joint_limits()[:3, 0]
        maxs = self.mplan.combined_robot.get_joint_limits()[:3, 1]
        tree_init = {"name": "init", "nodes": [Xinit], "edges": []}
        tree_goal = {"name": "goal", "nodes": [Xgoal], "edges": []}

        tree_cur, tree_tar = tree_init, tree_goal
        fin = False
        while time.time() - t_s < timeout:
            Xnew = np.random.uniform(mins, maxs)
            res = self.extend_tree(tree_cur, Qinit, Xnew, update_gscene=False,
                                   connect_dist=connect_dist, step_size=step_size)
            if res:
                edge = res[0]
                res = self.extend_tree(tree_tar, Qinit, tree_cur["nodes"][edge[1]], update_gscene=False,
                                       connect_dist=connect_dist, step_size=step_size)
                if res:
                    fin = res[1]
                    if fin:
                        link = {tree_cur["name"]: edge[1], tree_tar["name"]: res[0][1]}
                        edges_init = self.backtrack_tree(tree_init, link["init"])
                        edges_goal = self.backtrack_tree(tree_goal, link["goal"], invert=True)
                        edge_list = edges_init + edges_goal
                        return np.concatenate([edge[2] for edge in edge_list])
            tree_cur, tree_tar = tree_tar, tree_cur