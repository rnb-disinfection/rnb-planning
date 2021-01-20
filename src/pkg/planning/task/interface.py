from abc import *
import numpy as np
from ...utils.utils import differentiate, GlobalTimer
from ...constants import DIR_VEC_DICT
from collections import defaultdict

__metaclass__ = type


class SearchNode:
    def __init__(self, idx, state, parents, leafs, leafs_P, depth=None, edepth=None,
                 redundancy=None):
        self.idx, self.state, self.parents, self.leafs, self.leafs_P, self.depth, self.edepth, self.redundancy = \
            idx, state, parents, leafs, leafs_P, depth, edepth, redundancy
        self.traj = None
        self.traj_size = 0
        self.traj_length = 0

    def set_traj(self, traj_full):
        self.traj_size = len(traj_full)
        self.traj_length = np.sum(np.abs(differentiate(traj_full, 1)[:-1])) if self.traj_size > 1 else 0
        self.traj = np.array(traj_full)

    def get_traj(self):
        return self.traj

    def copy(self, graph):
        return SearchNode(self.idx, State(self.state.node, self.state.obj_pos_dict, self.state.Q, graph),
                          self.parents, self.leafs, self.leafs_P, self.depth, self.edepth, self.redundancy)

class State:
    def __init__(self, node, obj_pos_dict, Q, graph):
        self.obj_pos_dict = obj_pos_dict
        self.Q = Q
        self.set_node(node, graph)

    def set_node(self, node, graph):
        self.node = node
        self.onode = node2onode(graph, self.node)

    def get_tuple(self):
        return (self.node, self.obj_pos_dict, self.Q)

    def copy(self, graph):
        return State(self.node, self.obj_pos_dict, self.Q, graph)

    def __str__(self):
        return str((self.node,
                    {k: str(np.round(v, 2)) for k, v in
                     self.obj_pos_dict.items()} if self.obj_pos_dict is not None else None,
                    str(np.round(self.Q, 2)) if self.Q is not None else None))

def node2onode(graph, node):
    return tuple([graph.binder_dict[binding[2]].geometry.name for binding in node])

class SamplerInterface:
    NAME = None

    def __init__(self, graph):
        self.graph =graph
        self.gtimer = GlobalTimer.instance()

    @abstractmethod
    def build_graph(self, update_handles=True):
        pass

    @abstractmethod
    def search_graph(self, initial_state, goal_nodes, *args, **kwargs):
        pass

    def check_goal(self, node, goals):
        return node in goals

    def check_goal_by_score(self, node, goal_cost_dict):
        return goal_cost_dict[node] == 0


    def print_snode_list(self):
        for i_s, snode in sorted(self.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    def idxSchedule2SnodeScedule(self, schedule, ZERO_JOINT_POSE):
        snode_schedule = [self.snode_dict[i_sc] for i_sc in schedule]
        snode_schedule.append(snode_schedule[-1].copy(self.graph))
        snode_schedule[-1].state.Q = ZERO_JOINT_POSE
        snode_schedule[-1].set_traj(np.array([ZERO_JOINT_POSE]))
        return snode_schedule

    def find_best_schedule(self, schedule_sorted):
        best_snode_schedule = None
        best_score = 1e10
        for ss in schedule_sorted:
            schedule = ss
            snode_schedule_list = self.idxSchedule2SnodeScedule(schedule, self.graph.combined_robot.home_pose)
            score = np.sum([snode.traj_length for snode in snode_schedule_list])
            if score < best_score:
                best_score = score
                best_snode_schedule = snode_schedule_list
        return best_snode_schedule

def get_goal_nodes(initial_node, obj_name, target_name, postfix="_p"):
    return [tuple([(opair[0], ppoint, target_name) \
                   if opair[0] == obj_name \
                   else opair \
                   for opair in initial_node]) \
            for ppoint in [dvkey+postfix
                           for dvkey in DIR_VEC_DICT.keys()]]
