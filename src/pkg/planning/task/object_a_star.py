from .interface import *
from ...utils.utils import *
from ...utils.joint_utils import *
from ..scene import State
from collections import defaultdict
import random

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue


##
# @class    ObjectAstar
# @brief    object level A* algorithm
class ObjectAstar(TaskInterface):
    DEFAULT_TRANSIT_COST = 1.0
    DQ_MAX = np.deg2rad(45)
    WEIGHT_DEFAULT = 2.0
    DSCALE = 1e4

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene):
        TaskInterface.__init__(self, pscene)

    ##
    # @brief build object-level node graph
    def prepare(self):
        pscene = self.pscene

        # make all node connections
        self.node_list = pscene.get_all_nodes()
        self.node_dict = {k: [] for k in self.node_list}
        self.node_parent_dict = {k: [] for k in self.node_list}
        for node in self.node_list:
            for leaf in pscene.get_node_neighbor(node):
                self.node_dict[node].append(leaf)
                self.node_parent_dict[leaf].append(node)
        for node in self.node_list:
            self.node_dict[node] = list(set(self.node_dict[node]))
            self.node_parent_dict[node] = list(set(self.node_parent_dict[node]))

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, tree_margin=None, depth_margin=None):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.init_cost_dict, self.goal_cost_dict = \
            self.score_graph(initial_state.node), self.score_graph(goal_nodes, reverse=True)

        # set default margins
        tree_margin = tree_margin or self.goal_cost_dict[initial_state.node]
        depth_margin = depth_margin or self.goal_cost_dict[initial_state.node]

        self.reset_valid_node(tree_margin)
        self.depth_min = self.goal_cost_dict[initial_state.node]
        self.max_depth = self.depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()

    ##
    # @brief get sample leafs from snode
    # @param snode A validated SearchNode of which leafs should be added to queue
    # @param N_redundant_sample number of redundant samples
    # @return snode_tuple_list list of tuple(priority, (snode, from_state, to_state, redundancy))
    def get_leafs(self, snode, N_redundant_sample):
        queue = []
        state = snode.state
        leafs = self.valid_node_dict[state.node]
        Q_dict = list2dict(state.Q, self.pscene.gscene.joint_names)
        for leaf in leafs:
            depth = len(snode.parents) + 1
            expected_depth = depth + self.goal_cost_dict[leaf]
            if expected_depth > self.max_depth:
                continue
            available_binding_dict = self.pscene.get_available_binding_dict(state, leaf, Q_dict)
            if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                print("============== Non-available transition: Break =====================")
                break
            for _ in range(N_redundant_sample):
                to_state, redundancy_dict = self.pscene.sample_leaf_state(state, available_binding_dict, leaf)
                priority = (expected_depth - depth) * self.DSCALE + depth ## greedy
                queue.append((priority, (snode, state, to_state, redundancy_dict)))
        return queue

    ##
    # @brief get optimal remaining steps
    def get_optimal_remaining_steps(self, state):
        return self.goal_cost_dict[state.node]

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes

    ##
    # @brief filter out node links that cause depth to exceed the margin
    def reset_valid_node(self, margin=0, node=None):
        if node == None:
            node = self.initial_state.node
            self.valid_node_dict = {goal:[] for goal in self.goal_nodes}
        if node in self.valid_node_dict or self.check_goal_by_score(node):
            return
        neighbor = self.get_valid_neighbor(node, margin=margin)
        if node in self.valid_node_dict and self.valid_node_dict[node] == neighbor:
            return
        self.valid_node_dict[node] = neighbor
        for leaf in neighbor:
            new_margin = margin - max(0, self.goal_cost_dict[leaf]-self.goal_cost_dict[node])
            new_margin = max(0, new_margin)
            if leaf != node and new_margin>=0:
                self.reset_valid_node(margin=new_margin, node=leaf)

    def score_graph(self, reference_node, reverse=False):
        node_dict = self.node_parent_dict if reverse else self.node_dict
        came_from = {}
        node_cost_dict = {}
        frontier = PriorityQueue()
        if isinstance(reference_node, list):
            for goal in reference_node:
                frontier.put((0, goal))
                came_from[goal] = None
                node_cost_dict[goal] = 0
        else:
            frontier.put((0, reference_node))
            came_from[reference_node] = None
            node_cost_dict[reference_node] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            for next in node_dict[current]:
                if next == current:
                    continue
                new_cost = node_cost_dict[current] + self.DEFAULT_TRANSIT_COST
                if next not in node_cost_dict or new_cost < node_cost_dict[next]:
                    node_cost_dict[next] = new_cost
                    priority = new_cost
                    frontier.put((priority, next))
                    came_from[next] = current
        return node_cost_dict

    def get_valid_neighbor(self, node, margin=0):
        neighbor = self.node_dict[node]
        neighbor_valid = []
        for leaf in neighbor:
            if self.goal_cost_dict[leaf]<self.goal_cost_dict[node]+margin:
                neighbor_valid += [leaf]
        return neighbor_valid

    def check_goal_by_score(self, node):
        return self.goal_cost_dict[node] == 0