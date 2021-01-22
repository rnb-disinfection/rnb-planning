from .interface import *
from ...utils.utils import *
from ...utils.joint_utils import *
from ..constraint.constraint_common import combine_redundancy, sample_redundancy
from ..scene import node2onode
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

        oname_list = pscene.object_list
        boname_list = pscene.object_binder_dict.keys()
        obj_binding_dict = get_available_binder_dict(pscene, oname_list,
                                                     boname_list)  # matching possible binder dictionary
        binder_combinations = list(
            product(*[obj_binding_dict[oname] for oname in oname_list]))  # all possible binding combination list
        uniq_binders = pscene.get_unique_binders()  # binders cannot be shared by multiple objects
        uniq_bo_list = [boname for boname in boname_list if
                        (len(pscene.object_binder_dict[boname]) == 1 and all(
                            [bname in uniq_binders for bname in pscene.object_binder_dict[boname]]))]
        ctrl_binders = pscene.get_controlled_binders()  # all controllable binders
        ctrl_bo_list = [boname for boname in boname_list if
                        all([bname in ctrl_binders for bname in pscene.object_binder_dict[boname]])]

        # filter out conflicting use of uniq binding object
        usage_conflicts = []
        self_binding = []
        for bc in binder_combinations:
            for ubo in uniq_bo_list:
                if sum([ubo == bo for bo in bc]) > 1:
                    usage_conflicts.append(bc)
                    break
            if any([oname == boname for oname, boname in zip(pscene.object_list, bc)]):
                self_binding.append(bc)

        all_conflicts = list(set(self_binding + usage_conflicts))
        for conflict in all_conflicts:
            binder_combinations.remove(conflict)

        # make all node connections
        self.node_list = binder_combinations
        self.node_dict = {k: list() for k in self.node_list}
        for node in self.node_list:
            fixed_idx = [i_n for i_n in range(len(node)) if node[i_n] not in ctrl_bo_list]  # fixed binding index
            ctrl_idx = [i_n for i_n in range(len(node)) if node[i_n] in ctrl_bo_list]  # control binding index
            for leaf in self.node_list:
                if (all([node[fi] == leaf[fi] for fi in fixed_idx])  # fixed object is not changed
                        and all([node[ci] == leaf[ci] or (node[ci] not in leaf) for ci in
                                 ctrl_idx])  # controlled binder is empty or not changed
                        and sum([node[ci] != leaf[ci] for ci in ctrl_idx])==1  # at list one controlled binder is changed
                ):
                    self.node_dict[node].append(leaf)
                    self.node_dict[leaf].append(node)
        for node in self.node_list:
            self.node_dict[node] = list(set(self.node_dict[node]))

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal, tree_margin=None, depth_margin=None):

        goal_nodes = list(set([node2onode(self.pscene, gnode) for gnode in goal]))
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.onode), self.score_graph(goal_nodes)

        # set default margins
        tree_margin = tree_margin or self.goal_cost_dict[initial_state.onode]
        depth_margin = depth_margin or self.goal_cost_dict[initial_state.onode]

        self.reset_valid_node(tree_margin)
        self.depth_min = self.goal_cost_dict[initial_state.onode]
        self.max_depth = self.depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()

    ##
    # @brief get sample leafs from snode
    # @param snode A validated SearchNode of which leafs should be added to queue
    # @param N_redundant_sample number of redundant samples
    # @return snode_tuple_list list of tuple(priority, (snode, from_state, to_state, redundancy))
    @abstractmethod
    def get_leafs(self, snode, N_redundant_sample):
        queue = []
        state = snode.state
        leafs = self.valid_node_dict[state.onode]
        Q_dict = list2dict(state.Q, self.pscene.gscene.joint_names)
        for leaf in leafs:
            depth = len(snode.parents) + 1
            expected_depth = depth + self.goal_cost_dict[leaf]
            if expected_depth > self.max_depth:
                continue
            available_binding_dict = self.get_available_binding_dict(state, leaf, Q_dict)
            if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                print("============== Non-available transition: Break =====================")
                break
            for _ in range(N_redundant_sample):
                to_state, redundancy_dict = self.sample_leaf_state(state, available_binding_dict, leaf)
                priority = (expected_depth - depth) * self.DSCALE + depth ## greedy
                queue.append((priority, (snode, state, to_state, redundancy_dict)))
        return queue

    ##
    # @brief get optimal remaining steps
    def get_optimal_remaining_steps(self, state):
        return self.goal_cost_dict[state.onode]

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    @abstractmethod
    def check_goal(self, state):
        return state.onode in self.goal_nodes

    def score_graph(self, goal_node):
        came_from = {}
        node_cost_dict = {}
        frontier = PriorityQueue()
        if isinstance(goal_node, list):
            for goal in goal_node:
                frontier.put((0, goal))
                came_from[goal] = None
                node_cost_dict[goal] = 0
        else:
            frontier.put((0, goal_node))
            came_from[goal_node] = None
            node_cost_dict[goal_node] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            for next in self.node_dict[current]:
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

    def reset_valid_node(self, margin=0, node=None):
        if node == None:
            node = self.initial_state.onode
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

    def get_available_binding_dict(self, state, to_onode, Q_dict=None, pscene=None):
        if pscene is None:
            pscene = self.pscene
        if Q_dict is None:
            Q_dict = list2dict(state.Q, pscene.gscene.joint_names)
        return {oname:get_available_bindings(pscene, oname, boname, sbinding[1], sbinding[2],
                                                                 Q_dict=Q_dict)\
                                              if sboname!=boname else [sbinding[1:]]
                                      for oname, boname, sboname, sbinding
                                      in zip(pscene.object_list, to_onode, state.onode, state.node)}

    def sample_leaf_state(self, state, available_binding_dict, to_onode, pscene=None):
        if pscene is None:
            pscene = self.pscene
        to_state = state.copy(pscene)
        to_node = tuple([(((oname,)+\
                           random.choice(available_binding_dict[oname]))
                          if sboname!=boname else sbinding)
                         for oname, boname, sboname, sbinding
                         in zip(pscene.object_list, to_onode, state.onode, state.node)])
        to_state.set_node(to_node, pscene)
        redundancy_dict = {}
        for from_binding, to_binding in zip(state.node, to_node):
            obj = pscene.object_dict[from_binding[0]]
            to_ap = obj.action_points_dict[to_binding[1]]
            to_binder = pscene.binder_dict[to_binding[2]]
            redundancy_tot = combine_redundancy(to_ap, to_binder)
            redundancy = sample_redundancy(redundancy_tot)
            redundancy_dict[from_binding[0]] = redundancy
        return to_state, redundancy_dict

    def quiver_snodes(self, figsize=(10,10)):
        import matplotlib.pyplot as plt
        N_plot = self.snode_counter.value
        snode_vec = [v for k,v in sorted(self.snode_dict.items(), key=lambda x: x)]
        cost_vec = [self.goal_cost_dict[snode.state.onode] for snode in snode_vec[1:N_plot]]
        parent_vec = [self.goal_cost_dict[self.snode_dict[snode.parents[-1]].state.onode] for snode in snode_vec[1:N_plot]]
        plt.figure(figsize=figsize)
        X = list(range(1,N_plot))
        plt.quiver(X, parent_vec,
                   [0]*(N_plot-1),
                   np.subtract(cost_vec, parent_vec),
                   angles='xy', scale_units='xy', scale=1)
        plt.plot(X, cost_vec,'.')
        plt.plot(X, parent_vec,'.')
        plt.axis([0,N_plot+1,-0.5,4.5])


def get_available_binder_dict(pscene, oname_list, bname_list):
    available_binder_dict = defaultdict(list)
    for bname in bname_list:
        for oname in oname_list:
            pass_now = False
            for binder_name in pscene.object_binder_dict[bname]:
                binder = pscene.binder_dict[binder_name]
                for ap in pscene.object_dict[oname].action_points_dict.values():
                    if binder.check_type(ap):
                        available_binder_dict[oname].append(binder.geometry.name)
                        pass_now = True
                        break
                if pass_now:
                    break
    return available_binder_dict


def get_available_bindings(pscene, oname, boname, ap_exclude, bd_exclude, Q_dict):
    obj = pscene.object_dict[oname]
    ap_dict = obj.action_points_dict
    apk_list = ap_dict.keys()
    bd_list = [pscene.binder_dict[bname] for bname in pscene.object_binder_dict[boname]
               if pscene.binder_dict[bname].check_available(Q_dict)]

    apk_exclude = obj.get_conflicting_handles(ap_exclude)
    ap_list = [ap_dict[apk] for apk in apk_list if apk not in apk_exclude]
    bd_exclude = pscene.binder_dict[bd_exclude]
    if bd_exclude in bd_list:
        bd_list.remove(pscene.binder_dict[bd_exclude])

    available_bindings = []
    for bd in bd_list:
        for ap in ap_list:
            if bd.check_type(ap):
                available_bindings.append((ap.name, bd.name))
    if not available_bindings:
        print("=================================")
        print("=================================")
        print("=================================")
        print("Not available:{}-{}".format(oname,boname))
        print("np_exclude:{}".format(ap_exclude))
        print("bd_exclude:{}".format(bd_exclude.name))
        print("=================================")
        print("=================================")
        print("=================================")
    return available_bindings