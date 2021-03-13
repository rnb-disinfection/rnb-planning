from .interface import *
from ...utils.utils import *
from ...utils.joint_utils import *
from ..scene import State
from collections import defaultdict
from copy import deepcopy
import random

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue
from Queue import Queue

##
# @class    TaskRRT
# @brief    task level RRT algorithm
class TaskRRT(TaskInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene,
                 new_node_sampler=random.choice, parent_node_sampler=random.choice, parent_snode_sampler=random.choice,
                 binding_sampler=random.choice, redundancy_sampler=random.uniform, custom_rule=None):
        TaskInterface.__init__(self, pscene)
        self.new_node_sampler = new_node_sampler
        self.parent_node_sampler = parent_node_sampler
        self.parent_snode_sampler = parent_snode_sampler
        self.binding_sampler = binding_sampler
        self.redundancy_sampler = redundancy_sampler
        self.custom_rule = custom_rule

    ##
    # @brief build object-level node graph
    def prepare(self):
        pscene = self.pscene

        # make all node connections
        self.node_list = pscene.get_all_nodes()
        self.node_dict_full = {k: [] for k in self.node_list}
        self.node_parent_dict_full = {k: [] for k in self.node_list}
        for node in self.node_list:
            for leaf in pscene.get_node_neighbor(node):
                if leaf in self.node_list:
                    self.node_dict_full[node].append(leaf)
                    self.node_parent_dict_full[leaf].append(node)
        for node in self.node_list:
            self.node_dict_full[node] = set(self.node_dict_full[node])
            self.node_parent_dict_full[node] = set(self.node_parent_dict_full[node])

        self.unstoppable_subjects = [i_s for i_s, sname in enumerate(self.pscene.subject_name_list)
                                     if self.pscene.subject_dict[sname].unstoppable]

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.neighbor_nodes = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.node_snode_dict = multiprocess_manager.dict()
            self.snode_dict_lock = multiprocess_manager.Lock()
            self.attempts_reseved = multiprocess_manager.Queue()
            self.reserve_lock = multiprocess_manager.Lock()
        else:
            self.neighbor_nodes = dict()
            self.node_snode_dict = dict()
            self.snode_dict_lock = DummyBlock()
            self.attempts_reseved = Queue()
            self.reserve_lock = DummyBlock()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, tree_margin=None, depth_margin=None):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.reserved_attempt = False

        self.unstoppable_terminals = {}
        for sub_i in self.unstoppable_subjects:
            self.unstoppable_terminals[sub_i] = [self.initial_state.node[sub_i]]
            for goal in goal_nodes:
                self.unstoppable_terminals[sub_i].append(goal[sub_i])

        self.node_dict = {}
        for node, leafs in self.node_dict_full.items():
            self.node_dict[node] = set(
                [leaf for leaf in leafs ## unstoppable node should change or at terminal
                 if all([node[k] in terms or node[k]!=leaf[k]
                         for k, terms in self.unstoppable_terminals.items()])])

        self.node_parent_dict = {}
        for node, parents in self.node_parent_dict_full.items():
            self.node_parent_dict[node] = set(
                [parent for parent in parents ## unstoppable node should change or at terminal
                 if all([node[k] in terms or node[k]!=parent[k]
                         for k, terms in self.unstoppable_terminals.items()])])

        if hasattr(self.new_node_sampler, "init"):
            self.new_node_sampler.init(self, self.multiprocess_manager)
        if hasattr(self.parent_node_sampler, "init"):
            self.parent_node_sampler.init(self, self.multiprocess_manager)
        if hasattr(self.parent_snode_sampler, "init"):
            self.parent_snode_sampler.init(self, self.multiprocess_manager)
        if hasattr(self.binding_sampler, "init"):
            self.binding_sampler.init(self, self.multiprocess_manager)
        if hasattr(self.redundancy_sampler, "init"):
            self.redundancy_sampler.init(self, self.multiprocess_manager)
        if hasattr(self.custom_rule, "init"):
            self.custom_rule.init(self, self.multiprocess_manager)

        snode_root = self.make_search_node(None, initial_state, None, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)

    ##
    # @brief sample new state
    # @param lock lock instance to multiprocess
    def sample(self):
        sample_fail = True
        while sample_fail:
            self.reserved_attempt = False
            with self.reserve_lock:
                if not self.attempts_reseved.empty():
                    try:
                        parent_sidx, new_node = self.attempts_reseved.get(timeout=0.1)
                        self.reserved_attempt = True
                        print("got reserved one from {}".format(parent_sidx))
                    except:
                        pass
            if not self.reserved_attempt:
                try:
                    new_node = self.new_node_sampler(self.neighbor_nodes.keys())
                    parent_nodes = self.node_parent_dict[new_node]
                    parent_node = self.parent_node_sampler(list(parent_nodes.intersection(self.node_snode_dict.keys())))
                    parent_sidx = self.parent_snode_sampler(self.node_snode_dict[parent_node])
                    print("sampled one from {}".format(parent_sidx))
                except Exception as e:  ## currently occurs when terminating search in multiprocess
                    try:
                        print("ERROR sampling parent from : {} / parent nodes: {}".format(new_node, parent_nodes))
                    except:
                        print("ERROR sampling parent - NO SAMPLE REMAINED! add initial state again")
                        snode_root = self.make_search_node(None, self.initial_state, None, None)
                        self.connect(None, snode_root)
                        self.update(None, snode_root, True)
                    print(e)
                    sample_fail = True
                    continue
            parent_snode = self.snode_dict[parent_sidx]
            from_state = parent_snode.state
            if isinstance(new_node, State):
                to_state = new_node
                redundancy_dict = deepcopy(parent_snode.redundancy_dict)
            else:
                available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                                                  list2dict(from_state.Q, self.pscene.gscene.joint_names))
                if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                    print("============== Non-available transition: sample again =====================")
                    sample_fail = True
                    continue
                to_state, redundancy_dict = self.pscene.sample_leaf_state(from_state, available_binding_dict, new_node,
                                                                          binding_sampler=self.binding_sampler,
                                                                          redundancy_sampler=self.redundancy_sampler)
            sample_fail = False
        return parent_snode, from_state, to_state, redundancy_dict, sample_fail

    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        if hasattr(self.new_node_sampler, "update"):
            self.new_node_sampler.update(snode_src, snode_new, connection_result)
        if hasattr(self.parent_node_sampler, "update"):
            self.parent_node_sampler.update(snode_src, snode_new, connection_result)
        if hasattr(self.parent_snode_sampler, "update"):
            self.parent_snode_sampler.update(snode_src, snode_new, connection_result)

        if connection_result:
            node_new = snode_new.state.node
            for leaf in self.node_dict[node_new]:
                self.neighbor_nodes[leaf] = None

            with self.snode_dict_lock:
                if snode_new.state.node not in self.node_snode_dict:
                    self.node_snode_dict[snode_new.state.node] = [snode_new.idx]
                else:
                    self.node_snode_dict[snode_new.state.node] = \
                        self.node_snode_dict[snode_new.state.node]+[snode_new.idx]

            if self.check_goal(snode_new.state):
                return True

        cres = False
        if self.custom_rule is not None:
            cres, next_items = self.custom_rule(self, snode_src, snode_new, connection_result)
            if cres:
                print("make custom reservation: {}".format(next_items))
                for next_item in next_items:
                    self.attempts_reseved.put((snode_new.idx, next_item))

        if not cres:
            if connection_result:
                for gnode in self.goal_nodes:
                    if snode_new.state.node in self.node_parent_dict[gnode]:
                        self.attempts_reseved.put((snode_new.idx, gnode))
                        print("=============== try reaching goal from {} =================".format(node_new))
                    elif not self.reserved_attempt:
                        nodes_near = list(self.node_dict[node_new])
                        ## goal-matching item indexes
                        match_self_idx = np.where([ntem_s == ntem_g
                                                   for ntem_s, ntem_g in zip(node_new, gnode)])[0]
                        if len(match_self_idx)>0:
                            nodes_candi = [node_n   ## sample new node which matching items are remained
                                           for node_n in nodes_near
                                           if all([node_n[match_i] == node_new[match_i]
                                                   for match_i  in match_self_idx])]
                            node_extend = self.new_node_sampler(nodes_candi)
                            self.attempts_reseved.put((snode_new.idx, node_extend))
                            print("=============== try extend to goal {} -> {} =================".format(
                                node_new, node_extend))

        return False

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes