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
    # @param    pscene              rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param    allow_joint_motion  allow random joint motion, which is transition to same node. \r\n
    # @param    config_gen          configuration generator for random joint motion. by default, this homing motion is generated.
    def __init__(self, pscene, allow_joint_motion=False, config_gen=None,
                 new_node_sampler=random.choice, parent_node_sampler=random.choice, parent_snode_sampler=random.choice,
                 binding_sampler=random.choice, redundancy_sampler=random.uniform, custom_rule=None, node_trial_max=1e3,
                 random_try_goal=True, explicit_edges=None, explicit_rule=None, node_count_max=1e2):
        TaskInterface.__init__(self, pscene)
        self.new_node_sampler = new_node_sampler
        self.parent_node_sampler = parent_node_sampler
        self.parent_snode_sampler = parent_snode_sampler
        self.binding_sampler = binding_sampler
        self.redundancy_sampler = redundancy_sampler
        self.custom_rule = custom_rule
        self.explicit_edges = {} if explicit_edges is None else explicit_edges
        self.node_trial_max = node_trial_max
        self.random_try_goal = random_try_goal
        self.explicit_rule = (lambda pscene, node, leaf: True) if explicit_rule is None else explicit_rule
        self.allow_joint_motion = allow_joint_motion
        self.config_gen = config_gen
        self.node_count_max = node_count_max

    ##
    # @brief build object-level node graph
    def prepare(self):
        pscene = self.pscene

        # make all node connections
        self.node_list = pscene.get_all_nodes()
        self.node_dict_full = {k: [k] if self.allow_joint_motion else [] for k in self.node_list}
        self.node_parent_dict_full = {k: [k] if self.allow_joint_motion else [] for k in self.node_list}
        for node in self.node_list:
            for leaf in pscene.get_node_neighbor(node):
                if leaf in self.node_list:
                    self.node_dict_full[node].append(leaf)
                    self.node_parent_dict_full[leaf].append(node)
        for node in self.node_list:
            self.node_dict_full[node] = set(self.node_dict_full[node])
            self.node_parent_dict_full[node] = set(self.node_parent_dict_full[node])

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.neighbor_nodes = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.node_snode_dict = multiprocess_manager.dict()
            self.node_trial_dict = multiprocess_manager.dict()
            self.neighbor_node_lock = multiprocess_manager.Lock()
            self.snode_dict_lock = multiprocess_manager.Lock()
            self.attempts_reseved = multiprocess_manager.Queue()
            self.reserve_lock = multiprocess_manager.Lock()
        else:
            self.neighbor_nodes = dict()
            self.node_snode_dict = dict()
            self.node_trial_dict = dict()
            self.neighbor_node_lock = DummyBlock()
            self.snode_dict_lock = DummyBlock()
            self.attempts_reseved = Queue()
            self.reserve_lock = DummyBlock()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, **kwargs):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.reserved_attempt = False

        self.node_dict = {}
        self.node_parent_dict = defaultdict(set)
        for node, leafs in self.node_dict_full.items():
            self.node_trial_dict[node] = self.node_trial_max
            ## goal node does not have child leaf
            if node in goal_nodes:
                self.node_dict[node] = set()
                continue
            leaf_list = [leaf
                         for leaf in leafs
                         if self.explicit_rule(self.pscene, node, leaf)]
            self.node_dict[node] = set(leaf_list)
            if node in self.explicit_edges:
                self.node_dict[node] = self.node_dict[node].union(self.explicit_edges[node])
            for leaf in self.node_dict[node]:
                self.node_parent_dict[leaf].add(node)
        self.node_parent_dict = dict(self.node_parent_dict)

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

        snode_root = self.make_search_node(None, initial_state, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)

    ##
    # @brief sample new state
    # @param lock lock instance to multiprocess
    def sample(self):
        sample_fail = True
        fail_count = 0
        parent_snode, from_state, to_state = None, None, None
        while sample_fail and fail_count<3:
            fail_count += 1
            self.reserved_attempt = False
            with self.reserve_lock:
                if not self.attempts_reseved.empty():
                    try:
                        parent_sidx, new_node = self.attempts_reseved.get(timeout=0.1)
                        self.reserved_attempt = True
                        # print("gettting reserved one from {}: {}".format(parent_sidx, new_node))
                    except Exception as e:
                        print("error in gettting reserved one from {}".format(parent_sidx))
                        print(e)
                        pass
            if not self.reserved_attempt:
                try:
                    node_keys = self.neighbor_nodes.keys()
                    if len(node_keys)>0:
                        new_node = self.new_node_sampler(node_keys)
                        node_trial = self.node_trial_dict[new_node]
                        node_trial -= 1
                        self.node_trial_dict[new_node] = node_trial
                        if node_trial <= 0:
                            with self.neighbor_node_lock:
                                self.neighbor_nodes.pop(new_node)
                    else:
                        # print("ERROR sampling parent - NO SAMPLE REMAINED! Return None to stop")
                        return None, None, None, True
                    parent_nodes = self.node_parent_dict[new_node]
                    parent_node = self.parent_node_sampler(list(parent_nodes.intersection(self.node_snode_dict.keys())))
                    parent_sidx = self.parent_snode_sampler(self.node_snode_dict[parent_node])
                    if hasattr(self.custom_rule, "refoliate"):
                        new_node, parent_sidx, reject = self.custom_rule.refoliate(self, new_node, parent_sidx)
                        if reject:
                            sample_fail = True
                            continue
                    # print("sampled one from {}".format(parent_sidx))
                except Exception as e:  ## currently occurs when terminating search in multiprocess
                    try:
                        print("ERROR sampling parent from : {} / parent nodes: {}".format(new_node, parent_nodes))
                    except:
                        print("ERROR sampling parent - Failed to get new_node or parent_nodes")
                        # snode_root = self.make_search_node(None, self.initial_state, None)
                        # self.connect(None, snode_root)
                        # self.update(None, snode_root, True)
                        time.sleep(0.5)
                    print(e)
                    sample_fail = True
                    continue
            parent_snode = self.snode_dict[parent_sidx]
            from_state = parent_snode.state
            if isinstance(new_node, State):
                to_state = new_node
            elif isinstance(new_node, int): # for copy and extend existing SearchNode, especially for RRT* extension
                snode_tar = self.snode_dict[new_node]
                to_state = snode_tar.state
            else:
                available_binding_dict = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                                list2dict(from_state.Q, self.pscene.gscene.joint_names))
                if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                    print("============== Non-available transition: sample again =====================")
                    sample_fail = True
                    continue
                to_state = self.pscene.sample_leaf_state(from_state, available_binding_dict, new_node,
                                                         binding_sampler=self.binding_sampler,
                                                         redundancy_sampler=self.redundancy_sampler,
                                                         config_gen=self.config_gen)
            sample_fail = False
        return parent_snode, from_state, to_state, sample_fail

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
                if self.random_try_goal or leaf not in self.goal_nodes: # goal nodes are manually reached below when possible. no need for random access
                    if self.node_trial_dict[leaf]>0:
                        self.neighbor_nodes[leaf] = None # register as neighbor node group

            with self.snode_dict_lock:
                if snode_new.state.node not in self.node_snode_dict:
                    snode_list = [snode_new.idx]
                else:
                    snode_list = self.node_snode_dict[snode_new.state.node] + [snode_new.idx]
                self.node_snode_dict[snode_new.state.node] = snode_list
            if len(snode_list) > self.node_count_max:
                with self.neighbor_node_lock:
                    self.node_trial_dict[snode_new.state.node] = -1  # don't register this node anymore
                    if snode_new.state.node in self.neighbor_nodes:
                        self.neighbor_nodes.pop(snode_new.state.node)   # remove this node from neighbor group

            if self.check_goal(snode_new.state):
                print("Goal reached")
                return True

        cres = False
        if self.custom_rule is not None:
            cres, next_items = self.custom_rule(self, snode_src, snode_new, connection_result)
            if cres:
                # print("make custom reservation: {} -> {}".format(snode_new.idx, next_items))
                for next_item in next_items:
                    from_idx = snode_new.idx if connection_result else snode_src.idx
                    self.attempts_reseved.put((from_idx, next_item))

        if not cres:
            if connection_result:
                for gnode in self.goal_nodes:
                    if snode_new.state.node in self.node_parent_dict[gnode]:
                        self.attempts_reseved.put((snode_new.idx, gnode))
                        # print("=============== try reaching goal from {} =================".format(node_new))
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
                            try:
                                node_extend = self.new_node_sampler(nodes_candi)
                                self.attempts_reseved.put((snode_new.idx, node_extend))
                                # print("=============== try extend to goal {} -> {} =================".format(
                                #     node_new, node_extend))
                            except Exception as e:
                                print(e)

        return False