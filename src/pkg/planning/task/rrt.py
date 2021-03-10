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
from Queue import Queue

##
# @class    TaskRRT
# @brief    task level RRT algorithm
class TaskRRT(TaskInterface):
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
                if leaf in self.node_list:
                    self.node_dict[node].append(leaf)
                    self.node_parent_dict[leaf].append(node)
        for node in self.node_list:
            self.node_dict[node] = set(self.node_dict[node])
            self.node_parent_dict[node] = set(self.node_parent_dict[node])

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.neighbor_nodes = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.node_snode_dict = multiprocess_manager.dict()
            self.snode_dict_lock = multiprocess_manager.Lock()
            self.attempt_reseved = multiprocess_manager.Queue()
            self.reserve_lock = multiprocess_manager.Lock()
        else:
            self.neighbor_nodes = dict()
            self.node_snode_dict = dict()
            self.snode_dict_lock = DummyBlock()
            self.attempt_reseved = Queue()
            self.reserve_lock = DummyBlock()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, tree_margin=None, depth_margin=None):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes

        snode_root = self.make_search_node(None, initial_state, None, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)

    ##
    # @brief sample new state
    # @param lock lock instance to multiprocess
    def sample(self):
        sample_fail = True
        while sample_fail:
            get_reserved = False
            with self.reserve_lock:
                if not self.attempt_reseved.empty():
                    try:
                        parent_sidx, new_node = self.attempt_reseved.get(timeout=0.1)
                        get_reserved = True
                    except:
                        pass
            if not get_reserved:
                new_node = random.choice(self.neighbor_nodes.keys())
                parent_nodes = self.node_parent_dict[new_node]
                parent_node = random.choice(list(parent_nodes.intersection(self.node_snode_dict.keys())))
                parent_sidx = random.choice(self.node_snode_dict[parent_node])
            parent_snode = self.snode_dict[parent_sidx]
            from_state = parent_snode.state
            available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                            list2dict(from_state.Q, self.pscene.gscene.joint_names))
            if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                print("============== Non-available transition: sample again =====================")
                sample_fail = True
                continue
            to_state, redundancy_dict = self.pscene.sample_leaf_state(from_state, available_binding_dict, new_node)
            sample_fail = False
        return parent_snode, from_state, to_state, redundancy_dict, sample_fail

    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        ret = False
        if connection_result:
            for leaf in self.node_dict[snode_new.state.node]:
                self.neighbor_nodes[leaf] = None

            with self.snode_dict_lock:
                if snode_new.state.node not in self.node_snode_dict:
                    self.node_snode_dict[snode_new.state.node] = [snode_new.idx]
                else:
                    self.node_snode_dict[snode_new.state.node] = self.node_snode_dict[snode_new.state.node]+[snode_new.idx]

            ## check goal (custom)
            if self.check_goal(snode_new.state):
                ret = True
            else:
                for gnode in self.goal_nodes:
                    if snode_new.state.node in self.node_parent_dict[gnode]:
                        print("=============== try reaching goal =================")
                        self.attempt_reseved.put((snode_new.idx, gnode))
        return ret

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes