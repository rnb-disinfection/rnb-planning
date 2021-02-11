from abc import *
__metaclass__ = type
import numpy as np
from ...utils.utils import differentiate, SingleValue, DummyBlock
from ..scene import State

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

##
# @class SearchNode
# @brief search node
class SearchNode:
    ##
    # @param idx        incremental index of search node
    # @param state      rnb-planning.src.pkg.planning.scene.State
    # @param parents    list of parent SearchNode idx
    # @param leafs      list of available nodes
    # @param depth      depth of current node = number of parent
    # @param redundancy_dict defined redundancy of transition in dictionary form, {object name: {axis: value}}
    def __init__(self, idx, state, parents, leafs, depth=None,
                 redundancy_dict=None):
        self.idx, self.state, self.parents, self.leafs, self.depth, self.redundancy_dict = \
            idx, state, parents, leafs, depth, redundancy_dict
        self.traj = None
        self.traj_size = 0
        self.traj_length = 0
        self.traj_tot = 0

    ##
    # @brief    set current transition's trajectory + update trajectory length
    def set_traj(self, traj_full, traj_tot_parent):
        self.traj_size = len(traj_full)
        self.traj_length = np.sum(np.abs(differentiate(traj_full, 1)[:-1])) if self.traj_size > 1 else 0
        self.traj_tot = traj_tot_parent + self.traj_length
        self.traj = np.array(traj_full)

    def get_traj(self):
        return self.traj

    ##
    # @brief    copy SearchNode
    # @param    pscene  rnb-planning.src.pkg.planning.scene.PlanningScene
    def copy(self, pscene):
        return SearchNode(self.idx, State(self.state.binding_state, self.state.state_param, self.state.Q, pscene),
                          self.parents, self.leafs, self.depth, self.redundancy_dict)


##
# @class    TaskInterface
# @brief    class interface for task sampler
# @remark   Child classes should be implemented with initialize and search.
#           Child classes' constructor should call MotionInterface.__init__(self, pscene).
#           Specify NAME = "" for a child class as a class variable
class TaskInterface:
    NAME = None

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene):
        ## @brief rnb-planning.src.pkg.planning.scene.PlanningScene
        self.pscene = pscene
        ## @brief search node dictionary
        self.snode_dict = None
        ## @brief number of valid generated search nodes
        self.snode_counter = None

    ##
    # @brief (prototype) prepare search algorithm, bust be called before searching
    @abstractmethod
    def prepare(self):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        if multiprocess_manager is not None:
            self.snode_dict = multiprocess_manager.dict()
            self.snode_counter = multiprocess_manager.Value('i', 0)
            self.search_tree_lock = multiprocess_manager.Lock()
        else:
            self.snode_dict = {}
            self.snode_counter = SingleValue('i', 0)
            self.search_tree_lock = DummyBlock()

    ##
    # @brief (prototype) initialize searching loop with given initial state and goal. automatically called when starting search.
    # @remark Don't forget to make root SearchNode and connect, update it
    # @param initial_state rnb-planning.src.pkg.planning.scene.State
    # @param goal_nodes list of nodes
    @abstractmethod
    def init_search(self, initial_state, goal_nodes, multiprocess_manager):
        ## When overiding init_search, Don't forget to make root SearchNode and connect, update it, as below!
        snode_root = self.make_search_node(None, initial_state, None, None)
        self.connect(None, snode_root)
        self.update(snode_root, True)
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) sample new state
    # @param lock Lock() instance to lock multiprocess if required
    @abstractmethod
    def sample(self):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) update connection result to the searching algorithm
    @abstractmethod
    def update(self, snode_new, connection_result):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    @abstractmethod
    def check_goal(self, state):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief update snode_counter and snode.idx
    # @param snode_pre SearchNode
    # @param new_state State
    # @param traj traj from previous state to new state
    # @param redundancy_dict redundancy of current transition
    def make_search_node(self, snode_pre, new_state, traj,  redundancy_dict):
        if snode_pre is None:
            snode_new = SearchNode(idx=0, state=new_state, parents=[], leafs=[],
                                    depth=0)
        else:
            snode_new = SearchNode(
                idx=0, state=new_state, parents=snode_pre.parents + [snode_pre.idx], leafs=[],
                depth=len(snode_pre.parents) + 1, redundancy_dict=redundancy_dict)
            snode_new.set_traj(traj, snode_pre.traj_tot)
        return snode_new

    ##
    # @brief update snode_counter and snode.idx
    # @param snode_pre SearchNode
    # @param snode_new SearchNode
    # @param search_tree_lock lock instance for search tree
    def connect(self, snode_pre, snode_new):
        with self.search_tree_lock:
            snode_new.idx = self.snode_counter.value
            self.snode_dict[snode_new.idx] = snode_new
            self.snode_counter.value = self.snode_counter.value+1
            if snode_pre is not None:
                snode_pre.leafs += [snode_new.idx]
                self.snode_dict[snode_pre.idx] = snode_pre
        return snode_new


##
# @brief get dictionary of discrete node distance
def get_node_cost_dict(reference_node, node_dict, transit_cost=1.0):
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
            new_cost = node_cost_dict[current] + transit_cost
            if next not in node_cost_dict or new_cost < node_cost_dict[next]:
                node_cost_dict[next] = new_cost
                priority = new_cost
                frontier.put((priority, next))
                came_from[next] = current
    return node_cost_dict
