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
    def __init__(self, idx, state, parents, leafs, depth=None, traj=None, traj_tot_parent=0):
        self.traj_tot, self.traj_length = 0, 0
        self.idx, self.state, self.parents, self.leafs, self.depth = \
            idx, state, parents, leafs, depth
        self.set_traj(traj, traj_tot_parent)

    ##
    # @brief    set current transition's trajectory + update trajectory length
    def set_traj(self, traj_full, traj_tot_parent=None):
        if traj_full is not None:
            if traj_tot_parent is None:
                traj_tot_parent = self.traj_tot - self.traj_length
            self.traj_size = len(traj_full)
            self.traj_length = np.sum(np.abs(differentiate(traj_full, 1)[:-1])) if self.traj_size > 1 else 0
            self.traj_tot = traj_tot_parent + self.traj_length
            self.traj = np.array(traj_full)
        else:
            self.traj = None
            self.traj_size = 0
            self.traj_length = 0
            if traj_tot_parent is not None:
                self.traj_tot = traj_tot_parent

    def get_traj(self):
        return self.traj

    ##
    # @brief    copy SearchNode
    # @param    pscene  rnb-planning.src.pkg.planning.scene.PlanningScene
    def copy(self, pscene):
        return SearchNode(self.idx, State(self.state.binding_state, self.state.state_param, self.state.Q, pscene),
                          self.parents, self.leafs, self.depth,
                          self.traj, self.traj_tot-self.traj_length)


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
        self.snode_dict = {}
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
        self.multiprocess_manager = multiprocess_manager
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
        snode_root = self.make_search_node(None, initial_state, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)
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
    def update(self, snode_src, snode_new, connection_result):
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
    def make_search_node(self, snode_pre, new_state, traj):
        if snode_pre is None:
            snode_new = SearchNode(idx=0, state=new_state, parents=[], leafs=[],
                                    depth=0)
        else:
            snode_new = SearchNode(
                idx=0, state=new_state, parents=snode_pre.parents + [snode_pre.idx], leafs=[],
                depth=len(snode_pre.parents) + 1)
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
    # @brief find all schedules
    # @return list of SearchNode index list
    def find_schedules(self, at_home=True, home_pose=None, in_indices=True):
        schedule_dict = {}
        sidx_checked = set()
        home_pose = self.initial_state.Q if home_pose is None else home_pose
        for i in reversed(sorted(self.snode_dict.keys())):
            if i in sidx_checked:
                continue
            snode = self.snode_dict[i]
            state = snode.state
            # parent should be checked - there are bi-directional trees
            if (self.check_goal(state) and len(snode.parents)>0 and snode.parents[0] == 0):
                if at_home and np.linalg.norm(state.Q-home_pose)>1e-2:
                    continue
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
                sidx_checked = sidx_checked.union(schedule)
        if in_indices:
            return schedule_dict
        else:
            return {k: self.idxSchedule2SnodeScedule(v) for k, v in schedule_dict.items()}


    ##
    # @brief find schedule with shortest path
    # @returnlist of SearchNode instances
    def get_best_schedule(self, at_home=True, home_pose=None):
        schedules = self.find_schedules(at_home=at_home, home_pose=home_pose)
        schedules_sorted = self.sort_schedule(schedules)
        if len(schedules_sorted)==0:
            return []
        snode_schedule = self.idxSchedule2SnodeScedule(schedules_sorted[0])
        return snode_schedule

    ##
    # @brief find all schedules
    def print_snode_list(self):
        for i_s, snode in sorted(self.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.tplan.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

    ##
    # @brief sort schedules
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: self.snode_dict[x[-1]].traj_tot)

    ##
    # @brief get list of SearchNode from list of SearchNode index
    def idxSchedule2SnodeScedule(self, schedule):
        snode_schedule = [self.snode_dict[i_sc] for i_sc in schedule]
        return snode_schedule


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
