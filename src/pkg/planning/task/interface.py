from abc import *
import numpy as np
from ..scene import State
from ..scene import node2onode
import time
from collections import defaultdict


__metaclass__ = type


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

    ##
    # @brief (prototype) prepare search algorithm
    @abstractmethod
    def prepare(self):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) initialize searching loop with given initial state and goal
    # @param initial_state rnb-planning.src.pkg.planning.scene.State
    # @param goal definition of goal, different depending on algorithm
    @abstractmethod
    def init_search(self, initial_state, goal, *args, **kwargs):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) get sample leafs from snode
    # @param snode A validated SearchNode of which leafs should be added to queue
    # @param N_redundant_sample number of redundant samples
    # @return snode_tuple_list list of tuple(priority, (snode, from_state, to_state, redundancy))
    @abstractmethod
    def get_leafs(self, snode, N_redundant_sample):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) get optimal remaining steps
    # @param state rnb-planning.src.pkg.planning.scene.State
    @abstractmethod
    def get_optimal_remaining_steps(self, state):
        raise(NotImplementedError("abstract method"))

    ##
    # @brief (prototype) check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    @abstractmethod
    def check_goal(self, state):
        raise(NotImplementedError("abstract method"))

