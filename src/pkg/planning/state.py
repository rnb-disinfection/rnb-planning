import numpy as np
from collections import defaultdict
from copy import deepcopy

##
# @class    State
# @brief    planning scene state
class State:
    ##
    # @param binding_state  BindingState
    # @param state_param    task state parameters
    # @param Q              robot joint configuration
    # @param pscene         PlanningScene instance
    def __init__(self, binding_state, state_param, Q, pscene):
        self.Q = np.array(Q)
        self.set_binding_state(pscene,binding_state, state_param)

    def set_binding_state(self, pscene, binding_state, state_param):
        ## @brief tuple of binding state ((object name, binding point, binder), ..)
        self.binding_state = binding_state
        self.state_param = pscene.get_state_param_update(self.binding_state, state_param)
        ## @brief tuple of simplified binding state (binder geometry name 1, binder geometry name 2, ..)
        self.node = pscene.get_node(self.binding_state, self.state_param)

    def get_tuple(self):
        return (self.binding_state, self.state_param, self.Q)

    def copy(self, pscene):
        return State(deepcopy(self.binding_state), deepcopy(self.state_param), deepcopy(self.Q), pscene)