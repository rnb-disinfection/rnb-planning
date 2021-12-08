import numpy as np
from ...utils.utils import DummyBlock
from ..constraint.constraint_subject import AbstractObject, AbstractTask

import random

##
# @brief make state parameters hashable and return as dictionary
def make_state_param_hashable(pscene, state, as_dict=True):
    ## get hashable param dict
    param_flat_dict = {}
    for i_obj, oname in enumerate(pscene.subject_name_list):
        subject = pscene.subject_dict[oname]
        if isinstance(subject, AbstractObject):
            btf = state.binding_state[oname]
            link_name, param = btf.actor_link, btf.T_lao
            param_flat = (state.node[i_obj], link_name) + tuple(
                np.round(param, 4).flatten())  ## make state params hashable by flattenning
        elif isinstance(subject, AbstractTask):
            param = state.state_param[oname]
            if isinstance(param, np.ndarray):
                param_hash = param.flatten()
            else:
                param_hash = str(param)
            param_flat = (state.node[i_obj],) + tuple(param_hash)  ## make state params hashable by flattenning
        else:
            raise(NotImplementedError("Unknown subject"))
        param_flat_dict[oname] = param_flat
    if as_dict:
        return param_flat_dict
    else:
        return tuple([param_flat_dict[sname] for sname in pscene.subject_name_list])

##
# @class NodeSampler
# @biref uniformly sample nodes by decaying probability of sampled node
class UniformNodeSampler:
    def __init__(self, log2_decay=1):
        self.log2_decay = log2_decay

    def init(self, tplan, multiprocess_manager):
        self.node_dict = tplan.node_dict
        self.multiprocess_manager = multiprocess_manager
        if multiprocess_manager is not None:
            self.log2_prob_dict = multiprocess_manager.dict()
            self.prob_lock = multiprocess_manager.Lock()
        else:
            self.log2_prob_dict = dict()
            self.prob_lock = DummyBlock()

        for k in self.node_dict.keys():
            self.log2_prob_dict[k] = 1.0

    def __call__(self, nodes):
        nodes = list(nodes)
        probs = [np.exp2(self.log2_prob_dict[node]) for node in nodes]
        sumprobs = np.sum(probs)
        if sumprobs == 0:
            print("probability saturated")
            # probs = np.ones_like(probs)/len(probs)
            with self.prob_lock:
                for k in self.node_dict.keys():
                    self.log2_prob_dict[k] = 1.0
        else:
            probs = np.divide(probs, sumprobs)
        i_node = np.random.choice(len(nodes), p=probs)
        node = nodes[i_node]
        with self.prob_lock:
            self.log2_prob_dict[node] = self.log2_prob_dict[node] - self.log2_decay
        return node


##
# @class GrowingSampler
# @biref Grow sampler keys as new items are added, and sample uniformly
class GrowingSampler:
    def __init__(self, log2_decay=1):
        self.log2_decay = log2_decay

    def init(self, tplan, multiprocess_manager):
        self.node_dict = tplan.node_dict
        self.multiprocess_manager = multiprocess_manager
        if multiprocess_manager is not None:
            self.log2_prob_dict = multiprocess_manager.dict()
            self.prob_lock = multiprocess_manager.Lock()
        else:
            self.log2_prob_dict = dict()
            self.prob_lock = DummyBlock()

    def __call__(self, nodes):
        nodes = list(nodes)
        for node in nodes:
            if node not in self.log2_prob_dict:
                self.log2_prob_dict[node] = 1.0
        probs = [np.exp2(self.log2_prob_dict[node]) for node in nodes]
        sumprobs = np.sum(probs)
        if sumprobs == 0:
            print("probability saturated")
            #             probs = np.ones_like(probs)/len(probs)
            with self.prob_lock:
                for k in self.node_dict.keys():
                    self.log2_prob_dict[k] = 1.0
        else:
            probs = np.divide(probs, sumprobs)
        i_node = np.random.choice(len(nodes), p=probs)
        node = nodes[i_node]
        with self.prob_lock:
            self.log2_prob_dict[node] = self.log2_prob_dict[node] - self.log2_decay
        return node


##
# @class PenaltyNodeSampler
# @biref Give penalty for used samples
class PenaltyNodeSampler:
    def __init__(self, log2_decay=1, log2_count_decay=1):
        self.log2_decay = log2_decay
        self.log2_count_decay = log2_count_decay

    def init(self, tplan, multiprocess_manager):
        self.node_dict = tplan.node_dict
        self.multiprocess_manager = multiprocess_manager
        if multiprocess_manager is not None:
            self.log2_prob_dict = multiprocess_manager.dict()
            self.log2_count_dict = multiprocess_manager.dict()
            self.prob_lock = multiprocess_manager.Lock()
        else:
            self.log2_prob_dict = dict()
            self.log2_count_dict = dict()
            self.prob_lock = DummyBlock()

        for k in self.node_dict.keys():
            self.log2_prob_dict[k] = 1.0
            self.log2_count_dict[k] = 0.0

    def __call__(self, nodes):
        nodes = list(nodes)
        probs = [np.exp2(self.log2_prob_dict[node] + self.log2_count_dict[node]) for node in nodes]
        sumprobs = np.sum(probs)
        if sumprobs == 0:
            print("probability saturated")
            #             probs = np.ones_like(probs)/len(probs)
            with self.prob_lock:
                for k in self.node_dict.keys():
                    self.log2_prob_dict[k] = 1.0
                    self.log2_count_dict[k] = 0.0
        else:
            probs = np.divide(probs, sumprobs)
        i_node = np.random.choice(len(nodes), p=probs)
        node = nodes[i_node]
        with self.prob_lock:
            self.log2_prob_dict[node] = self.log2_prob_dict[node] - self.log2_decay
        return node

    def update(self, snode_src, snode_new, connection_result):
        if connection_result:
            self.log2_count_dict[snode_new.state.node] = self.log2_count_dict[
                                                             snode_new.state.node] - self.log2_count_decay
