import numpy as np
from ...utils.utils import DummyBlock

class NodeSampler:
    def __init__(self, gamma_decay=0.8):
        self.gamma_decay = gamma_decay

    def init(self, node_dict, multiprocess_manager):
        self.node_dict = node_dict
        self.multiprocess_manager = multiprocess_manager
        if multiprocess_manager is not None:
            self.prob_dict = multiprocess_manager.dict()
            self.prob_lock = multiprocess_manager.Lock()
        else:
            self.prob_dict = dict()
            self.prob_lock = DummyBlock()

        for k in self.node_dict.keys():
            self.prob_dict[k] = 1.0

    def __call__(self, nodes):
        nodes = list(nodes)
        probs = [self.prob_dict[node] for node in nodes]
        probs = np.divide(probs, np.sum(probs))
        i_node = np.random.choice(len(nodes), p=probs)
        node = nodes[i_node]
        with self.prob_lock:
            self.prob_dict[node] = self.prob_dict[node] * self.gamma_decay
        return node