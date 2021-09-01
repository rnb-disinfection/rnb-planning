from .rrt import  *
from ..sampling.node_sampling import make_state_param_hashable

##
# @class    TaskRRTstar
# @brief    task level RRT* algorithm
class TaskRRTstar(TaskRRT):
    REWIND_MAX = 3
    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        rrt_res = TaskRRT.update(self, snode_src, snode_new, connection_result)
        if connection_result and not rrt_res:
            node_new = snode_new.state.node
            leafs = self.node_dict[node_new]
            reserve_stack = []
            for leaf in leafs:
                if leaf in self.node_snode_dict:
                    if leaf not in self.goal_nodes: # goal nodes are manually reached when possible. no need for additional resevation
                        sleafs = self.node_snode_dict[leaf]
                        sparam_set = set()
                        for sleaf in sleafs:
                            snode_candi = self.snode_dict[sleaf]
                            sparam = make_state_param_hashable(self.pscene, snode_candi.state, as_dict=False)
                            if sparam not in sparam_set:
                                sparam_set.add(sparam)
                                if snode_candi.depth > snode_new.depth:
                                    print("reserve {} -> {}".format(snode_new.idx, snode_candi.idx))
                                    reserve_stack.append((snode_new.idx, snode_candi.idx))
            for item in random.sample(reserve_stack, min(len(reserve_stack), self.REWIND_MAX)):
                self.attempts_reseved.put(item)
        return False