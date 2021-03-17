from copy import deepcopy
from ....utils.utils import *
from ...constraint.constraint_subject import AbstractObject, SweepLineTask
from interface import CustomRuleInterface


##
# @class SweepEntranceControlRule
# @brief Strictly control entrance to sweep tasks, as they are fully defined and no need to be sampled multiple times
class SweepEntranceControlRule(CustomRuleInterface):
    def __init__(self, pscene):
        self.pscene = pscene
        self.chain_dict = pscene.get_robot_chain_dict()
        self.NUM_WP_TRIALS = 1

    def init(self, tplan, multiprocess_manager):
        self.multiprocess_manager = multiprocess_manager
        if multiprocess_manager is not None:
            self.call_count_dict = multiprocess_manager.dict()
            self.count_lock = multiprocess_manager.Lock()
        else:
            self.call_count_dict = dict()
            self.count_lock = DummyBlock()

        no_enter_sidxes = [stype == SweepLineTask for stype in enumerate(self.pscene.subject_type_list)]
        no_enter_initials = [tplan.initial_state.node[sidx] for sidx in no_enter_sidxes]
        self.sorted_unstop_keys = sorted(tplan.unstoppable_terminals.keys())

        self.node_dict = {}
        self.enter_dict = {}
        for node, leafs in tplan.node_dict.items():
            self.node_dict[node] = deepcopy(leafs)
            if not all([node[k] in terms for k, terms in tplan.unstoppable_terminals.items()]):
                tplan.node_dict[node] = set([])  ## unstoppable node change will be reserved by this custom rule
            else:
                # entering to unstoppable terminal is controlled by this rule
                self.enter_dict[node] = set([leaf for leaf in leafs
                                             if any(
                        [leaf[k] not in terms for k, terms in tplan.unstoppable_terminals.items()])
                                             and all(
                        [leaf[k] >= leaf[k + 1] for k in self.sorted_unstop_keys[:-1]])])  # the task is done in order
                tplan.node_dict[node] = set([leaf for leaf in leafs
                                             if
                                             all([leaf[k] in terms for k, terms in tplan.unstoppable_terminals.items()])
                                             ])

        self.node_parent_dict = {}
        for node, parents in tplan.node_parent_dict.items():
            self.node_parent_dict[node] = deepcopy(parents)
            tplan.node_parent_dict[node] = parents = set(
                [parent for parent in parents  ## unstoppable node change will be reserved by this custom rule
                 if all([parent[k] in terms for k, terms in tplan.unstoppable_terminals.items()])])

    def __call__(self, tplan, snode_src, snode_new, connection_result):
        #         print("CustomRule call")
        stack_res = False
        stack_items = []
        if connection_result and snode_new.state.node in self.enter_dict:
            with tplan.snode_dict_lock:
                nb_nodes = list(tplan.neighbor_nodes.keys())
                for nb_node in nb_nodes:
                    for ukey in self.sorted_unstop_keys:
                        if nb_node[ukey] < snode_new.state.node[ukey]:
                            del tplan.neighbor_nodes[nb_node]
                            break  # task is done in order

        if connection_result and snode_new.state.node in self.enter_dict:
            stack_res, stack_items = True, list(self.enter_dict[snode_new.state.node])

        if snode_src is not None:
            with self.count_lock:
                if snode_src.idx in self.call_count_dict:
                    self.call_count_dict[snode_src.idx] = self.call_count_dict[snode_src.idx] + [snode_new.state.node]
                else:
                    self.call_count_dict[snode_src.idx] = [snode_new.state.node]

            node_src = snode_src.state.node
            node_new = snode_new.state.node
            diff_sidxes = np.where([ntem_s != ntem_g for ntem_s, ntem_g in zip(node_src, node_new)])[0]
            #             print("{}->{} , diff: {}".format(node_src, node_new, diff_sidxes))
            if len(diff_sidxes) == 0:
                return stack_res, stack_items
            diff_sidx = diff_sidxes[0]
            diff_sname = self.pscene.subject_name_list[diff_sidx]
            diff_subject = self.pscene.subject_dict[diff_sname]

            if isinstance(diff_subject, SweepLineTask):
                #                 print("Rule for SweepLineTask")
                with tplan.snode_dict_lock:
                    if node_src[diff_sidx] not in tplan.unstoppable_terminals[
                        diff_sidx]:  # from intermediate wp -> remove access to them
                        snode_list = tplan.node_snode_dict[node_src]
                        if snode_src.idx in snode_list:
                            #                             print("Removing {} from {}: {} not in {}".format(snode_src.idx, snode_list, node_src[diff_sidx], tplan.unstoppable_terminals[diff_sidx]))
                            #                             print(snode_list)
                            snode_list.remove(snode_src.idx)
                            tplan.node_snode_dict[node_src] = snode_list
                            print(tplan.node_snode_dict[node_src])
                        if len(snode_list) == 0 and node_new in tplan.neighbor_nodes:
                            del tplan.neighbor_nodes[node_new]
                    if connection_result:
                        if node_new[diff_sidx] in tplan.unstoppable_terminals[diff_sidx]:  ## in terminal condition
                            #                             print("Check home: {} in {}".format(node_new[diff_sidx], tplan.unstoppable_terminals[diff_sidx]))
                            link_name = self.pscene.gscene.NAME_DICT[
                                snode_new.state.binding_state[diff_sidx][-1]].link_name
                            rname_candis = [rname for rname, chain_vals in self.chain_dict.items() if
                                            link_name in chain_vals['link_names']]
                            if len(rname_candis) == 0:
                                print("no robot candis")
                            else:
                                #                                 print("reserve homing = {}".format(snode_new.idx))
                                newstate = snode_new.state.copy(self.pscene)
                                jidxes = self.pscene.combined_robot.idx_dict[rname_candis[0]]
                                newstate.Q[jidxes] = self.pscene.combined_robot.home_pose[jidxes]
                                return True, [newstate] + stack_items
                        else:  ## not in terminal condition
                            #                             print("Reserve next wp from {}".format(snode_new.idx))
                            snode_list = tplan.node_snode_dict[node_new]
                            if snode_new.idx in snode_list:
                                #                                 print("Removing {} from {} as next wp is reserved".format(snode_new.idx, snode_list))
                                snode_list.remove(snode_new.idx)
                                tplan.node_snode_dict[node_new] = snode_list
                            next_node_candis = list(self.node_dict[node_new])
                            assert len(
                                next_node_candis) == 1, "non-terminal sweep task should have only 1 leaf ({}) {}-{}".format(
                                diff_sidx, node_new, next_node_candis)
                            return True, next_node_candis * self.NUM_WP_TRIALS + stack_items
                return stack_res, stack_items

            if isinstance(diff_subject, AbstractObject):
                #                 print("Rule for AbstractObject")
                if connection_result:
                    link_name1 = snode_src.state.state_param[diff_sname][0]
                    link_name2 = snode_new.state.state_param[diff_sname][0]
                    rname_candis = [rname for rname, chain_vals in self.chain_dict.items() if
                                    link_name1 in chain_vals['link_names'] or link_name2 in chain_vals['link_names']]
                    if len(rname_candis) == 0:
                        print("no robot candis")
                        return stack_res, stack_items
                    else:
                        #                         print("try homing")
                        newstate = snode_new.state.copy(self.pscene)
                        jidxes = self.pscene.combined_robot.idx_dict[rname_candis[0]]
                        newstate.Q[jidxes] = self.pscene.combined_robot.home_pose[jidxes]
                        return True, [newstate] + stack_items
        return stack_res, stack_items