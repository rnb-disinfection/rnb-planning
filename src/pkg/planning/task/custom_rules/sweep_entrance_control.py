from copy import deepcopy
from ....utils.utils import *
from ...constraint.constraint_subject import AbstractObject, SweepLineTask, SubjectType
from interface import CustomRuleInterface


##
# @class SweepEntranceControlRule
# @brief Strictly control entrance to sweep tasks, as they are fully defined and no need to be sampled multiple times
class SweepEntranceControlRule(CustomRuleInterface):
    def __init__(self, pscene):
        self.pscene = pscene
        self.chain_dict = pscene.robot_chain_dict
        self.NUM_WP_TRIALS = 1

    def init(self, tplan, multiprocess_manager):
        self.multiprocess_manager = multiprocess_manager

        no_enter_sidxes = [stype == SweepLineTask for stype in enumerate(self.pscene.subject_type_list)]
        no_enter_initials = [tplan.initial_state.node[sidx] for sidx in no_enter_sidxes]

        self.node_dict = {}
        self.enter_dict = {}
        for node, leafs in tplan.node_dict.items():
            self.node_dict[node] = deepcopy(leafs)
            if not tplan.check_unstoppable_terminals(node):
                tplan.node_dict[node] = set([])  ## unstoppable node change will be reserved by this custom rule
            else:
                # entering to unstoppable terminal is controlled by this rule
                self.enter_dict[node] = set([leaf for leaf in leafs
                                             if not tplan.check_unstoppable_terminals(leaf)
                                             and all(
                        [leaf[k] >= leaf[k + 1] for k in self.sorted_unstop_keys[:-1]])])  # the task is done in order
                tplan.node_dict[node] = set([leaf for leaf in leafs
                                             if tplan.check_unstoppable_terminals(leaf)])

        self.node_parent_dict = {}
        for node, parents in tplan.node_parent_dict.items():
            self.node_parent_dict[node] = deepcopy(parents)
            tplan.node_parent_dict[node] = set(
                [parent for parent in parents  ## unstoppable node change will be reserved by this custom rule
                 if tplan.check_unstoppable_terminals(parent)])

    ##
    # @brief    re-foliate for some defined cases
    # @remark   case 1: meaningless homing - stayed on same node for two turns and do action of same kind \n
    #           case 2: already-moved object
    def refoliate(self, tplan, new_node, parent_sidx):
        # return new_node, parent_sidx, False
        reject = False
        parent_snode = tplan.snode_dict[parent_sidx]
        parent_node = parent_snode.state.node
        anc_nodes = [tplan.snode_dict[pidx].state.node for pidx in parent_snode.parents]
        if new_node != parent_node:  # this is not homing motion
            # subject_gname_list = [obj.geometry.name for obj in self.pscene.subject_dict.values()
            #                       if obj.stype == SubjectType.OBJECT]
            active_binder_geo = [b for a, b in zip(parent_node, new_node) if a != b][0]
            if any([active_binder_geo in subject.geometry.get_family()
                    for subject in self.pscene.subject_dict.values() if isinstance(subject, AbstractObject)]):     # don't put on other object
                reject = True
                return new_node, parent_sidx, reject

            if len(anc_nodes) > 1 and anc_nodes[-1] == parent_node:  # previous motion was homing
                if ([a for a, b in zip(anc_nodes[-2], parent_node) if a != b][0]
                        == [b for a, b in zip(parent_node, new_node) if a != b][0]):  # use same binder after homing
                    parent_sidx = parent_snode.parents[-1]
                    parent_snode = tplan.snode_dict[parent_sidx]
                    parent_node = parent_snode.state.node
                    anc_nodes = anc_nodes[:-1]
            if len(anc_nodes) > 0 and new_node in anc_nodes:  # already-moved object
                active_binder_geo = [b for a, b in zip(parent_node, new_node) if a != b][0]
                if ((active_binder_geo not in self.pscene.geometry_actor_dict)  # active binder is doing Task
                        or
                        (self.pscene.actor_robot_dict[self.pscene.geometry_actor_dict[active_binder_geo][0]]
                         is not None)  # active binder is controllable actor
                ):
                    first_access_idx = anc_nodes.index(new_node)
                    if first_access_idx == 0:  # this is returning to initial state
                        reject = True
                    else:  # foliate from before first move of this object
                        parent_sidx = parent_snode.parents[first_access_idx - 1]
                        parent_snode = tplan.snode_dict[parent_sidx]
                        parent_node = parent_snode.state.node
                        anc_nodes = anc_nodes[:first_access_idx - 1]
        return new_node, parent_sidx, reject

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

            node_src = snode_src.state.node
            node_new = snode_new.state.node
            diff_ntems = [(sname, ntem_s, ntem_g)
                           for sname, ntem_s, ntem_g
                           in zip(tplan.pscene.subject_name_list, node_src, node_new) if ntem_s != ntem_g]
            if len(diff_ntems) == 0:
                return stack_res, stack_items
            diff_sname, diff_ntem_s, diff_ntem_g = diff_ntems[0]
            diff_subject = self.pscene.subject_dict[diff_sname]

            if isinstance(diff_subject, SweepLineTask): # Sweep entrance control rule
                #                 print("Rule for SweepLineTask")
                with tplan.snode_dict_lock:
                    if diff_ntem_s not in tplan.unstoppable_terminals[diff_sname]:
                        # from intermediate wp -> remove access to them
                        snode_list = tplan.node_snode_dict[node_src]
                        if snode_src.idx in snode_list:
                            # print("Removing {} from {}: {} not in {}".format(
                            #     snode_src.idx, snode_list,
                            #     node_src[diff_sidx], tplan.unstoppable_terminals[diff_sidx]))
                            # print(snode_list)
                            snode_list.remove(snode_src.idx)
                            tplan.node_snode_dict[node_src] = snode_list
                            print(tplan.node_snode_dict[node_src])
                        if len(snode_list) == 0 and node_new in tplan.neighbor_nodes:
                            del tplan.neighbor_nodes[node_new]
                    if connection_result:
                        if diff_ntem_g in tplan.unstoppable_terminals[diff_sname]:  ## in terminal condition
                            # print("Check home: {} in {}".format(
                            #     node_new[diff_sidx], tplan.unstoppable_terminals[diff_sidx]))
                            link_name = self.pscene.gscene.NAME_DICT[
                                snode_new.state.binding_state[diff_sname].chain.actor_root_gname].link_name
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
                            # print("Reserve next wp from {}".format(snode_new.idx))
                            snode_list = tplan.node_snode_dict[node_new]
                            if snode_new.idx in snode_list:
                                # print("Removing {} from {} as next wp is reserved".format(snode_new.idx, snode_list))
                                snode_list.remove(snode_new.idx)
                                tplan.node_snode_dict[node_new] = snode_list
                            next_node_candis = list(self.node_dict[node_new])
                            assert len(next_node_candis) == 1, \
                                "non-terminal sweep task should have 1 leaf ({}) {}-{}".format(
                                    diff_sname, node_new, next_node_candis)
                            return True, next_node_candis * self.NUM_WP_TRIALS + stack_items
                    elif len(tplan.neighbor_nodes)==0 and not stack_res: # connection failed but no other new node
                        print("connection failed but no other new node - retry previous one")
                        return True, [snode_new.state.node]
                return stack_res, stack_items

            if isinstance(diff_subject, AbstractObject):
                #                 print("Rule for AbstractObject")
                if connection_result:
                    link_name1 = snode_src.state.binding_state[diff_sname].actor_link[0]
                    link_name2 = snode_new.state.binding_state[diff_sname].actor_link[0]
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