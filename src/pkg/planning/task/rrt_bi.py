from .rrt import  *
from copy import deepcopy
from ..constraint.constraint_subject import AbstractObject, AbstractTask
from ..constraint.constraint_common import combine_redundancy, sample_redundancy, calc_redundancy
from ..filtering.grasp_filter import GraspChecker



##
# @class    TaskBiRRT
# @brief    task level Bi-RRT algorithm
class TaskBiRRT(TaskInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene, gcheck, goal_trial_count=3, flag_swap=False):
        TaskInterface.__init__(self, pscene)
        self.gcheck = gcheck
        self.goal_trial_count = goal_trial_count
        self.flag_swap = flag_swap

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

        self.unstoppable_subjects = [i_s for i_s, sname in enumerate(self.pscene.subject_name_list)
                                     if self.pscene.subject_dict[sname].unstoppable]

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.snode_dict_lock = multiprocess_manager.Lock()

            self.node_snode_dict = multiprocess_manager.dict()
            self.visited_snodes = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.neighbor_nodes = multiprocess_manager.dict()
            self.param_snode_dict = {k: multiprocess_manager.dict() for k in self.pscene.subject_name_list}

            self.node_snode_dict_rev = multiprocess_manager.dict()
            self.visited_snodes_rev = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.neighbor_nodes_rev = multiprocess_manager.dict()
            self.param_snode_dict_rev = {k: multiprocess_manager.dict() for k in self.pscene.subject_name_list}

            self.attempt_reseved = multiprocess_manager.Queue()
            self.reserve_lock = multiprocess_manager.Lock()
        else:
            self.snode_dict_lock = DummyBlock()

            self.node_snode_dict = dict()
            self.visited_snodes = dict()
            self.neighbor_nodes = dict()
            self.param_snode_dict = {k: dict() for k in self.pscene.subject_name_list}

            self.node_snode_dict_rev = dict()
            self.visited_snodes_rev = dict()
            self.neighbor_nodes_rev = dict()
            self.param_snode_dict_rev = {k: dict() for k in self.pscene.subject_name_list}

            self.attempt_reseved = Queue()
            self.reserve_lock = DummyBlock()

        self.reset_trees()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, tree_margin=None, depth_margin=None):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.target_sidx = -1
        self.bool_forward = True

        self.unstoppable_terminals = {}
        for sub_i in self.unstoppable_subjects:
            self.unstoppable_terminals[sub_i] = [self.initial_state.node[sub_i]]
            for goal in goal_nodes:
                self.unstoppable_terminals[sub_i].append(goal[sub_i])

        snode_root = self.make_search_node(None, initial_state, None, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)

    ##
    # @brief sample new state
    # @param lock lock instance to multiprocess
    def sample(self):
        sample_fail = True
        while sample_fail:
            self.target_sidx = -1  # save target sidx
            self.reserved_attempt = False  # flag for reserved attempt (no additional extension attempt)
            with self.reserve_lock:
                if not self.attempt_reseved.empty():
                    #                     try:
                    parent_sidx, new_item, tree_cur, tree_tar, direct = self.attempt_reseved.get(timeout=0.1)
                    parent_snode = self.snode_dict[parent_sidx]
                    from_state = parent_snode.state
                    self.visited_snodes_cur, self.neighbor_nodes_cur, self.node_snode_dict_cur = tree_cur
                    self.visited_snodes_cur, self.neighbor_nodes_tar, self.node_snode_dict_tar = tree_tar
                    if direct:
                        self.target_sidx = new_item
                        snode_new = self.snode_dict[self.target_sidx]
                        to_state = snode_new.state
                        # redundancy_dict = deepcopy(parent_snode.redundancy_dict) or {}
                        redundancy_dict = deepcopy(snode_new.redundancy_dict) or {}
                        self.reserved_attempt = True
                        sample_fail = False
                        print("direct reaching: {} -> {}".format(parent_sidx, self.target_sidx))
                    else:
                        new_node = new_item
                        available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state,
                                                                                                          new_node,
                                                                                                          list2dict(
                                                                                                              from_state.Q,
                                                                                                              self.pscene.gscene.joint_names))
                        print("non-direct reaching: {} ->{}".format(parent_sidx, new_node))
                        if not all([len(abds) > 0 for abds in available_binding_dict.values()]):
                            print("============== Non-available transition: in extending =====================")
                            self.reserved_attempt = False
                            sample_fail = True
                        else:
                            self.reserved_attempt = True
                            to_state, redundancy_dict_new = self.pscene.sample_leaf_state(from_state,
                                                                                          available_binding_dict,
                                                                                          new_node)
                            redundancy_dict = deepcopy(parent_snode.redundancy_dict) or {}
                            redundancy_dict.update(redundancy_dict_new)
                            sample_fail = False
            #                     except:
            #                         self.reserved_attempt = False
            #                         pass
            if not self.reserved_attempt:
                self.reset_trees()
                if self.flag_swap:
                    if ((not self.bool_forward) and len(self.visited_snodes_rev)>0):
                        self.swap_trees()
                    self.bool_forward = not self.bool_forward
                new_node = random.choice(self.neighbor_nodes_cur.keys())
                parent_nodes = self.node_parent_dict[new_node]
                parent_node = random.choice(list(parent_nodes.intersection(self.node_snode_dict_cur.keys())))
                parent_sidx = random.choice(self.node_snode_dict_cur[parent_node])
                parent_snode = self.snode_dict[parent_sidx]
                from_state = parent_snode.state
                available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                                                  list2dict(
                                                                                                      from_state.Q,
                                                                                                      self.pscene.gscene.joint_names))
                if not all([len(abds) > 0 for abds in available_binding_dict.values()]):
                    print("============== Non-available transition: sample again =====================")
                    sample_fail = True
                    continue
                to_state, redundancy_dict_new = self.pscene.sample_leaf_state(from_state, available_binding_dict,
                                                                              new_node)
                redundancy_dict = deepcopy(parent_snode.redundancy_dict) or {}
                redundancy_dict.update(redundancy_dict_new)
                sample_fail = False
        return parent_snode, from_state, to_state, redundancy_dict, sample_fail

    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        ret = False
        if connection_result:
            with self.snode_dict_lock:
                ## add to tree
                if snode_new.idx not in self.visited_snodes_cur:
                    self.visited_snodes_cur[snode_new.idx] = None

                for leaf in self.node_dict[snode_new.state.node]:
                    self.neighbor_nodes_cur[leaf] = None

                snode_count = self.snode_counter.value
                if snode_new.state.node not in self.node_snode_dict_cur:
                    self.node_snode_dict_cur[snode_new.state.node] = [snode_new.idx]
                else:
                    self.node_snode_dict_cur[snode_new.state.node] = self.node_snode_dict_cur[snode_new.state.node] + [
                        snode_new.idx]

            ## get hashable param dict
            param_flat_dict = {}
            for i_obj, oname in enumerate(self.pscene.subject_name_list):
                subject = self.pscene.subject_dict[oname]
                if isinstance(subject, AbstractObject):
                    link_name, param = snode_new.state.state_param[oname]
                    param_flat = (snode_new.state.node[i_obj], link_name) + tuple(
                        np.round(param, 4).flatten())  ## make state params hashable by flattenning
                else:
                    param = snode_new.state.state_param[oname]
                    param_flat = (snode_new.state.node[i_obj],) + tuple(
                        np.round(param, 4).flatten())  ## make state params hashable by flattenning
                param_flat_dict[oname] = param_flat

            # update param_snode_dict_cur
            param_match_dict_cur = {}
            for oname, param_flat in param_flat_dict.items():
                if param_flat in self.param_snode_dict_cur[oname]:
                    param_match_dict_cur[oname] = self.param_snode_dict_cur[oname][param_flat]
                else:
                    param_match_dict_cur[oname] = set()
                self.param_snode_dict_cur[oname][param_flat] = param_match_dict_cur[oname].union([snode_new.idx])

            subj_num = len(self.pscene.subject_name_list)
            if snode_src is not None:
                if len(snode_src.parents) > 0 and snode_src.parents[0] == 0:
                    ## sample new goal
                    from_state = snode_src.state
                    node_src = snode_src.state.node
                    Qnew = self.initial_state.Q
                    Qdict = list2dict(Qnew, self.pscene.gscene.joint_names)
                    for goal in self.goal_nodes:
                        sample_fail = True
                        available_binding_dict, transition_count = self.pscene.get_available_binding_dict(
                            from_state, goal, Qdict)

                        trial_count = 0
                        while sample_fail and trial_count < self.goal_trial_count:
                            trial_count += 1
                            to_state, redundancy_dict = self.pscene.sample_leaf_state(from_state,
                                                                                      available_binding_dict,
                                                                                      goal)
                            for i_s, ntem_s, ntem_g in zip(range(subj_num), node_src, goal):
                                if ntem_s != ntem_g:
                                    _oname, _hname, _bname, _bgname = to_state.binding_state[i_s]
                                    subject = self.pscene.subject_dict[_oname]
                                    if isinstance(subject, AbstractObject):
                                        to_ap = subject.action_points_dict[_hname]
                                        to_binder = self.pscene.actor_dict[_bname]
                                        redundancy = redundancy_dict[_oname]
                                        point_add_handle, rpy_add_handle = calc_redundancy(redundancy[to_ap.name],
                                                                                           to_ap)
                                        point_add_actor, rpy_add_actor = calc_redundancy(redundancy[to_binder.name],
                                                                                         to_binder)

                                        T_handle_oh = np.matmul(to_ap.Toff_oh,
                                                                SE3(Rot_rpy(rpy_add_handle), point_add_handle))
                                        T_actor_lh = np.matmul(to_binder.Toff_lh,
                                                               SE3(Rot_rpy(rpy_add_actor), point_add_actor))
                                        T_lo = np.matmul(T_actor_lh, SE3_inv(T_handle_oh))

                                        to_state.state_param[_oname] = (
                                        self.pscene.gscene.NAME_DICT[_bgname].link_name, T_lo)

                                        redundancy_values = {}
                                        redundancy_values[(_oname, to_ap.name)] = point_add_handle, rpy_add_handle
                                        redundancy_values[(_oname, to_binder.name)] = point_add_actor, rpy_add_actor
                                        print(redundancy_values)
                                        sample_fail = not self.gcheck.check(to_binder, subject, to_ap,
                                                                            redundancy_values, Qdict
                                                                            , obj_only=False)
                                        if sample_fail:
                                            break
                                    elif isinstance(subject, AbstractTask):
                                        raise (NotImplementedError("Reverse direction Task is not implemented"))
                                        to_state.state_param[_oname] = random.choice(
                                            subject.get_corresponding_params(ntem_g))
                        if sample_fail:
                            break
                        if not sample_fail:
                            goal_state_new = State(to_state.binding_state, to_state.state_param, self.initial_state.Q,
                                                   self.pscene)
                            goal_snode_new = self.connect(None, self.make_search_node(None, goal_state_new, None,
                                                                                      redundancy_dict))
                            tree_cur, tree_tar = self.get_trees()
                            self.swap_trees()
                            self.update(None, goal_snode_new, True)
                            self.set_trees(tree_cur, tree_tar)
                            print("Goal sample success")
                        else:
                            print("Goal sample fail")

                ## check goal (custom)
                if (len(snode_new.parents) > 0
                        and snode_new.parents[0] == 0
                        and self.check_goal(snode_new.state)):
                    ret = True
                elif (self.target_sidx >= 0):  ## This node is in connecting stream
                    snode_tar = self.snode_dict[self.target_sidx]
                    assert len(
                        snode_tar.parents) > 0, "This case should've checked as goal in the previous if statement"
                    self.attempt_reseved.put((snode_new.idx,  ## reserve connection from target to current snode
                                              snode_tar.parents[-1],
                                              (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                               self.node_snode_dict_cur),
                                              (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                               self.node_snode_dict_tar),
                                              True  ## directly reachable
                                              ))
                elif not self.reserved_attempt:  ## Add reaching-from-target attempt
                    if (len(snode_new.parents) > 0 and snode_new.parents[0] == 0):
                        for gnode in self.goal_nodes:
                            if snode_new.state.node in self.node_parent_dict[gnode]:
                                print("=============== try reaching goal =================")
                                self.attempt_reseved.put((snode_new.idx, gnode,
                                                          (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                                           self.node_snode_dict_cur),
                                                          (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                                           self.node_snode_dict_tar),
                                                          False  ## non-directly reachable
                                                          ))
                    else:
                        # get param_match_dict_tar
                        param_match_dict_tar = {}
                        for oname, param_flat in param_flat_dict.items():
                            if param_flat in self.param_snode_dict_tar[oname]:
                                param_match_dict_tar[oname] = self.param_snode_dict_tar[oname][param_flat]
                            else:
                                param_match_dict_tar[oname] = set()
                        with self.snode_dict_lock:
                            cur_tree_keys = np.array(sorted(self.visited_snodes_cur.keys()))
                            subj_num = len(self.pscene.subject_name_list)
                        snode_count = self.snode_counter.value
                        match_mat = np.zeros((subj_num, snode_count),
                                             dtype=np.int)  ## matrix representation of matching snode indices for each subject.
                        match_mat[:, cur_tree_keys] = -1
                        for i, oname in enumerate(self.pscene.subject_name_list):
                            match_mat[i, sorted(param_match_dict_tar[oname])] = True
                        match_counts = np.sum(match_mat, axis=0)
                        max_match = np.maximum(0, np.max(match_counts))
                        max_match_sidxes = np.where(match_counts == max_match)[0]
                        if len(max_match_sidxes) == 0:
                            print(
                                "Theoretically this should not happen after first goal is added, if this error occurs, sample new goal here")
                        else:
                            sidx_tar = random.choice(max_match_sidxes)
                            snode_tar = self.snode_dict[sidx_tar]
                            node_tar = snode_tar.state.node
                            node_src = snode_new.state.node
                            # print("node_src: {} -> node_tar: {}".format(node_src, node_tar))
                            if max_match == subj_num - 1 and node_src in self.node_dict[node_tar]:
                                # print("node_src in self.node_dict[node_tar]")
                                if len(snode_new.parents) > 0 and snode_new.parents[
                                    0] == 0:  # snode_new is from initial state
                                    self.attempt_reseved.put(
                                        (snode_new.idx,  ## reserve connection from current snode to target
                                         sidx_tar,
                                         (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                          self.node_snode_dict_cur),
                                         (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                          self.node_snode_dict_tar),
                                         True  ## directly reachable
                                         ))
                                else:  # snode_new is not from initial state
                                    self.attempt_reseved.put((sidx_tar,  ## reserve connection from target to current snode
                                                              snode_new.idx,
                                                              (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                                               self.node_snode_dict_tar),
                                                              (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                                               self.node_snode_dict_cur),
                                                              True  ## directly reachable
                                                              ))
                            else:
                                # print("node_src not in self.node_dict[node_tar]")
                                match_vec = match_mat[:, sidx_tar]
                                nomatch_idxes = np.where(np.logical_not(match_vec))[0]
                                nomatch_idx = random.choice(nomatch_idxes)
                                node_new = list(deepcopy(node_tar))
                                extend_subject = self.pscene.subject_dict[self.pscene.subject_name_list[nomatch_idx]]
                                node_new[nomatch_idx] = random.choice(  ## sample switch with no-matching subject
                                    extend_subject.get_neighbor_node_component_list(node_tar[nomatch_idx], self.pscene))
                                node_new = tuple(node_new)
                                self.attempt_reseved.put((sidx_tar,  ## reserve connection from target to sampled node
                                                          node_new,
                                                          (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                                           self.node_snode_dict_tar),
                                                          (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                                           self.node_snode_dict_cur),
                                                          False  ## non-directly reachable
                                                          ))
        return ret

    ## @brief set trees in forward direction
    def reset_trees(self):
        self.visited_snodes_cur = self.visited_snodes
        self.neighbor_nodes_cur = self.neighbor_nodes
        self.node_snode_dict_cur = self.node_snode_dict
        self.param_snode_dict_cur = self.param_snode_dict

        self.visited_snodes_tar = self.visited_snodes_rev
        self.neighbor_nodes_tar = self.neighbor_nodes_rev
        self.node_snode_dict_tar = self.node_snode_dict_rev
        self.param_snode_dict_tar = self.param_snode_dict_rev

    ## @brief set trees to given direction
    def set_trees(self, tree_cur, tree_tar):
        self.visited_snodes_cur, self.neighbor_nodes_cur, self.node_snode_dict_cur, self.param_snode_dict_cur = tree_cur
        self.visited_snodes_tar, self.neighbor_nodes_tar, self.node_snode_dict_tar, self.param_snode_dict_tar = tree_tar

    ## @brief get current trees
    def get_trees(self):
        return ((self.visited_snodes_cur, self.neighbor_nodes_cur, self.node_snode_dict_cur, self.param_snode_dict_cur),
                (self.visited_snodes_tar, self.neighbor_nodes_tar, self.node_snode_dict_tar, self.param_snode_dict_tar))

    ## @brief swap two trees
    def swap_trees(self):
        self.visited_snodes_tmp = self.visited_snodes_tar
        self.neighbor_nodes_tmp = self.neighbor_nodes_tar
        self.node_snode_dict_tmp = self.node_snode_dict_tar
        self.param_snode_dict_tmp = self.param_snode_dict_tar

        self.visited_snodes_tar = self.visited_snodes_cur
        self.neighbor_nodes_tar = self.neighbor_nodes_cur
        self.node_snode_dict_tar = self.node_snode_dict_cur
        self.param_snode_dict_tar = self.param_snode_dict_cur

        self.visited_snodes_cur = self.visited_snodes_tmp
        self.neighbor_nodes_cur = self.neighbor_nodes_tmp
        self.node_snode_dict_cur = self.node_snode_dict_tmp
        self.param_snode_dict_cur = self.param_snode_dict_tmp

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes