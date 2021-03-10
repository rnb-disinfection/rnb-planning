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
    def __init__(self, pscene, end_link_couple_dict, goal_trial_count=3):
        TaskInterface.__init__(self, pscene)
        self.gcheck = GraspChecker(pscene=pscene, end_link_couple_dict=end_link_couple_dict)
        self.goal_trial_count = goal_trial_count

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

    ##
    # @brief prepare memory variables
    # @param multiprocess_manager multiprocess_mananger instance if multiprocessing is used
    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.snode_dict_lock = multiprocess_manager.Lock()
            self.param_snode_dict = {k: multiprocess_manager.dict() for k in self.pscene.subject_name_list}

            self.node_snode_dict = multiprocess_manager.dict()
            self.visited_snodes = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.neighbor_nodes = multiprocess_manager.dict()

            self.node_snode_dict_rev = multiprocess_manager.dict()
            self.visited_snodes_rev = multiprocess_manager.dict()  # keys of dict is used as set, as set is not in multiprocess
            self.neighbor_nodes_rev = multiprocess_manager.dict()

            self.connected_snode_pairs = multiprocess_manager.list()

            self.attempt_reseved = multiprocess_manager.Queue()
            self.reserve_lock = multiprocess_manager.Lock()
        else:
            self.snode_dict_lock = DummyBlock()
            self.param_snode_dict = {k: dict() for k in self.pscene.subject_name_list}

            self.node_snode_dict = dict()
            self.visited_snodes = dict()
            self.neighbor_nodes = dict()

            self.node_snode_dict_rev = dict()
            self.visited_snodes_rev = dict()
            self.neighbor_nodes_rev = dict()

            self.connected_snode_pairs = list()

            self.attempt_reseved = Queue()
            self.reserve_lock = DummyBlock()

        self.set_tree()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, tree_margin=None, depth_margin=None):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes

        snode_root = self.make_search_node(None, initial_state, None, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)

    ##
    # @brief sample new state
    # @param lock lock instance to multiprocess
    def sample(self):
        sample_fail = True
        while sample_fail:
            self.target_sidx = -1 # save target sidx
            get_reserved = False
            with self.reserve_lock:
                if not self.attempt_reseved.empty():
                    try:
                        parent_sidx, new_item, tree_cur, tree_tar, direct = self.attempt_reseved.get(timeout=0.1)
                        parent_snode = self.snode_dict[parent_sidx]
                        from_state = parent_snode.state
                        self.visited_snodes_cur, self.neighbor_nodes_cur, self.node_snode_dict_cur = tree_cur
                        self.visited_snodes_cur, self.neighbor_nodes_tar, self.node_snode_dict_tar = tree_tar
                        if direct:
                            self.target_sidx = new_item
                            snode_new = self.snode_dict[self.target_sidx]
                            to_state = snode_new.state
                            redundancy_dict = deepcopy(snode_new.redundancy_dict)
                            get_reserved = True
                            sample_fail = False
                        else:
                            new_node = new_item
                            available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                                            list2dict(from_state.Q, self.pscene.gscene.joint_names))
                            if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                                print("============== Non-available transition: in extending =====================")
                                get_reserved = False
                            else:
                                get_reserved = True
                                to_state, redundancy_dict_new = self.pscene.sample_leaf_state(from_state, available_binding_dict, new_node)
                                redundancy_dict = deepcopy(parent_snode.redundancy_dict)
                                redundancy_dict.update_dict(redundancy_dict_new)
                                sample_fail = False
                    except:
                        get_reserved = False
                        pass
            if not get_reserved:
                new_node = random.choice(self.neighbor_nodes_cur.keys())
                parent_nodes = self.node_parent_dict[new_node]
                parent_node = random.choice(list(parent_nodes.intersection(self.node_snode_dict_cur.keys())))
                parent_sidx = random.choice(self.node_snode_dict_cur[parent_node])
                parent_snode = self.snode_dict[parent_sidx]
                from_state = parent_snode.state
                available_binding_dict, transition_count = self.pscene.get_available_binding_dict(from_state, new_node,
                                                                                list2dict(from_state.Q, self.pscene.gscene.joint_names))
                if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                    print("============== Non-available transition: sample again =====================")
                    sample_fail = True
                    continue
                to_state, redundancy_dict_new = self.pscene.sample_leaf_state(from_state, available_binding_dict, new_node)
                redundancy_dict = deepcopy(parent_snode.redundancy_dict)
                redundancy_dict.update_dict(redundancy_dict_new)
                sample_fail = False
        return parent_snode, from_state, to_state, redundancy_dict, sample_fail

    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        ret = False
        if connection_result:
            ## sample new goal
            node_src = snode_src.state.node
            Qnew = snode_src.state.Q
            Qdict = list2dict(Qnew, self.pscene.gscene.joint_names)
            for goal in self.goal_nodes:
                binding_state_new = deepcopy(snode_src.state.binding_state)
                state_param_new = deepcopy(snode_src.state.state_param)

                for oname, ntem_s, ntem_g in zip(self.pscene.subject_name_list, node_src, goal):
                    subject = self.pscene.subject_dict[oname]
                    redundancy_dict = deepcopy(snode_src.redundancy_dict)
                    if ntem_s != ntem_g:  # sample state param for node item for which goal is different from src
                        if isinstance(subject, AbstractObject):
                            bname = random.choice(self.geometry_actor_dict[ntem_g])
                            to_binder = self.pscene.actor_dict[bname]
                            to_ap = random.choice(self.pscene.subject_dict[oname].action_points_dict.values())
                            redundancy_tot = combine_redundancy(to_ap, to_binder)
                            sample_fail = True
                            trial_count = 0
                            while sample_fail and trial_count < self.goal_trial_count:
                                trial_count+=1
                                redundancy = sample_redundancy(redundancy_tot)
                                redundancy_dict[oname] = redundancy
                                point_add_handle, rpy_add_handle = calc_redundancy(redundancy[to_ap.name], to_ap)
                                point_add_actor, rpy_add_actor = calc_redundancy(redundancy[to_binder.name], to_binder)
                                redundancy_values = {}
                                redundancy_values[(oname, to_ap.name)] = point_add_handle, rpy_add_handle
                                redundancy_values[(oname, to_binder.name)] = point_add_actor, rpy_add_actor
                                sample_fail = not self.gcheck.check(to_binder, subject, to_ap, redundancy_values, Qdict
                                                                    , obj_only=False)
                            T_handle_oh = np.matmul(to_ap.Toff_oh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
                            T_actor_lh = np.matmul(to_binder.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
                            T_lo = np.matmul(T_actor_lh, SE3_inv(T_handle_oh))
                            state_param_new[oname] = (to_binder.geometry.link_name, T_lo)
                            binding_state_new = tuple((oname, to_ap.name, to_binder.name, to_binder.geometry.name)
                                                      if binding[0]==oname
                                                      else binding for binding in binding_state_new)
                        if isinstance(subject, AbstractTask):
                            raise(NotImplementedError("Reverse direction Task is not implemented"))
                            state_param_new[oname] = random.choice(subject.get_corresponding_params(ntem_g))
                    if sample_fail:
                        break
                if not sample_fail:
                    goal_state_new = State(binding_state_new, state_param_new, snode_src.state.Q, self.pscene)
                    self.connect(None, self.make_search_node(None, goal_state_new, None,  redundancy_dict))

            if snode_new.idx not in self.visited_snodes_cur:
                self.visited_snodes_cur[snode_new.state.node] = None

            for leaf in self.node_dict[snode_new.state.node]:
                self.neighbor_nodes_cur[leaf] = None

            param_flat_dict = {}
            for i_obj, oname in enumerate(self.pscene.subject_name_list):
                subject = self.pscene.subject_dict[oname]
                if isinstance(subject, AbstractObject):
                    link_name, param = snode_new.state_param[oname]
                    param_flat = (snode_new.state.node[i_obj], link_name) + tuple(np.round(param, 4).flatten()) ## make state params hashable by flattenning
                else:
                    param = snode_new.state_param[oname]
                    param_flat = (snode_new.state.node[i_obj], ) + tuple(np.round(param, 4).flatten()) ## make state params hashable by flattenning
                param_flat_dict[oname] = param_flat

            with self.snode_dict_lock:
                snode_count = self.snode_counter
                if snode_new.state.node not in self.node_snode_dict_cur:
                    self.node_snode_dict_cur[snode_new.state.node] = [snode_new.idx]
                else:
                    self.node_snode_dict_cur[snode_new.state.node] = self.node_snode_dict_cur[snode_new.state.node]+[snode_new.idx]

                param_match_dict = {}
                for oname, param_flat in param_flat_dict.items():
                    if param_flat in self.param_snode_dict[oname]:
                        param_match_dict[oname] = self.param_snode_dict[oname][param_flat]
                    else:
                        param_match_dict[oname] = set()
                    self.param_snode_dict[oname][param_flat] = param_match_dict[oname].union([snode_new.idx])

                if self.target_sidx>=0:
                    self.connected_snode_pairs.append((snode_new.idx, self.target_sidx))

            ## check goal (custom)
            if self.target_sidx>=0 or self.check_goal(snode_new.state): ## This node is foliated from target tree
                ret = True
            else:
                subj_num = len(self.pscene.subject_name_list)
                match_mat = np.zeros((subj_num, snode_count), dtype=np.bool) ## matrix representation of matching snode indices for each subject.
                for i, oname in enumerate(self.subject_name_list):
                    match_mat[i, sorted(param_match_dict[oname])] = True
                match_counts = np.sum(match_mat, axis=0)
                max_match_sidxes = np.where(match_counts==np.max(match_counts))[0]
                if len(max_match_sidxes)==0:
                    raise(NotImplementedError("Theoretically impossible, if this error occurs, sample new goal here"))
                else:
                    sidx_tar = random.choice(max_match_sidxes)
                    snode_tar = self.snode_dict[sidx_tar]
                    node_tar = snode_tar.state.node
                    node_src = snode_new.state.node
                    if node_src in self.node_dict[node_tar]:
                        self.attempt_reseved.put((sidx_tar,         ## reserve connection from target to current snode
                                                  snode_new.idx,
                                                  (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                                   self.node_snode_dict_tar),
                                                  (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                                   self.node_snode_dict_cur),
                                                  True ## directly reachable
                                                  ))
                    else:
                        match_vec = match_mat[sidx_tar]
                        nomatch_idxes = np.where(np.ogical_not(match_vec))[0]
                        nomatch_idx = random.choice(nomatch_idxes)
                        node_new = deepcopy(node_tar)
                        extend_subject = self.pscene.subject_type_list[nomatch_idx]
                        node_new[nomatch_idx] = random.choice(      ## sample switch with no-matching subject
                            extend_subject.get_neighbor_node_component_list(node_tar[nomatch_idx], self.pscene))
                        self.attempt_reseved.put((sidx_tar,         ## reserve connection from target to sampled node
                                                  node_new,
                                                  (self.visited_snodes_tar, self.neighbor_nodes_tar,
                                                   self.node_snode_dict_tar),
                                                  (self.visited_snodes_cur, self.neighbor_nodes_cur,
                                                   self.node_snode_dict_cur),
                                                  False ## non-directly reachable
                                                  ))
        self.set_tree()
        # self.swap_tree()
        return ret

    ## @brief set trees in forward direction
    def set_tree(self):
        self.visited_snodes_cur = self.visited_snodes
        self.neighbor_nodes_cur = self.neighbor_nodes
        self.node_snode_dict_cur = self.node_snode_dict

        self.visited_snodes_tar = self.visited_snodes_rev
        self.neighbor_nodes_tar = self.neighbor_nodes_rev
        self.node_snode_dict_tar = self.node_snode_dict_rev

    ## @brief swap two trees
    def swap_tree(self):
        self.visited_snodes_tmp = self.visited_snodes_tar
        self.neighbor_nodes_tmp = self.neighbor_nodes_tar
        self.node_snode_dict_tmp = self.node_snode_dict_tar

        self.visited_snodes_tar = self.visited_snodes_cur
        self.neighbor_nodes_tar = self.neighbor_nodes_cur
        self.node_snode_dict_tar = self.node_snode_dict_cur

        self.visited_snodes_cur = self.visited_snodes_tmp
        self.neighbor_nodes_cur = self.neighbor_nodes_tmp
        self.node_snode_dict_cur = self.node_snode_dict_tmp

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes

    ##
    # @brief find all schedules
    # @return list of SearchNode index list
    def find_schedules(self):
        schedule_dict = TaskInterface.find_schedules(self)
        for pair in self.connected_snode_pairs:
            snodeA = self.snode_dict[pair[0]]
            snodeB = self.snode_dict[pair[1]]
            if snodeA.parents[0] == 0: # snodeA is connected to source
                snode_src = snodeA
                snode_tar = snodeB
            else: # snodeB is connected to source
                snode_src = snodeB
                snode_tar = snodeA
            assert snode_src.state.node == snode_tar.state.node
            seq_tar = reversed(snode_tar.parents + [snode_tar.idx]) ## reversed snode sequence in target tree
            snode_pre = snode_src
            for i_s1, i_s2 in zip(seq_tar[:-1], seq_tar[1:]):
                snode_new_src = self.snode_dict[i_s1]
                snode_new_tar = self.snode_dict[i_s2]
                snode_new = self.make_search_node(snode_new_src, snode_new_tar.state, list(reversed(snode_new_src.traj)),  snode_new_tar.redundancy_dict)
                snode_new = self.connect(snode_pre, snode_new)
                snode_pre = snode_new
            schedule_dict[snode_new.idx] = snode_new.parents + [snode_new.idx]
        return schedule_dict