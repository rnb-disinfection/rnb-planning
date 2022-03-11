from .pipeline import *
from .task.interface import *
from ..utils.utils_graph import *
from ..utils.utils import *
from ..utils.rotation_utils import *
from ..controller.combined_robot import RobotSpecs
from .constraint.constraint_subject import AbstractTask
from copy import deepcopy
import random

HOLD_DEBUG = False


def get_gtem_list_except(pscene, snames):
    gtem_dict = {sname: [pscene.gscene.NAME_DICT[gname]
                         for gname in subject.geometry.get_family()
                         if (pscene.gscene.NAME_DICT[gname].collision and
                             not pscene.gscene.NAME_DICT[gname].fixed)
                         ]
                 for sname, subject in pscene.subject_dict.items()}

    gtem_list = []
    for sname in pscene.subject_name_list:
        if sname not in snames:
            gtem_list += gtem_dict[sname]
    return gtem_list


class Transition:
    def __init__(self, node, rho):
        self.traj = None
        self.rho = rho
        self.node = deepcopy(node)

    def set_traj(self, traj):
        self.traj = traj


##
# @class BindingTransition
# @remark set btf=None, state_param=None to sample states when possible
class BindingTransition(Transition):
    def __init__(self, subject_name, btf, state_param, **kwargs):
        Transition.__init__(self, **kwargs)
        self.subject_name = subject_name
        self.btf, self.state_param = btf, state_param


class JointTransition(Transition):
    def __init__(self, pscene, rname, Q, **kwargs):
        Transition.__init__(self, **kwargs)
        self.rname = rname
        self.Q = np.copy(Q)
        self.idc_joints = pscene.combined_robot.idx_dict[rname]


##
# @class    IncrementalSearch
class IncrementalSearch(TaskInterface, PlanningPipeline):
    ##
    # @param    pscene              rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param    allow_joint_motion  allow random joint motion, which is transition to same node. \r\n
    # @param    config_gen          configuration generator for random joint motion. by default, this homing motion is generated.
    # @param    rho                 default priority
    # @param    gamma               decay ratio
    def __init__(self, pscene, gcheck, rcheck, allow_joint_motion=False, config_gen=None,
                 explicit_rule=None, rho=0.5, gamma=0.5):
        TaskInterface.__init__(self, pscene)
        PlanningPipeline.__init__(self, pscene)

        self.gcheck, self.rcheck = gcheck, rcheck
        self.allow_joint_motion = allow_joint_motion
        self.config_gen = config_gen
        self.explicit_rule = (lambda pscene, node, leaf: True) if explicit_rule is None else explicit_rule
        self.rho = rho
        self.gamma = gamma
        self.set_task_planner(self)

    ##
    # @brief build object-level node graph
    def prepare(self):
        pscene = self.pscene

        # make all node connections
        self.node_list = pscene.get_all_nodes()
        self.node_dict_full = {k: [k] if self.allow_joint_motion else [] for k in self.node_list}
        self.node_parent_dict_full = {k: [k] if self.allow_joint_motion else [] for k in self.node_list}
        for node in self.node_list:
            for leaf in pscene.get_node_neighbor(node):
                if leaf in self.node_list:
                    self.node_dict_full[node].append(leaf)
                    self.node_parent_dict_full[leaf].append(node)
        for node in self.node_list:
            self.node_dict_full[node] = set(self.node_dict_full[node])
            self.node_parent_dict_full[node] = set(self.node_parent_dict_full[node])

    def initialize_memory(self, multiprocess_manager):
        TaskInterface.initialize_memory(self, multiprocess_manager)
        if multiprocess_manager is not None:
            self.transition_queue = multiprocess_manager.PriorityQueue()
            self.queue_lock = multiprocess_manager.Lock()
        else:
            self.transition_queue = PriorityQueue()
            self.queue_lock = DummyBlock()

    ##
    # @brief calculate initial/goal scores and filter valid nodes
    def init_search(self, initial_state, goal_nodes, **kwargs):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes

        self.node_dict = {}
        self.node_parent_dict = defaultdict(set)
        for node, leafs in self.node_dict_full.items():
            ## goal node does not have child leaf
            if node in goal_nodes:
                self.node_dict[node] = set()
                continue
            leaf_list = [leaf
                         for leaf in leafs
                         if self.explicit_rule(self.pscene, node, leaf)]
            self.node_dict[node] = set(leaf_list)
            for leaf in self.node_dict[node]:
                self.node_parent_dict[leaf].add(node)
        self.node_parent_dict = dict(self.node_parent_dict)

        self.error_dict = score_graph(self.node_parent_dict, goal_nodes)
        self.cost_dict = score_graph(self.node_dict, initial_state.node)
        nodes_valid = sorted(set(self.cost_dict.keys()).intersection(self.error_dict.keys()))
        self.score_dict = {}
        for node in nodes_valid:
            self.score_dict[node] = self.error_dict[node] + self.cost_dict[node]

        snode_root = self.make_search_node(None, initial_state, None)
        self.connect(None, snode_root)
        self.update(None, snode_root, True)
        self.queue_transitions_all(snode_root)

    ##
    # @brief (prototype) update connection result to the searchng algorithm
    def update(self, snode_src, snode_new, connection_result):
        if connection_result:
            if self.check_goal(snode_new.state):
                print("Goal reached")
                return True
        return False

    def get_transited_state(self, state, transition):
        to_state = state.copy(self.pscene)
        if isinstance(transition, BindingTransition):
            to_state.binding_state[transition.subject_name] = transition.btf
            to_state.state_param[transition.subject_name] = transition.state_param
            to_state.set_binding_state(self.pscene, to_state.binding_state, to_state.state_param)
        elif isinstance(transition, JointTransition):
            to_state.Q[transition.idc_joints] = transition.Q
        else:
            raise (NotImplementedError("non-implemented transition type {}".format(transition.__class__.__name__)))
        return to_state

    def calc_priority(self, snode_from, transitions):
        to_node = transitions[-1].node
        rho = 1.0
        for transition in transitions:
            rho *= transition.rho
        return - rho * (self.rho ** self.error_dict[to_node])

    ##
    # @param snode_from  SearchNode from which transitoin will be made
    # @param transition  BindingTransform or JointTransition
    def queue_transition(self, snode_from, transitions):
        if isinstance(transitions[0], BindingTransition):
            assert transitions[0].btf != None, "here"
        self.transition_queue.put((self.calc_priority(snode_from, transitions),
                                   (snode_from.idx, deepcopy(transitions))))

    ##
    # @brief sample leaf states for all available binding changes
    def queue_transitions_all(self, snode_from, leafs=None, reserveds=[], snames=None,
                              constraint_fun=None, sample_all=False, rho=None):
        transitions = self.sample_transitions(snode_from.state, leafs, snames=snames,
                                              constraint_fun=constraint_fun, sample_all=sample_all, rho=rho)

        for bts in transitions:
            self.queue_transition(snode_from, [bts] + reserveds)

    def sample_transitions(self, from_state, leafs=None, snames=None,
                           constraint_fun=None, sample_all=False, rho=None):
        if rho is None:
            rho = self.rho
        if leafs is None:
            leafs = [leaf for leaf in self.node_dict[from_state.node] if leaf in self.error_dict]

        if snames is not None:
            leafs_s = []
            for sname in snames:  # pick if target subject is changed
                i_s = self.pscene.subject_name_list.index(sname)
                leafs_s += [leaf for leaf in leafs if leaf[i_s] != from_state.node[i_s]]
            leafs = leafs_s

        if leafs:
            bts_list = []
            error_min = np.min([self.error_dict[leaf] for leaf in leafs])
            leafs = [leaf for leaf in leafs if self.error_dict[leaf] == error_min]
            random.shuffle(leafs)

            for leaf in leafs:
                available_binding_dict = self.pscene.get_available_binding_dict(from_state, leaf)
                to_state = self.pscene.sample_leaf_state(from_state, available_binding_dict, leaf)
                snames, succ = self.pscene.get_changing_subjects(from_state, to_state)
                random.shuffle(snames)
                for sname in snames:
                    bts = BindingTransition(
                        sname, to_state.binding_state[sname], to_state.state_param[sname],
                        node=to_state.node, rho=rho)
                    if constraint_fun is not None and not constraint_fun(from_state, bts):
                        continue
                    if not sample_all:
                        return [bts]
                    bts_list.append(bts)
            return bts_list
        else:
            return []

    def sample_pick_put(self, snode_from, sname_list, display=False):
        state = snode_from.state
        Q_dict = list2dict(state.Q, self.pscene.gscene.joint_names)
        sname_list = deepcopy(sname_list)
        random.shuffle(sname_list)
        pick_put_all = []
        for sname in sname_list:
            subject = self.pscene.subject_dict[sname]
            btf = state.binding_state[sname]
            state_param = state.state_param[sname]
            pick_put = False
            subject.set_state(btf, state_param)
            bts_pick_list = self.sample_transitions(state, snames=[sname], sample_all=True)
            random.shuffle(bts_pick_list)
            for bts_pick in bts_pick_list:
                if self.gcheck.check(bts_pick.btf, Q_dict):
                    subject.set_state(bts_pick.btf, state_param)
                    bts_put_list = self.sample_transitions(state, snames=[sname], sample_all=True)
                    random.shuffle(bts_put_list)
                    for bts_put in bts_put_list:
                        if self.gcheck.check(bts_put.btf, Q_dict):
                            if display:
                                self.pscene.show_binding(bts_pick.btf)
                                self.pscene.show_binding(bts_put.btf)
                                time.sleep(0.01)
                                self.pscene.gscene.clear_highlight()
                            pick_put = [bts_pick, bts_put]
                            break
                    if pick_put:
                        break
            subject.set_state(btf, state_param)
            if pick_put:
                pick_put_all += pick_put
            else:
                # fail! just return
                return
        return pick_put_all

    def trace_back(self, snode_from, transitions):
        # trace back one edge
        transition = transitions[0]
        snode_next = snode_from
        snode_from = self.snode_dict[snode_from.parents[-1]]
        state = snode_next.state
        sname = transition.subject_name
        transition = BindingTransition(
            sname, state.binding_state[sname], state.state_param[sname],
            node=state.node, rho=transition.rho)
        transition.set_traj(snode_next.traj)
        transitions = [transition] + transitions
        return snode_from, transitions

    ##
    # @brief check if a state is in pre-defined goal nodes
    # @param state rnb-planning.src.pkg.planning.scene.State
    def check_goal(self, state):
        return state.node in self.goal_nodes

    def step_search(self, **kwargs):
        with self.queue_lock:
            if self.transition_queue.empty():
                TextColors.RED.println("[WARN] Transition Queue Empty")
            priority, (idx_from, transitions) = self.transition_queue.get()
        snode_from = self.snode_dict[idx_from]
        transition = transitions[0]
        #     print("=============== {} ===================".format(i_step))
        #     print("{}: {} -> {}".format(snode_from.idx, snode_from.state.node, transition.node))

        succ, snode_return = False, None
        self.pscene.set_object_state(snode_from.state)
        for resv in self.resolver_stack:
            reason = resv.check(snode_from.state, transitions[0], skip_set_state=True, **kwargs)
            if reason:
                snode_new = resv.resolve(snode_from, reason, transitions, **kwargs)
                if snode_new is not None and self.check_goal(snode_new.state):
                    succ, snode_return = True, snode_new
                    break

        # queue repeat
        if isinstance(transition, BindingTransition):
            if succ:
                rho = self.gamma * self.gamma * self.gamma * self.gamma * transition.rho # lower priority if already suceeded
            else:
                rho = self.gamma * self.gamma * transition.rho # same priority to 1 additional item moving
            self.queue_transitions_all(snode_from, snames=[transition.subject_name], reserveds=transitions[1:], rho=rho)
        return succ, snode_return

    def _search_loop(self, ID, N_search,
                     display=False, dt_vis=None, verbose=False, timeout_loop=600,
                     add_homing=True, post_optimize=False, home_pose=None, **kwargs):
        loop_counter = 0
        sample_fail_counter = 0
        sample_fail_max = 5
        no_queue_stop = False
        ret = False
        while (N_search is None or self.tplan.snode_counter.value < N_search) \
                and (time.time() - self.t0) < timeout_loop \
                and (self.stop_now.value < self.N_agents):
            loop_counter += 1
            ret, snode_new = self.step_search(verbose=verbose, display=display, dt_vis=dt_vis, **kwargs)
            if ret:
                if add_homing:
                    print("++ adding return motion to acquired answer ++")
                    home_state = self.tplan.snode_dict[0].copy(self.pscene)
                    home_state.Q = self.pscene.combined_robot.home_pose if home_pose is None else home_pose
                    if add_homing > 1:
                        self.add_return_motion(snode_new, try_count=add_homing, initial_state=home_state)
                    else:
                        self.add_return_motion(snode_new, initial_state=home_state)
                if post_optimize:
                    print("++ post-optimizing acquired answer ++")
                    self.post_optimize_schedule(snode_new)
                sol_count = self.solution_count.value + 1
                self.solution_count.value = sol_count
                if sol_count > self.max_solution_count:
                    break

        if no_queue_stop:
            self.stop_now.value = self.stop_now.value + 1
            term_reason = "node queue empty {}th".format(self.stop_now.value)
        elif N_search is not None and self.tplan.snode_counter.value >= N_search:
            term_reason = "max search node count reached ({}/{})".format(self.tplan.snode_counter.value, N_search)
            self.stop_now.value = self.N_agents
        elif (time.time() - self.t0) >= timeout_loop:
            term_reason = "max iteration time reached ({}/{} s)".format(round(time.time() - self.t0, 1),
                                                                        round(timeout_loop, 1))
            self.stop_now.value = self.N_agents
        elif ret:
            term_reason = "required answers acquired"
            self.stop_now.value = self.N_agents
        elif self.stop_now.value >= self.N_agents:
            term_reason = "Stop called from other agent"
        else:
            term_reason = "Unknown issue"
        print(
            "=========================================================================================================")
        print("======================= terminated {}: {}  ({}/{}) ===============================".format(
            ID, term_reason, round(time.time() - self.t0, 1), round(timeout_loop, 1)))
        print(
            "=========================================================================================================")

    def print_remaining_queue(self):
        sorted_elements = sorted(self.transition_queue.elements)
        idx_list = []
        for idx, item in enumerate(sorted_elements):
            score = item[0]
            idx_from = item[1][0]
            transitions = item[1][1]
            transition = transitions[0]
            print("{} ({}): {} -> {} ({})".format(
                idx, score, idx_from,
                (transition.subject_name, transition.node)
                if isinstance(transition, BindingTransition) else transition.rname,
                len(transitions)))


class ConstraintResolver:
    @abstractmethod
    def check(self, state, transition, skip_set_state=False, **kwargs):
        raise (NotImplementedError("check is not implemented for {}".format(self.__class__.__name__)))

    @abstractmethod
    def resolve(self, snode_from, reason, transitions, **kwargs):
        raise (NotImplementedError("resolve is not implemented for {}".format(self.__class__.__name__)))


class CollisionResolver(ConstraintResolver):
    ##
    # @param inc IncrementalSearch instance
    # @param gcheck GraspChecker
    def __init__(self, inc, gcheck):
        self.inc, self.gcheck = inc, gcheck
        self.pscene = inc.pscene

    def check(self, state, transition, skip_set_state=False, display=False, verbose=False, **kwargs):
        if not skip_set_state:
            self.pscene.set_object_state(state)

        if isinstance(transition, BindingTransition):
            Q_dict = list2dict(state.Q, self.pscene.gscene.joint_names)
            dist_dict = self.gcheck.check(transition.btf, Q_dict, return_dist_dict=True)
            if display:
                self.pscene.gscene.update_markers_all()
                self.pscene.show_binding(transition.btf)

            collision_set = set()
            for aname, adict in dist_dict.items():
                for oname, dist in adict.items():
                    if dist <= 1e-20:
                        collision_set.add(aname)
                        collision_set.add(oname)
                        if display:
                            self.pscene.gscene.highlight_geometry("hl_cresv", oname, color=(1, 0, 0, 0.5))
                            self.pscene.gscene.highlight_geometry("hl_cresv", aname, color=(0, 0, 1, 0.5))

            if display:
                self.pscene.gscene.clear_highlight()
            gtem_dict = {}
            col_tems = []
            for sname, subject in self.pscene.subject_dict.items():
                gfam = subject.geometry.get_family()
                gtem_dict[sname] = gfam
                if collision_set.intersection(gfam):
                    col_tems.append(sname)
            reason = col_tems
        elif isinstance(transition, JointTransition):
            reason = False
        else:
            raise (NotImplementedError("Undefined transition type {}".format(transition.__class__.__name__)))

        return reason

    def resolve(self, snode_from, reason, transitions, display=False, verbose=False, **kwargs):
        assert reason, "resolve is supposed to called only when there is failure reason"
        state = snode_from.state
        for sname in reason:
            if self.pscene.subject_dict[sname].geometry.fixed:
                # fixed collision is not resolvable - trace back!!
                return

        transition = transitions[0]
        if self.pscene.actor_robot_dict[transition.btf.chain.actor_name] == None:
            # this is placement. trace back to pick
            snode_from, transitions = self.inc.trace_back(snode_from, transitions)

        pick_put_all = self.inc.sample_pick_put(snode_from, reason, display=display)
        if pick_put_all is not None:
            self.inc.queue_transition(snode_from, pick_put_all + transitions)


class ReachResolver(ConstraintResolver):
    ##
    # @param inc IncrementalSearch instance
    # @param gcheck GraspChecker
    def __init__(self, inc, rcheck, mplan, floor=None, N_try_max=20):
        self.inc, self.rcheck, self.mplan = inc, rcheck, mplan
        self.pscene = inc.pscene
        self.crob = self.pscene.combined_robot
        self.rconfig_dict = self.crob.get_robot_config_dict()

        self.N_try_max = N_try_max
        self.floor = floor
        if floor is not None:
            xy_list = floor.get_vertice_radius_from(self.crob.home_dict)[0][:, :2]
            self.xy_max = np.max(xy_list, axis=0).tolist()
            self.xy_min = np.min(xy_list, axis=0).tolist()

    def check(self, state, transition, skip_set_state=False, display=False, verbose=False, **kwargs):
        if not skip_set_state:
            self.pscene.set_object_state(state)
        if isinstance(transition, BindingTransition):
            Q_dict = list2dict(state.Q, self.pscene.gscene.joint_names)
            if display:
                self.pscene.gscene.show_pose(state.Q)
                self.pscene.show_binding(transition.btf)
            success = self.rcheck.check(transition.btf, Q_dict)
            if display:
                self.pscene.gscene.clear_highlight()
        elif isinstance(transition, JointTransition):
            success = True
        else:
            raise (NotImplementedError("Undefined transition type {}".format(transition.__class__.__name__)))

        return not success

    def resolve(self, snode_from, reason, transitions, display=False, verbose=False, **kwargs):
        state = snode_from.state
        transition = transitions[0]
        assert reason, "resolve is supposed to called only when there is failure reason"
        if verbose: print("resolve reach from {}".format(np.round(state.Q, 2)))

        btf_cur = state.binding_state[transition.subject_name]
        ref_link = self.pscene.subject_dict[
            transition.subject_name].geometry.link_name  # assume new actor is active, get currently attached object link
        aname = transition.btf.chain.actor_name  # get new active actor
        T_loal = transition.btf.T_loal  # get Tf from static object to active actor

        subject = self.pscene.subject_dict[transition.subject_name]
        if subject.make_constraints(btf_cur.chain, transition.btf.chain):  # if target transition actor is not on robot
            return  # base sampling not  implemented with constrained motion

        if self.pscene.actor_robot_dict[aname] is None:  # if target transition actor is not on robot
            aname_to = aname
            aname = btf_cur.chain.actor_name
            T_loal = transition.btf.T_laol
            ref_link = transition.btf.actor_link
            # currently bound actor should be on robot
            assert aname in self.pscene.actor_robot_dict, \
                "no controllable actor in transition of {}: {} -> {}".format(transition.subject_name, aname, aname_to)

        rname = self.pscene.actor_robot_dict[aname]
        rconfig = self.rconfig_dict[rname]
        rbase = self.pscene.robot_chain_dict[rname]["link_names"][0]

        if rconfig.root_on == "base_link":
            if verbose: print("robot is not movable - just return")
            return

        mlink = rconfig.root_on
        Tmr = self.pscene.gscene.get_tf(rbase, state.Q, from_link=mlink)
        Trm = np.linalg.inv(Tmr)

        for mname, mchain in self.pscene.robot_chain_dict.items():
            if rconfig.root_on in mchain['link_names']:
                break
            else:
                mname = None
        assert mname is not None, "mobile robot not found for {} - root link={}".format(rname, rconfig.root_on)

        idc_robot = self.crob.idx_dict[rname]
        idc_mobile = self.crob.idx_dict[mname]
        theta_range = self.crob.get_joint_limits()[idc_mobile[2]]

        Q = np.copy(state.Q)
        home_transitions = []
        if not np.all(np.abs(self.crob.home_pose[idc_robot] - Q[idc_robot]) < 1e-2):  # if robot is not at home
            if verbose: print("home robot {}".format(state.node))
            home_transitions.append(JointTransition(
                self.pscene, rname, self.crob.home_pose[idc_robot],
                node=state.node, rho=self.inc.rho))  # move robot to home config
            Q[idc_robot] = self.crob.home_pose[idc_robot]

        snames_fixed = []
        for sname, subject in self.pscene.subject_dict.items(): # do not ignore task objects
            if isinstance(subject, AbstractTask):
                snames_fixed.append(sname)

        radii_max = np.linalg.norm(rconfig.xyzrpy[0][:2]) + RobotSpecs.get_shoulder_reach(rconfig.type)
        sample_count = 0
        ignorant_answer_done = False
        for _ in range(self.N_try_max):
            r_loc, theta_loc, theta = np.random.uniform([0, -np.pi, theta_range[0]], [radii_max, np.pi, theta_range[1]])
            xyz_lr = np.matmul(Rot_axis(3, theta_loc), [r_loc, 0, 0])[:3]
            xyz_br = T_loal[:3, 3] + xyz_lr
            Tbr_ = SE3(Rot_axis(3, theta), xyz_br)
            Tbm_ = np.matmul(Tbr_, Trm)
            xy_bm = Tbm_[:2, 3]
            theta_bm = Rot2axis(Tbm_[:3, :3], 3)
            xyt = np.array(xy_bm.tolist() + [theta_bm])
            Qmb = xyt.tolist() + [0] * 3

            if self.floor is not None and not (np.all(xy_bm > self.xy_min) and np.all(xy_bm < self.xy_max)):
                continue

            Q[idc_mobile] = Qmb
            Tbr = self.pscene.gscene.get_tf(rbase, Q, from_link=ref_link)
            Trb = np.linalg.inv(Tbr)
            Tre = np.matmul(Trb, T_loal)
            res = self.rcheck.check_T(rname, Tre)
            if res:
                if display:
                    Tbm = self.pscene.gscene.get_tf(mlink, Q, from_link=ref_link)
                    self.pscene.show_binding(transition.btf)
                    self.pscene.gscene.add_highlight_axis("hl", "t_loal", link_name=ref_link, T=T_loal,
                                                          dims=(0.3, 0.03, 0.03))
                    self.pscene.gscene.add_highlight_axis("hl", "tbr_", link_name=ref_link, T=Tbr_,
                                                          dims=(0.3, 0.03, 0.03))
                    self.pscene.gscene.add_highlight_axis("hl", "tbr", link_name=ref_link, T=Tbr,
                                                          dims=(0.3, 0.03, 0.03))
                    self.pscene.gscene.add_highlight_axis("hl", "tbm_", link_name=ref_link, T=Tbm_,
                                                          dims=(0.3, 0.03, 0.03))
                    self.pscene.gscene.add_highlight_axis("hl", "tbm", link_name=ref_link, T=Tbm,
                                                          dims=(0.3, 0.03, 0.03))
                    self.pscene.gscene.show_pose(Q)

                # check ignoring moving subjects first, than not ignoring any
                res = self.mplan.validate_trajectory(
                    [Q], ignore=get_gtem_list_except(self.pscene, snames_fixed) if not ignorant_answer_done else [])
                if res: # valid trajectory
                    if HOLD_DEBUG:
                        raw_input()
                    if verbose: print("move base {}".format(state.node))
                    transition_new = JointTransition(self.pscene, mname, Qmb, node=state.node, rho=self.inc.rho)
                    self.inc.queue_transition(snode_from, transitions=home_transitions + [transition_new] + transitions)

                    if ignorant_answer_done: # added is a non-ignorant answer - stop and return
                        if display:
                            self.pscene.gscene.clear_highlight()
                        return
                    else: # ignorant answer acquired - re-check with subject collision
                        ignorant_answer_done = True
                        res = self.mplan.validate_trajectory([Q])
                        if res: # added is valid again with subject collision - this finishes all. return
                            if display:
                                self.pscene.gscene.clear_highlight()
                            return

        if display:
            self.pscene.gscene.clear_highlight()
        if verbose: print("reach not resolved")


class MotionResolver(ConstraintResolver):
    ##
    # @param inc IncrementalSearch instance
    # @param gcheck GraspChecker
    def __init__(self, inc, mplan):
        self.inc, self.mplan = inc, mplan
        self.pscene = inc.pscene

    def check(self, state, transition, skip_set_state=False, display=False, verbose=False, dt_vis=0.01, **kwargs):
        if not skip_set_state:
            self.pscene.set_object_state(state)

        if isinstance(transition, BindingTransition):
            subject_list = [transition.subject_name]
        elif isinstance(transition, JointTransition):
            subject_list = []
        else:
            raise (NotImplementedError("Undefined transition type {}".format(transition.__class__.__name__)))

        to_state = self.inc.get_transited_state(state, transition)
        if display:
            self.pscene.gscene.show_pose(state.Q)
            if isinstance(transition, BindingTransition):
                self.pscene.show_binding(transition.btf)
            elif isinstance(transition, JointTransition):
                time.sleep(0.01)
                self.pscene.gscene.show_pose(to_state.Q)

        col_tems_all = deepcopy(subject_list)  # do not ignore target subject from begining

        for sname, subject in self.pscene.subject_dict.items(): # do not ignore task objects
            if isinstance(subject, AbstractTask):
                col_tems_all.append(sname)
        col_tems_all = sorted(set(col_tems_all))

        reason = []
        while True:
            subjects_remain = list(set(self.pscene.subject_name_list) - set(col_tems_all))

            # ignore all except previously collided subject
            gtems_ignore = get_gtem_list_except(self.pscene, col_tems_all)
            Traj, LastQ, error, success = self.mplan.plan_algorithm(
                state, to_state, subject_list, ignore=gtems_ignore, display=display, verbose=verbose, **kwargs)

            if not success:
                # If failed to find trajectory, end there
                break

            # get colliding items with the trajectory
            col_tems = []
            for sname in subjects_remain:
                # ignore all gtems except for sname to check collision
                ignore = get_gtem_list_except(self.pscene, [sname])
                res = self.mplan.validate_trajectory(Traj, ignore=ignore)
                if not res:
                    col_tems.append(sname)
            if display:
                for sname in col_tems:
                    gnames = self.pscene.subject_dict[sname].geometry.get_family()
                    for gname in gnames:
                        if self.pscene.gscene.NAME_DICT[gname].collision:
                            self.pscene.gscene.highlight_geometry("mresv", gname, color=(1, 1, 0, 0.5))
                self.pscene.gscene.show_motion(Traj, period=dt_vis)
                self.pscene.gscene.clear_highlight()
            reason.append((col_tems, Traj))
            col_tems_all += col_tems
            if not col_tems:
                # If no collision occurs, this is already an answer. no need to try planning more
                break

        return reason

    def resolve(self, snode_from, reason, transitions, display=False, verbose=False, dt_vis=0.01, **kwargs):
        state = snode_from.state
        subj_seq = [trs.subject_name for trs in transitions if isinstance(trs, BindingTransition)]
        assert reason, "resolve is supposed to called only when there is failure reason"
        snode_new = None

        transitions_pre = []
        transition = transitions[0]
        reserveds = transitions[1:]
        snode_next = snode_from
        if isinstance(transition, BindingTransition):
            if self.pscene.actor_robot_dict[transition.btf.chain.actor_name] == None:
                # this is placement. trace back to pick
                snode_from, transitions = self.inc.trace_back(snode_from, transitions)
                transitions_pre = [transitions[0]]

        for col_tems, Traj in reason:
            if not col_tems:  # validated trajectory
                to_state = self.inc.get_transited_state(snode_next.state, transition)
                to_state.Q = Traj[-1]
                snode_new = self.inc.make_search_node(snode_next, to_state, Traj)
                self.inc.connect(snode_next, snode_new)
                if display:
                    self.pscene.gscene.show_motion(Traj, period=dt_vis)
                if verbose:
                    print("valid transition found: {} - {}".format(snode_next.state.node, snode_new.state.node))
                if reserveds:
                    self.inc.queue_transition(snode_new, transitions=reserveds)  # que transitions to reach goal
                else:
                    self.inc.queue_transitions_all(snode_new)  # que transitions to reach goal
                continue

            transition_traj = deepcopy(transition)
            transition_traj.set_traj(Traj)
            random.shuffle(col_tems)
            for col_tem in col_tems:
                pick_put_all = self.inc.sample_pick_put(snode_from, [col_tem], display=display)
                if pick_put_all is not None:
                    self.inc.queue_transition(
                        snode_from, transitions=pick_put_all + transitions_pre + [transition_traj] + reserveds)
        return snode_new
