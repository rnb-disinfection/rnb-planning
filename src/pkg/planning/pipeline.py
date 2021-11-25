import time
from .scene import State
from ..utils.utils import SingleValue, DummyBlock, list2dict, differentiate, GlobalTimer, TextColors
from ..utils.joint_utils import apply_vel_acc_lims
from ..controller.trajectory_client.trajectory_client import DEFAULT_TRAJ_FREQUENCY, MultiTracker
from .task.interface import SearchNode
import numpy as np
from copy import copy

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

from multiprocessing import Process, cpu_count
from multiprocessing.managers import SyncManager
import random
class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)


##
# @class    PlanningPipeline
# @brief    planning pipeline
class PlanningPipeline:
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene):
        pscene.set_object_state(pscene.initialize_state(pscene.combined_robot.home_pose))
        ## @brief rnb-planning.src.pkg.planning.scene.PlanningScene
        self.pscene = pscene
        ## @brief rnb-planning.src.pkg.planning.motion.interface.MotionInterface
        self.mplan = None
        ## @brief rnb-planning.src.pkg.planning.task.interface.TaskInterface
        self.tplan = None
        ## @brief number of multiprocess agents
        self.N_agents = None
        ## @brief count search steps
        self.search_counter = None
        ## @brief flag to stop multiprocess search
        self.stop_now =  None
        ## @brief PriorityQueueManager
        self.execute_res = True
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.counter_lock = self.manager.Lock()
        self.gtimer = GlobalTimer.instance()
        self.constrained_motion_scale = 0.3

    ##
    # @param mplan subclass instance of rnb-planning.src.pkg.planning.motion.interface.MotionInterface
    def set_motion_planner(self, mplan):
        self.mplan = mplan
        mplan.update_gscene()

    ##
    # @param tplan subclass instance of rnb-planning.src.pkg.planning.task.interface.TaskInterface
    def set_task_planner(self, tplan):
        self.tplan = tplan
        self.tplan.prepare()

    ##
    # @brief update planners
    def update(self):
        if self.mplan:
            self.mplan.update_gscene()
        if self.tplan:
            self.tplan.prepare()

    ##
    # @brief run search algorithm
    # @param initial_state rnb-planning.src.pkg.planning.scene.State
    # @param goal_nodes list of goal nodes
    # @param multiprocess boolean flag for multiprocess search
    # @param N_redundant_sample number of redundancy sampling
    # @param terminate_on_first boolean flag for terminate on first answer
    # @param max_solution_count number of maximum solution count
    # @param N_search maximum number of search for each thread
    # @param timeout_loop search timeout default = 600 sec
    # @param N_agents number of multiprocess agents, Default = cpu_count
    # @param wait_proc wait for sub processes
    # @param display boolean flag for one-by-one motion display on rvia
    # @param dt_vis display period
    # @param verbose boolean flag for printing intermediate process
    def search(self, initial_state, goal_nodes, multiprocess=False,
               max_solution_count=100, N_search=None, N_agents=None, wait_proc=True,
               display=False, dt_vis=0.01, verbose=False, timeout_loop=600, looptime_extra=None,
               terminate_on_first=False, timeout=1, **kwargs):
        self.gtimer.tic("search_loop")
        if looptime_extra is None:
            looptime_extra = timeout

        if terminate_on_first:
            TextColors.RED.println("==========================================================")
            TextColors.RED.println("terminate_on_first is deprecated. Use max_solution_count=1")
            TextColors.RED.println("==========================================================")
            max_solution_count = 1

        ## @brief runber of redundancy sampling
        self.t0 = time.time()
        self.DOF = len(initial_state.Q)
        self.initial_state = initial_state
        self.max_solution_count = max_solution_count
        with self.gtimer.block("initialize_memory"):
            if multiprocess:
                if display:
                    print("Cannot display motion in multiprocess")
                    display = False
                if N_agents is None:
                    N_agents = cpu_count() / 2
                self.N_agents = N_agents
                print("Use {}/{} agents".format(N_agents, cpu_count()))
                self.search_counter = self.manager.Value('i', 0)
                self.solution_count = self.manager.Value('i', 0)
                self.stop_now = self.manager.Value('i', 0)
                self.tplan.initialize_memory(self.manager)
                if self.mplan.flag_log:
                    self.mplan.reset_log(flag_log=self.mplan.flag_log, manager=self.manager)
                for mfilter in self.mplan.motion_filters:
                    mfilter.prepare_multiprocess_lock(self.manager)
            else:
                self.N_agents = 1
                self.search_counter = SingleValue('i', 0)
                self.solution_count = SingleValue('i', 0)
                self.stop_now =  SingleValue('i', 0)
                self.tplan.initialize_memory(None)
                for mfilter in self.mplan.motion_filters:
                    mfilter.prepare_multiprocess_lock(None)

        with self.gtimer.block("init_search"):
            self.tplan.init_search(initial_state, goal_nodes)

        if multiprocess:
            with self.gtimer.block("start_process"):
                kwargs.update({"timeout": timeout})
                self.proc_list = [Process(
                    target=self.__search_loop,
                    args=(id_agent, N_search, False, dt_vis, verbose, timeout_loop),
                    kwargs=kwargs) for id_agent in range(N_agents)]
                for proc in self.proc_list:
                    proc.daemon = True
                    proc.start()

            if wait_proc:
                self.wait_procs(timeout_loop, looptime_extra)
        else:
            self.proc_list = []
            self.__search_loop(0, N_search, display, dt_vis, verbose, timeout_loop, timeout=timeout, **kwargs)
        elapsed = self.gtimer.toc("search_loop")
        print(
            "========================== FINISHED ({} / {} s) ==============================]".format(
                round(elapsed/1e3, 1), round(timeout_loop, 1)
            ))

    def wait_procs(self, timeout_loop, looptime_extra):
        self.non_joineds = []
        self.gtimer.tic("wait_procs")
        elapsed = 0
        while (self.stop_now.value < self.N_agents) and (elapsed<timeout_loop):
            time.sleep(0.1)
            elapsed = self.gtimer.toc("wait_procs") / self.gtimer.scale

        self.gtimer.tic("wait_procs_extra")
        all_stopped = False
        while not all_stopped:
            all_stopped = True
            for i_p, proc in enumerate(self.proc_list):
                elapsed = self.gtimer.toc("wait_procs_extra") / self.gtimer.scale
                proc_alive = proc.is_alive()
                all_stopped = all_stopped and not proc_alive
                if proc_alive:
                    if elapsed > looptime_extra:
                        proc.terminate()
                        time.sleep(0.1)
                        if not proc.is_alive():
                            self.non_joineds.append(i_p)
                        continue
                    proc.join(timeout=0.1)
        if len(self.non_joineds) > 0:
            TextColors.RED.println("[ERROR] Non-joined subprocesses: {}".format(self.non_joineds))

    def __search_loop(self, ID, N_search,
                      display=False, dt_vis=None, verbose=False, timeout_loop=600,
                      add_homing=True, post_optimize=False, home_pose=None,  **kwargs):
        loop_counter = 0
        sample_fail_counter = 0
        sample_fail_max = 5
        no_queue_stop = False
        ret = False
        while (N_search is None or self.tplan.snode_counter.value < N_search) \
                and (time.time() - self.t0) < timeout_loop \
                and (self.stop_now.value < self.N_agents):
            loop_counter += 1
            snode, from_state, to_state, sample_fail = self.tplan.sample()
            if not sample_fail:
                sample_fail_counter = 0
                with self.counter_lock:
                    self.search_counter.value = self.search_counter.value + 1
            else:
                time.sleep(0.1)
                sample_fail_counter += 1
                if sample_fail_counter > sample_fail_max:
                    no_queue_stop = True
                    break
                else:
                    continue
            if verbose:
                print('try: {} - {}->{}'.format(snode.idx, from_state.node, to_state.node))
            self.gtimer.tic("test_connection")
            traj, new_state, error, succ = self.test_connection(from_state, to_state,
                                                                display=display, dt_vis=dt_vis, verbose=verbose,
                                                                **kwargs)
            if new_state.node == (2, 2, 'grip0'):
                print("x")
            simtime = self.gtimer.toc("test_connection")
            snode_new = self.tplan.make_search_node(snode, new_state, traj)
            if succ:
                snode_new = self.tplan.connect(snode, snode_new)
            ret = self.tplan.update(snode, snode_new, succ)
            if verbose:
                print('result: {} - {}->{} = {}'.format(snode.idx, from_state.node, new_state.node, "success" if succ else "fail"))
                if succ:
                    print('branching: {}->{} ({}/{} s, steps/err: {}({} ms)/{})'.format(
                        snode.idx, snode_new.idx if succ else "", round(time.time() - self.t0, 2), round(timeout_loop, 2),
                        len(traj), simtime,
                        error))
                    print('=' * 150)
            if ret:
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
            term_reason = "max iteration time reached ({}/{} s)".format(round(time.time()-self.t0, 1), round(timeout_loop, 1))
            self.stop_now.value = self.N_agents
        elif ret:
            term_reason = "required answers acquired"
            if add_homing:
                print("++ adding return motion to acquired answer ++")
                home_state = self.tplan.snode_dict[0].copy(self.pscene)
                home_state.Q = self.pscene.combined_robot.home_pose if home_pose is None else home_pose
                if add_homing>1:
                    self.add_return_motion(snode_new, try_count=add_homing, initial_state=home_state)
                else:
                    self.add_return_motion(snode_new, initial_state=home_state)
            if post_optimize:
                print("++ post-optimizing acquired answer ++")
                self.post_optimize_schedule(snode_new)
            self.stop_now.value = self.N_agents
        elif self.stop_now.value >= self.N_agents:
            term_reason = "Stop called from other agent"
        else:
            term_reason = "Unknown issue"
        print("=========================================================================================================")
        print("======================= terminated {}: {}  ({}/{}) ===============================".format(
            ID, term_reason, round(time.time()-self.t0, 1), round(timeout_loop, 1)))
        print("=========================================================================================================")

    ##
    # @brief test transition between states to connect
    # @param from_state         starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state           goal state (rnb-planning.src.pkg.planning.scene.State)
    # @return Traj      Full trajectory as array of Q
    # @return end_state     new state at the end of transition (rnb-planning.src.pkg.planning.scene.State)
    # @return error     planning error
    # @return success   success/failure of planning result
    def test_connection(self, from_state=None, to_state=None, display=False, dt_vis=1e-2,
                          error_skip=0, verbose=False, **kwargs):
        if display:
            self.pscene.gscene.clear_highlight()
            snames, change_ok = self.pscene.get_changing_subjects(from_state, to_state)
            for sname in snames:
                self.pscene.set_object_state(from_state)
                self.pscene.gscene.update_markers_all()
                self.pscene.gscene.show_pose(from_state.Q)
                self.pscene.show_binding(to_state.binding_state[sname])

        Traj, LastQ, error, success, chain_list = \
            self.mplan.plan_transition(from_state, to_state, verbose=verbose, **kwargs)

        if display:
            if success:
                self.pscene.gscene.show_motion(Traj, error_skip=error_skip, period=dt_vis)
            self.pscene.gscene.clear_highlight()

        end_state = self.pscene.rebind_all(chain_list, LastQ)
        return Traj, end_state, error, success

    ##
    # @brief refine trajectories in a task schedule
    # @param schedule       sequence of SearchNodes to refine trajectory
    # @param multiprocess   use multiprocess
    # @param N_try      number of trials for each motion
    # @param repeat_until_success   repeat for failed motions
    # @param max_repeat    max nubmer of repeats
    # @param kwargs         arguments to be used to refine trajectory
    def refine_trajectory(self, schedule, multiprocess=True, N_try=None, repeat_until_success=True, max_repeat=2,
                          **kwargs):
        schedule_dict = {i_s: snode for i_s, snode in enumerate(schedule)}
        snode_keys = sorted(schedule_dict.keys())[1:]
        try_count = 0
        if multiprocess:
            self.refined_traj_dict = self.manager.dict()
            self.refine_success_dict = self.manager.dict()
        else:
            self.refined_traj_dict = dict()
            self.refine_success_dict = dict()
        while len(snode_keys)>0 and try_count < max_repeat:
            try_count += 1
            if multiprocess:
                if N_try is None:
                    # optimal efficiency is acquired with the number of agents same as the number of cores.
                    N_agents = cpu_count()
                    N_try = int(np.ceil(N_agents/float((len(snode_keys)-1))))
                print("Try {} times for each trajectory".format(N_try))
                self.refine_proc_dict = {skey: [Process(
                    target=self.__refine_trajectory,
                    args=(skey,schedule_dict[skey-1],schedule_dict[skey]), kwargs=kwargs)
                    for _ in range(N_try)] for skey in snode_keys}
                for proc_list in self.refine_proc_dict.values():
                    for proc in proc_list:
                        proc.daemon = True
                        proc.start()
                time_start = time.time()
                timeout_max = np.max([v for k, v in kwargs.items() if "timeout" in k])*1.2
                while ((not all([key in self.refine_success_dict and self.refine_success_dict[key]
                                for key in self.refine_proc_dict.keys()]))
                       and time.time()-time_start < timeout_max):
                    for proc_list in self.refine_proc_dict.values():
                        for proc in proc_list:
                            proc.join(timeout=0.1)
            else:
                for skey in snode_keys:
                    self.__refine_trajectory(skey, schedule_dict[skey-1],schedule_dict[skey], **kwargs)
            snode_keys = [skey for skey in snode_keys
                          if skey not in self.refine_success_dict or not self.refine_success_dict[skey]]
            if not repeat_until_success:
                break
        for key in self.refine_success_dict.keys():
            if self.refine_success_dict[key]:
                schedule[key].set_traj(self.refined_traj_dict[key],  schedule[key].traj_tot)


    def __refine_trajectory(self, key, snode_from, snode_to, **kwargs):
        from_state = snode_from.state
        to_state = snode_to.state
        Traj, LastQ, error, success, chain_list = \
            self.mplan.plan_transition(from_state, to_state, **kwargs)

        if success:
            if key in self.refine_success_dict and self.refine_success_dict[key]:
                traj_old = self.refined_traj_dict[key]
                len_traj_old = np.sum(np.abs(differentiate(traj_old, 1)[:-1]))
                len_traj_new = np.sum(np.abs(differentiate(Traj, 1)[:-1]))
                Traj = Traj if len_traj_new < len_traj_old else traj_old
            self.refined_traj_dict[key] = Traj
        self.refine_success_dict[key] = success



    ##
    # @brief add return motion to a SearchNode schedule
    def add_return_motion(self, snode_last, initial_state=None, timeout=1, try_count=3):
        if initial_state is None:
            initial_state = self.initial_state
        state_last = snode_last.state
        initial_state.Q = np.array(initial_state.Q)
        diffQ = initial_state.Q - state_last.Q
        diff_dict = {rname: np.sum(np.abs(diffQ[idx]))>1e-4
                     for rname, idx in self.pscene.combined_robot.idx_dict.items()}
        snode_pre = snode_last
        state_pre = state_last
        added_list = []
        for rname, diff in diff_dict.items():   # add return motion to robots not at home
            if diff:
                rbt_idx = self.pscene.combined_robot.idx_dict[rname]
                state_new = state_pre.copy(self.pscene)
                state_new.Q[rbt_idx] = initial_state.Q[rbt_idx]
                for _ in range(try_count):
                    traj, state_next, error, succ = self.test_connection(state_pre, state_new,
                                                                         display=False, timeout=timeout)
                    if succ:
                        break
                snode_next = self.tplan.make_search_node(snode_pre, state_next, traj)
                if succ:
                    snode_next = self.tplan.connect(snode_pre, snode_next)
                    self.tplan.update(snode_pre, snode_next, succ)
                    added_list.append(snode_next)
                    snode_pre = snode_next
                    state_pre = state_next
                else:
                    break
                time.sleep(0.2)
        return added_list

    ##
    # @brief    optimize a SearchNode schedule
    # @remark   MoveitPlanner should be set as the motion planner by set_motion_planner
    # @param    post_opt    set this value True to use post optimizer in OMPL (default), you need to install optimizer plugins
    # @param    plannerconfig   set this keyword value to a optimal algorithm in PlannerConfig and set post_opt False to use optimal planning
    # @param    reverse     set this value True to apply planning in reverse direction
    def post_optimize_schedule(self, snode_last, timeout=5, post_opt=True, reverse=False, **kwargs):
        snode_pre = None
        state_pre = None
        snode_schedule = self.tplan.idxSchedule2SnodeScedule(snode_last.parents + [snode_last.idx])
        for snode in snode_schedule:
            if snode.traj is not None:
                state_new = state_pre.copy(self.pscene)
                state_new.Q = snode.traj[-1]
                # no optimization for constrained motion
                post_opt = self.pscene.is_constrained_transition(state_pre, snode.state, check_available=False)

                if post_opt:
                    print("Optimize {} - > {}:".format(snode_pre.state.node, snode.state.node))
                    if reverse:
                        Traj, LastQ, error, success, chain_list = self.mplan.plan_transition(state_new, state_pre,
                                                                                               timeout=timeout,
                                                                                               post_opt=post_opt,
                                                                                               **kwargs)
                        Traj = np.array(list(reversed(Traj)))
                    else:
                        Traj, LastQ, error, success, chain_list = self.mplan.plan_transition(state_pre, state_new,
                                                                                               timeout=timeout,
                                                                                               post_opt=post_opt,
                                                                                               **kwargs)
                    if success:
                        print("Success")
                        snode.set_traj(Traj, snode_pre.traj_tot)
                        snode.state.Q = Traj[-1]
                        self.tplan.snode_dict[snode.idx] = snode
                    else:
                        print("Failure")
            snode_pre = snode
            state_pre = snode.state

    ##
    # @brief play schedule on rviz
    # @param snode_schedule list of SearchNode
    # @param period play period
    def play_schedule(self, snode_schedule, period=0.01):
        snode_pre = snode_schedule[0]
        for snode in snode_schedule[1:]:
            self.pscene.set_object_state(snode_pre.state)
            self.pscene.gscene.update_markers_all()
            print("{}->{}".format(snode_pre.state.node, snode.state.node))
            if snode.traj is None or len(snode.traj) == 0:
                snode_pre = snode
                continue
            self.pscene.gscene.clear_highlight()
            time.sleep(0.1)
            for sname in self.pscene.subject_name_list:
                btf_pre, btf = snode_pre.state.binding_state[sname], snode.state.binding_state[sname]
                if btf_pre.get_chain() != btf.get_chain():
                    self.pscene.show_binding(btf)
            if period<0.01:
                self.pscene.gscene.show_motion(snode.traj[::int(0.01/period)], period=0.01)
            else:
                self.pscene.gscene.show_motion(snode.traj, period=period)
            time.sleep(period)
            self.pscene.gscene.show_pose(snode.traj[-1])
            snode_pre = snode
        self.pscene.gscene.clear_highlight()
        time.sleep(0.1)

    ##
    # @brief execute grasping as described in the given state
    def execute_grip(self, state):
        grasp_dict = {}
        for name in self.pscene.combined_robot.robot_names:
            grasp_dict[name] = False

        for btf in state.binding_state.values():
            oname, bpoint, binder, bgeo = btf.get_chain()
            print("binder: {}".format(binder))
            if binder in self.pscene.actor_robot_dict:
                rname = self.pscene.actor_robot_dict[binder]
                print("rname: {}".format(rname))
                if rname is not None:
                    grasp_dict[rname] = True

        self.pscene.combined_robot.grasp(**grasp_dict)

    ##
    # @brief execute schedule
    # @param mode_switcher ModeSwitcher class instance
    def execute_schedule(self, snode_schedule, auto_stop=True, mode_switcher=None, one_by_one=False, multiproc=False,
                         error_stop_deg=10, auto_sync_robot_pose=False):
        self.execute_res = False
        snode_pre = snode_schedule[0]
        if auto_sync_robot_pose:
            self.pscene.combined_robot.joint_move_make_sure(snode_schedule[0].state.Q)
        for snode in snode_schedule:
            if snode.traj is None or len(snode.traj) == 0:
                snode_pre = snode
                continue
            if mode_switcher is not None:
                switch_state = mode_switcher.switch_in(snode_pre, snode)

            self.pscene.set_object_state(snode_pre.state)
            if multiproc:
                t_exe = Process(target=self.pscene.combined_robot.move_joint_traj,
                                args = (snode.traj,),
                                kwargs=dict(auto_stop=False, one_by_one=one_by_one, error_stop=error_stop_deg))
                t_exe.daemon = True
                t_exe.start()
                t_exe.join()
            else:
                self.pscene.combined_robot.move_joint_traj(snode.traj, auto_stop=False, one_by_one=one_by_one)
                if len(self.pscene.combined_robot.get_robots_in_act(snode.traj)) == 0:
                    print("No motion for connected robot - playing motion in RVIZ")
                    self.pscene.gscene.show_motion(snode.traj)
            if not one_by_one:
                if not self.pscene.combined_robot.wait_queue_empty(
                        trajectory=[snode.traj[0], snode.traj[-1]], error_stop=error_stop_deg):
                    self.pscene.combined_robot.stop_tracking()
                    self.execute_res = False
                    print("=================== ERROR ===================")
                    print("====== Robot configuration not in sync ======")
                    return False

            if mode_switcher is not None:
                mode_switcher.switch_out(switch_state, snode)
            self.execute_grip(snode.state)
            snode_pre = snode

        if auto_stop:
            self.pscene.combined_robot.stop_tracking()
        self.execute_res = True
        return True

    ##
    # @brief execute schedule
    # @param vel_scale velocity scale to max. robot velocity defined in RobotConfig
    # @param acc_scale acceleration scale to max. robot velocity defined in RobotConfig
    def execute_schedule_interpolate(self, snode_schedule, vel_scale=None, acc_scale=None):
        snode_schedule = [snode for snode in snode_schedule]  # re-wrap not to modify outer list
        snode_pre = snode_schedule[0]
        for snode, snode_next in zip(snode_schedule, snode_schedule[1:] + [None]):
            if snode.traj is None or len(snode.traj) == 0:
                snode_pre = snode
                continue
            self.pscene.set_object_state(snode_pre.state)

            # slow down if constrained motion
            scale_tmp = 1
            subject_list, success = self.pscene.get_changing_subjects(snode_pre.state, snode.state)
            for sname in subject_list:
                binding_to = snode.state.binding_state[sname].get_chain()
                binding_prev = snode_pre.state.binding_state[sname].get_chain()
                if self.pscene.subject_dict[sname].make_constraints(binding_to, binding_prev):
                    scale_tmp = self.constrained_motion_scale
                    break

            self.pscene.combined_robot.move_joint_wp(snode.traj,
                                                     vel_scale=vel_scale * scale_tmp,
                                                     acc_scale=acc_scale * scale_tmp)
            self.execute_grip(snode.state)
            snode_pre = snode

        for robot in self.pscene.combined_robot.robot_dict.values():
            if robot is not None:
                robot.stop_tracking()

    ##
    # @brief execute schedule
    def execute_schedule_in_sync(self, snode_schedule, control_freq=DEFAULT_TRAJ_FREQUENCY, on_rviz=False, stop_count_ref=25,
                         vel_scale=0.2, acc_scale=0.005, rviz_pub=None):
        snode_schedule = [snode for snode in snode_schedule]    # re-wrap not to modify outer list
        state_0 = snode_schedule[0].state
        state_fin = snode_schedule[-1].state
        state_home = state_fin.copy(self.pscene)
        state_home.Q = np.array(self.pscene.combined_robot.home_pose)
        trajectory, Q_last, error, success, chain_list = self.mplan.plan_transition(state_fin, state_home)
        if success:
            snode_home = SearchNode(0, state_home, [], [], depth=0)
            snode_home.set_traj(trajectory, 0)
            snode_schedule.append(snode_home)

        if not on_rviz:
            self.execute_grip(state_0)
            self.pscene.set_object_state(state_0)

        for snode in snode_schedule:
            if snode.traj is not None:
                trajectory, trajectory_vel = apply_vel_acc_lims(snode.traj, DT=1 / float(control_freq),
                                                                urdf_content=self.pscene.gscene.urdf_content,
                                                                joint_names=self.pscene.gscene.joint_names,
                                                                vel_scale=vel_scale, acc_scale=acc_scale)
                Q0 = trajectory[0]
                time.sleep(0.2)
                if not on_rviz:
                    self.pscene.combined_robot.joint_move_make_sure(Q0)

                with MultiTracker(self.pscene.combined_robot.get_robot_list(), self.pscene.combined_robot.get_indexing_list(),
                                  Q0, on_rviz=on_rviz) as mt:
                    stop_count = 0
                    N_traj = len(trajectory)
                    i_q = 0
                    while True:
                        POS_CUR = trajectory[i_q]
                        self.gtimer.tic("move_wait")
                        if on_rviz:
                            self.pscene.combined_robot.wait_step(self.pscene.gscene.rate.sleep_dur.to_sec())
                            all_sent = True
                        else:
                            all_sent = mt.move_possible_joints_x4(POS_CUR)
                        self.gtimer.toc("move_wait", stack=True)
                        self.gtimer.tic("rviz")
                        if rviz_pub is not None:
                            rviz_pub.update({}, POS_CUR)
                        self.gtimer.toc("rviz", stack=True)

                        if all_sent:
                            if i_q < N_traj-1:
                                i_q += 1            # go to next Q
                            else:
                                stop_count += 1     # increase stop count (slow stop)

                            if stop_count>stop_count_ref:
                                break               # stop after reference count

            self.pscene.set_object_state(snode.state)
            if not on_rviz:
                self.execute_grip(state_0)

    def get_updated_schedule(self, snode_schedule, Q0, stype_overwrite_on_conflict=None, **kwargs):
        snode_schedule_new = []
        for snode in snode_schedule:
            snode_schedule_new.append(snode.copy(self.pscene))
        snode_schedule_new[0].state.Q = Q0

        for snode_pre, snode_nxt in zip(snode_schedule_new[:-1], snode_schedule_new[1:]):
            rnames = map(lambda x: x[0], self.pscene.combined_robot.get_robots_in_act(snode_nxt.traj))
            rnames_new = map(lambda x: x[0], self.pscene.combined_robot.get_robots_in_act(
                [snode_pre.state.Q, snode_nxt.state.Q]))
            rnames_diff = list(set(rnames_new) - set(rnames))
            if len(rnames_diff) == 0:
                break
            elif len(rnames_diff) > 1:
                raise (RuntimeError("Too much difference"))
            else:
                rname = rnames_diff[0]
                idx_fix = self.pscene.combined_robot.idx_dict[rname]

                print("Try Update {} -> {}".format(rname, snode_pre.state.node, snode_nxt.state.node))
                print("{}: {} -> {}".format(rname,
                                            np.round(snode_pre.state.Q[idx_fix], 3),
                                            np.round(snode_nxt.state.Q[idx_fix], 3)))

                snode_nxt.state.Q[idx_fix] = snode_pre.state.Q[idx_fix]

                if np.sum([n0 != n1
                           for n0, n1
                           in zip(snode_pre.state.node, snode_nxt.state.node)]
                          ) > 1:
                    wtask_name = [sname
                                  for sname, stype
                                  in zip(pscene.subject_name_list,
                                         pscene.subject_type_list)
                                  if stype == stype_overwrite_on_conflict][0]

                    snode_pre.state.binding_state[wtask_name] = \
                        snode_nxt.state.binding_state[wtask_name]
                    snode_pre.state.set_binding_state(pscene, snode_pre.state.binding_state, snode_pre.state.state_param)

                traj, state_next, error, succ = \
                    self.test_connection(from_state=snode_pre.state,
                                         to_state=snode_nxt.state, **kwargs)
                if succ:
                    snode_nxt.set_traj(traj)
                    snode_nxt.state = state_next
                    print("Update success: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node))
                else:
                    TextColors.RED.println(("Update fail: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node)))
                    raise (RuntimeError("Update fail: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node)))
        return snode_schedule_new

    def replan_schedule(self, snode_schedule, Q0, stype_overwrite_on_conflict=None, **kwargs):
        snode_schedule_new = []
        for snode in snode_schedule:
            snode_schedule_new.append(snode.copy(self.pscene))
        snode_schedule_new[0].state.Q = Q0

        for snode_pre, snode_nxt in zip(snode_schedule_new[:-1], snode_schedule_new[1:]):
            rnames = map(lambda x: x[0], self.pscene.combined_robot.get_robots_in_act(snode_nxt.traj))
            rnames_new = map(lambda x: x[0], self.pscene.combined_robot.get_robots_in_act(
                [snode_pre.state.Q, snode_nxt.state.Q]))
            rnames_diff = list(set(rnames_new) - set(rnames))
            if len(rnames_diff) == 0:
                break
            elif len(rnames_diff) > 1:
                raise (RuntimeError("Too much difference"))
            else:
                rname = rnames_diff[0]
                idx_fix = self.pscene.combined_robot.idx_dict[rname]

                print("Try Update {} -> {}".format(rname, snode_pre.state.node, snode_nxt.state.node))
                print("{}: {} -> {}".format(rname,
                                            np.round(snode_pre.state.Q[idx_fix], 3),
                                            np.round(snode_nxt.state.Q[idx_fix], 3)))

                snode_nxt.state.Q[idx_fix] = snode_pre.state.Q[idx_fix]

                if np.sum([n0 != n1
                           for n0, n1
                           in zip(snode_pre.state.node, snode_nxt.state.node)]
                          ) > 1:
                    wtask_name = [sname
                                  for sname, stype
                                  in zip(pscene.subject_name_list,
                                         pscene.subject_type_list)
                                  if stype == stype_overwrite_on_conflict][0]

                    snode_pre.state.binding_state[wtask_name] = \
                        snode_nxt.state.binding_state[wtask_name]
                    snode_pre.state.set_binding_state(pscene, snode_pre.state.binding_state, snode_pre.state.state_param)

                traj, state_next, error, succ = \
                    self.test_connection(from_state=snode_pre.state,
                                         to_state=snode_nxt.state, **kwargs)
                if succ:
                    snode_nxt.set_traj(traj)
                    snode_nxt.state = state_next
                    print("Update success: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node))
                else:
                    TextColors.RED.println(("Update fail: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node)))
                    raise (RuntimeError("Update fail: {} -> {}".format(snode_pre.state.node, snode_nxt.state.node)))
        return snode_schedule_new