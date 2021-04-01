import time
from .scene import State
from ..utils.utils import SingleValue, list2dict, differentiate, GlobalTimer
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
    # @param N_search maximum number of search for each thread
    # @param timeout_loop search timeout default = 600 sec
    # @param N_agents number of multiprocess agents, Default = cpu_count
    # @param wait_proc wait for sub processes
    # @param display boolean flag for one-by-one motion display on rvia
    # @param dt_vis display period
    # @param verbose boolean flag for printing intermediate process
    def search(self, initial_state, goal_nodes, multiprocess=False,
                     terminate_on_first=True, N_search=None, N_agents=None, wait_proc=True,
                     display=False, dt_vis=0.01, verbose=False, timeout_loop=600, **kwargs):
        ## @brief runber of redundancy sampling
        self.t0 = time.time()
        self.DOF = len(initial_state.Q)
        self.initial_state = initial_state
        with self.gtimer.block("initialize_memory"):
            if multiprocess:
                if display:
                    print("Cannot display motion in multiprocess")
                    display = False
                if N_agents is None:
                    N_agents = cpu_count()
                self.N_agents = N_agents
                print("Use {}/{} agents".format(N_agents, cpu_count()))
                self.search_counter = self.manager.Value('i', 0)
                self.stop_now = self.manager.Value('i', 0)
                self.tplan.initialize_memory(self.manager)
            else:
                self.N_agents = 1
                self.search_counter = SingleValue('i', 0)
                self.stop_now =  SingleValue('i', 0)
                self.tplan.initialize_memory(None)

        with self.gtimer.block("init_search"):
            self.tplan.init_search(initial_state, goal_nodes)

        if multiprocess:
            with self.gtimer.block("start_process"):
                self.proc_list = [Process(
                    target=self.__search_loop,
                    args=(id_agent, terminate_on_first, N_search, False, dt_vis, verbose, timeout_loop),
                    kwargs=kwargs) for id_agent in range(N_agents)]
                for proc in self.proc_list:
                    proc.start()

            if wait_proc:
                self.wait_procs()
        else:
            self.proc_list = []
            self.__search_loop(0, terminate_on_first, N_search, display, dt_vis, verbose, timeout_loop, **kwargs)

    def wait_procs(self):
        for proc in self.proc_list:
            while not self.stop_now.value:
                proc.join(timeout=0.1)

    def __search_loop(self, ID, terminate_on_first, N_search,
                      display=False, dt_vis=None, verbose=False, timeout_loop=600, **kwargs):
        loop_counter = 0
        sample_fail_counter = 0
        sample_fail_max = 100
        no_queue_stop = False
        ret = False
        while (N_search is None or self.tplan.snode_counter.value < N_search) and (time.time() - self.t0) < timeout_loop and not self.stop_now.value:
            loop_counter += 1
            snode, from_state, to_state, redundancy_dict, sample_fail = self.tplan.sample()
            with self.counter_lock:
                if not sample_fail:
                    sample_fail_counter = 0
                    self.search_counter.value = self.search_counter.value + 1
                else:
                    sample_fail_counter += 1
                    if sample_fail_counter > sample_fail_max:
                        no_queue_stop = True
                        break
                    else:
                        continue
            if verbose:
                print('try: {} - {}->{}'.format(snode.idx, from_state.node, to_state.node))
            self.gtimer.tic("test_connection")
            traj, new_state, error, succ = self.test_connection(from_state, to_state, redundancy_dict=redundancy_dict,
                                                                  display=display, dt_vis=dt_vis, **kwargs)
            simtime = self.gtimer.toc("test_connection")
            snode_new = self.tplan.make_search_node(snode, new_state, traj,  redundancy_dict)
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
            if terminate_on_first and ret:
                break

        if no_queue_stop:
            term_reason = "node queue empty"
        elif N_search is not None and self.tplan.snode_counter.value >= N_search:
            term_reason = "max search node count reached ({}/{})".format(self.tplan.snode_counter.value, N_search)
        elif (time.time() - self.t0) >= timeout_loop:
            term_reason = "max iteration time reached ({}/{} s)".format(int(time.time()), self.t0)
        elif ret:
            print("++ adding return motion to acquired answer ++")
            self.add_return_motion(snode_new)
            term_reason = "first answer acquired"
        else:
            term_reason = "first answer acquired from other agent"
        self.stop_now.value = 1

        print("=========================================================================================================")
        print("======================= terminated {}: {} ===============================".format(ID, term_reason))
        print("=========================================================================================================")

    ##
    # @brief test transition between states to connect
    # @param from_state         starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state           goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param redundancy_dict    redundancy in dictionary format {object name: {axis: value}}
    # @return Traj      Full trajectory as array of Q
    # @return end_state     new state at the end of transition (rnb-planning.src.pkg.planning.scene.State)
    # @return error     planning error
    # @return success   success/failure of planning result
    def test_connection(self, from_state=None, to_state=None, redundancy_dict=None, display=False, dt_vis=1e-2,
                          error_skip=0, **kwargs):
        Traj, LastQ, error, success, binding_list = \
            self.mplan.plan_transition(from_state, to_state, redundancy_dict=redundancy_dict, **kwargs)

        if success:
            if display:
                self.pscene.gscene.show_motion(Traj, error_skip=error_skip, period=dt_vis)

        end_state = self.pscene.rebind_all(binding_list, LastQ)
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
        redundancy_dict = snode_to.redundancy_dict
        Traj, LastQ, error, success, binding_list = \
            self.mplan.plan_transition(from_state, to_state, redundancy_dict=redundancy_dict, **kwargs)

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
    def add_return_motion(self, snode_last, initial_state=None, timeout=5):
        if initial_state is None:
            initial_state = self.initial_state
        state_last = snode_last.state
        diffQ = initial_state.Q - state_last.Q
        diff_dict = {rname: np.sum(np.abs(diffQ[idx]))>1e-4
                     for rname, idx in self.pscene.combined_robot.idx_dict.items()}
        snode_pre = snode_last
        state_pre = state_last
        for rname, diff in diff_dict.items():   # add return motion to robots not at home
            if diff:
                rbt_idx = self.pscene.combined_robot.idx_dict[rname]
                state_new = state_pre.copy(self.pscene)
                state_new.Q[rbt_idx] = initial_state.Q[rbt_idx]
                traj, state_next, error, succ = self.test_connection(state_pre, state_new, redundancy_dict=None,
                                                                      display=False, timeout=timeout)
                snode_next = self.tplan.make_search_node(snode_pre, state_next, traj, None)
                if succ:
                    snode_next = self.tplan.connect(snode_pre, snode_next)
                    self.tplan.update(snode_pre, snode_next, succ)
                    snode_pre = snode_next
                    state_pre = state_next
                else:
                    break
                time.sleep(0.2)

    ##
    # @brief play schedule on rviz
    # @param snode_schedule list of SearchNode
    # @param period play period
    def play_schedule(self, snode_schedule, period=0.01):
        self.pscene.set_object_state(snode_schedule[0].state)
        for snode in snode_schedule:
            if snode.traj is not None:
                self.pscene.gscene.show_motion(snode.traj, period=period)
                time.sleep(period)
                self.pscene.gscene.show_pose(snode.traj[-1])
            self.pscene.set_object_state(snode.state)

    ##
    # @brief execute grasping as described in the given state
    def execute_grip(self, state):
        grasp_dict = {}
        for name in self.pscene.combined_robot.robot_names:
            grasp_dict[name] = False

        for binding in state.binding_state:
            oname, bpoint, binder, bgeo = binding
            print("binder: {}".format(binder))
            if binder in self.pscene.actor_robot_dict:
                rname = self.pscene.actor_robot_dict[binder]
                print("rname: {}".format(rname))
                if rname is not None:
                    grasp_dict[rname] = True

        self.pscene.combined_robot.grasp(**grasp_dict)

    ##
    # @brief execute schedule
    # @param vel_scale velocity scale to max. robot velocity defined in RobotConfig
    # @param acc_scale acceleration scale to max. robot velocity defined in RobotConfig
    def execute_schedule(self, snode_schedule, vel_scale=None, acc_scale=None):
        snode_schedule = [snode for snode in snode_schedule]    # re-wrap not to modify outer list
        state_0 = snode_schedule[0].state
        # state_fin = snode_schedule[-1].state
        # state_home = state_fin.copy(self.pscene)
        # state_home.Q = np.array(self.pscene.combined_robot.home_pose)
        # trajectory, Q_last, error, success, binding_list = self.mplan.plan_transition(state_fin, state_home)
        # if success:
        #     snode_home = SearchNode(0, state_home, [], [], depth=0, redundancy_dict=None)
        #     snode_home.set_traj(trajectory, 0)
        #     snode_schedule.append(snode_home)

        # self.pscene.combined_robot.joint_move_make_sure(snode_schedule[0].state.Q)
        # self.execute_grip(state_0)
        self.pscene.set_object_state(state_0)

        snode_pre = None
        for snode in snode_schedule:
            if snode.traj is not None:
                scale_tmp = 1
                if snode_pre is not None:
                    binding_list, success = self.pscene.get_slack_bindings(snode_pre.state, snode.state)
                    for binding in binding_list:
                        obj_name, ap_name, binder_name, binder_geometry_name = binding
                        binder_geometry_prev = snode_pre.state.binding_state[self.pscene.subject_name_list.index(obj_name)][-1]
                        if binder_geometry_prev == binder_geometry_name and \
                                self.pscene.subject_dict[obj_name].constrained:
                            scale_tmp = self.constrained_motion_scale
                snode_pre = snode
                self.pscene.combined_robot.move_joint_wp(snode.traj,
                                                         vel_scale=vel_scale*scale_tmp, acc_scale=acc_scale*scale_tmp)

            self.pscene.set_object_state(snode.state)
            self.execute_grip(snode.state)
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
        trajectory, Q_last, error, success, binding_list = self.mplan.plan_transition(state_fin, state_home)
        if success:
            snode_home = SearchNode(0, state_home, [], [], depth=0, redundancy_dict=None)
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
                            self.pscene.combined_robot.wait_step(self.pscene.gscene.rate)
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
