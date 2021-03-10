import time
from .scene import State
from ..utils.utils import SingleValue, list2dict, differentiate, GlobalTimer
from ..utils.joint_utils import apply_vel_acc_lims
from ..controller.repeater.repeater import DEFAULT_TRAJ_FREQUENCY, MultiTracker
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
        pscene.set_object_state(pscene.update_state(pscene.combined_robot.home_pose))
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
        ## @brief stop flag dictionary for multiple agents
        self.stop_dict = None
        ## @brief PriorityQueueManager
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.counter_lock = self.manager.Lock()
        self.gtimer = GlobalTimer.instance()

    ##
    # @param mplan subclass instance of rnb-planning.src.pkg.planning.motion.interface.MotionInterface
    def set_motion(self, mplan):
        self.mplan = mplan
        mplan.update_gscene()

    ##
    # @param tplan subclass instance of rnb-planning.src.pkg.planning.task.interface.TaskInterface
    def set_sampler(self, tplan):
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
            self.stop_dict = self.manager.dict()
            self.tplan.initialize_memory(self.manager)
        else:
            self.N_agents = 1
            self.search_counter = SingleValue('i', 0)
            self.stop_now =  SingleValue('i', 0)
            self.stop_dict = {}
            self.tplan.initialize_memory(None)

        self.tplan.init_search(initial_state, goal_nodes)
        if multiprocess:
            self.proc_list = [Process(
                target=self.__search_loop,
                args=(id_agent, terminate_on_first, N_search, False, dt_vis, verbose, timeout_loop),
                kwargs=kwargs) for id_agent in range(N_agents)]
            for proc in self.proc_list:
                proc.start()

            if wait_proc:
                for proc in self.proc_list:
                    while not self.stop_now.value:
                        proc.join(timeout=0.1)
        else:
            self.__search_loop(0, terminate_on_first, N_search, display, dt_vis, verbose, timeout_loop, **kwargs)

    def __search_loop(self, ID, terminate_on_first, N_search,
                      display=False, dt_vis=None, verbose=False, timeout_loop=600, **kwargs):
        loop_counter = 0
        self.stop_dict[ID] = False
        ret = False
        while (N_search is None or self.tplan.snode_counter.value < N_search) and (time.time() - self.t0) < timeout_loop and not self.stop_now.value:
            loop_counter += 1
            snode, from_state, to_state, redundancy_dict, sample_fail = self.tplan.sample()
            with self.counter_lock:
                if not sample_fail:
                    self.search_counter.value = self.search_counter.value + 1
                self.stop_dict[ID] = sample_fail
                if sample_fail:
                    if all([self.stop_dict[i_proc] for i_proc in range(self.N_agents)]):
                        break
                    else:
                        continue
            self.gtimer.tic("test_connection")
            traj, new_state, error, succ = self.test_connection(from_state, to_state, redundancy_dict=redundancy_dict,
                                                                  display=display, dt_vis=dt_vis, **kwargs)
            simtime = self.gtimer.toc("test_connection")
            snode_new = self.tplan.make_search_node(snode, new_state, traj,  redundancy_dict)
            if succ:
                snode_new = self.tplan.connect(snode, snode_new)
            ret = self.tplan.update(snode, snode_new, succ)
            if verbose:
                print('node: {}->{} = {}'.format(from_state.node, to_state.node, "success" if succ else "fail"))
                if succ:
                    print('branching: {}->{} ({}/{} s, steps/err: {}({} ms)/{})'.format(
                        snode.idx, snode_new.idx if succ else "", round(time.time() - self.t0, 2), round(timeout_loop, 2),
                        len(traj), simtime,
                        error))
                    print('=' * 150)
            if terminate_on_first and ret:
                self.stop_now.value = 1
                break

        if self.stop_dict[ID]:
            term_reson = "node queue empty"
        elif N_search is not None and self.tplan.snode_counter.value >= N_search:
            term_reson = "max search node count reached ({}/{})".format(self.tplan.snode_counter.value, N_search)
        elif (time.time() - self.t0) >= timeout_loop:
            term_reson = "max iteration time reached ({}/{} s)".format(int(time.time()), self.t0)
        elif ret:
            term_reson = "first answer acquired"
        else:
            term_reson = "first answer acquired from other agent"

        print("=========================================================================================================")
        print("======================= terminated {}: {} ===============================".format(ID, term_reson))
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
            for bd in binding_list:
                self.pscene.rebind(bd, list2dict(LastQ, self.pscene.gscene.joint_names))

        binding_state, state_param = self.pscene.get_object_state()
        end_state = State(binding_state, state_param, list(LastQ), self.pscene)
        return Traj, end_state, error, success

    ##
    # @brief find best schedule by trajectory length
    def find_best_schedule(self, schedule_sorted):
        best_snode_schedule = None
        best_score = 1e10
        for ss in schedule_sorted:
            schedule = ss
            snode_schedule_list = self.idxSchedule2SnodeScedule(schedule, self.pscene.combined_robot.home_pose)
            score = np.sum([snode.traj_length for snode in snode_schedule_list])
            if score < best_score:
                best_score = score
                best_snode_schedule = snode_schedule_list
        return best_snode_schedule

    ##
    # @brief find all schedules
    def print_snode_list(self):
        for i_s, snode in sorted(self.tplan.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.tplan.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

    ##
    # @brief sort schedules
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    ##
    # @brief get list of SearchNode from list of SearchNode index
    def idxSchedule2SnodeScedule(self, schedule):
        snode_schedule = [self.tplan.snode_dict[i_sc] for i_sc in schedule]
        return snode_schedule

    ##
    # @brief add return motion to a SearchNode schedule
    def add_return_motion(self, snode_schedule, timeout=5):
        snode_last = snode_schedule[-1]
        state_last = snode_last.state
        state_first = snode_schedule[0].state
        state_new = state_last.copy(self.pscene)
        state_new.Q = copy(state_first.Q)
        traj, new_state, error, succ = self.test_connection(state_last, state_new, redundancy_dict=None,
                                                              display=False, timeout=timeout)
        snode_new = self.tplan.make_search_node(snode_last, new_state, traj, None)
        if succ:
            snode_new = self.tplan.connect(snode_last, snode_new)
            snode_schedule.append(snode_new)
        return snode_schedule

    ##
    # @brief play schedule on rviz
    # @param snode_schedule list of SearchNode
    # @param period play period
    def play_schedule(self, snode_schedule, period=0.01):
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

        self.pscene.combined_robot.grasp_by_dict(grasp_dict)

    ##
    # @brief execute schedule
    def execute_schedule(self, snode_schedule, vel_scale=None, acc_scale=None):
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

        self.execute_grip(state_0)
        self.pscene.set_object_state(state_0)

        for snode in snode_schedule:
            if snode.traj is not None:
                time.sleep(1)
                print("go")
                self.pscene.combined_robot.move_joint_wp(snode.traj, vel_scale, acc_scale)

            self.pscene.set_object_state(snode.state)
            self.execute_grip(snode.state)

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
                    self.pscene.combined_robot.joint_make_sure(Q0)

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
