import time
from .scene import State
from ..utils.utils import SingleValue, list2dict, differentiate, GlobalTimer
import numpy as np

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

from multiprocessing import Process, Lock, cpu_count
from multiprocessing.managers import SyncManager
import random
class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)


##
# @class SearchNode
# @brief search node
class SearchNode:
    ##
    # @param idx        incremental index of search node
    # @param state      rnb-planning.src.pkg.planning.scene.State
    # @param parents    list of parent SearchNode idx
    # @param leafs      list of available nodes
    # @param depth      depth of current node = number of parent
    # @param edepth     expected depth of current node = depth + optimal number of remaining steps
    # @param redundancy defined redundancy of transition in dictionary form, {object name: {axis: value}}
    def __init__(self, idx, state, parents, leafs, depth=None, edepth=None,
                 redundancy=None):
        self.idx, self.state, self.parents, self.leafs, self.depth, self.edepth, self.redundancy = \
            idx, state, parents, leafs, depth, edepth, redundancy
        self.traj = None
        self.traj_size = 0
        self.traj_length = 0

    ##
    # @brief    set current transition's trajectory + update trajectory length
    def set_traj(self, traj_full):
        self.traj_size = len(traj_full)
        self.traj_length = np.sum(np.abs(differentiate(traj_full, 1)[:-1])) if self.traj_size > 1 else 0
        self.traj = np.array(traj_full)

    def get_traj(self):
        return self.traj

    ##
    # @brief    copy SearchNode
    # @param    pscene  rnb-planning.src.pkg.planning.scene.PlanningScene
    def copy(self, pscene):
        return SearchNode(self.idx, State(self.state.node, self.state.obj_pos_dict, self.state.Q, pscene),
                          self.parents, self.leafs, self.depth, self.edepth, self.redundancy)


##
# @class    PlanningPipeline
# @brief    planning pipeline
class PlanningPipeline:
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene):
        ## @brief rnb-planning.src.pkg.planning.scene.PlanningScene
        self.pscene = pscene
        ## @brief number of multiprocess agents
        self.N_agents = None
        ## @brief number of valid generated search nodes
        self.snode_counter = None
        ## @brief count search steps
        self.search_counter = None
        ## @brief flag to stop multiprocess search
        self.stop_now =  None
        ## @brief search node dictionary
        self.snode_dict = None
        ## @brief stop flag dictionary for multiple agents
        self.stop_dict = None
        ## @brief waiting queue for non-validated search nodes
        self.snode_queue = None
        ## @brief PriorityQueueManager
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.dict_lock = self.manager.Lock()
        self.que_lock = self.manager.Lock()
        self.gtimer = GlobalTimer()

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
    # @brief run search algorithm
    # @param initial_state rnb-planning.src.pkg.planning.scene.State
    # @param goal definition of goal, depends on algorithem
    # @param multiprocess boolean flag for multiprocess search
    # @param N_redundant_sample number of redundancy sampling
    # @param terminate_on_first boolean flag for terminate on first answer
    # @param N_search maximum number of search for each thread
    # @param timeout_loop search timeout default = 600 sec
    # @param N_agents number of multiprocess agents, Default = cpu_count
    # @param display boolean flag for one-by-one motion display on rvia
    # @param dt_vis display period
    # @param verbose boolean flag for printing intermediate process
    def search(self, initial_state, goal, multiprocess=False, N_redundant_sample=30,
                     terminate_on_first=True, N_search=100, N_agents=None,
                     display=False, dt_vis=None, verbose=False, timeout_loop=600, **kwargs):
        ## @brief runber of redundancy sampling
        self.N_redundant_sample = N_redundant_sample
        self.t0 = time.time()
        self.DOF = len(initial_state.Q)
        self.tplan.init_search(initial_state, goal)
        snode_root = SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                depth=0, edepth=self.tplan.get_optimal_remaining_steps(initial_state))

        if multiprocess:
            if display:
                print("Cannot display motion in multiprocess")
                display = False
            if N_agents is None:
                N_agents = cpu_count()
            self.N_agents = N_agents
            print("Use {}/{} agents".format(N_agents, cpu_count()))
            self.snode_counter = self.manager.Value('i', 0)
            self.search_counter = self.manager.Value('i', 0)
            self.stop_now = self.manager.Value('i', 0)
            self.snode_dict = self.manager.dict()
            self.stop_dict = self.manager.dict()
            self.snode_queue = self.manager.PriorityQueue()
            self.__process_snode(snode_root)
            self.proc_list = [Process(
                target=self.__search_loop,
                args=(id_agent, terminate_on_first, N_search, False, dt_vis, verbose, timeout_loop),
                kwargs=kwargs) for id_agent in range(N_agents)]
            for proc in self.proc_list:
                proc.start()

            for proc in self.proc_list:
                proc.join()
        else:
            self.N_agents = 1
            self.snode_counter = SingleValue('i', 0)
            self.search_counter = SingleValue('i', 0)
            self.stop_now =  SingleValue('i', 0)
            self.snode_dict = {}
            self.stop_dict = {}
            self.snode_queue = PriorityQueue()
            self.__process_snode(snode_root)
            self.__search_loop(0, terminate_on_first, N_search, display, dt_vis, verbose, timeout_loop, **kwargs)

    def __search_loop(self, ID, terminate_on_first, N_search,
                      display=False, dt_vis=None, verbose=False, timeout_loop=600, **kwargs):
        loop_counter = 0
        self.stop_dict[ID] = False
        ret = False
        while self.snode_counter.value < N_search and (time.time() - self.t0) < timeout_loop and not self.stop_now.value:
            loop_counter += 1
            self.que_lock.acquire()
            stop = False
            if self.snode_queue.empty():
                stop = True
            else:
                try:
                    snode, from_state, to_state, redundancy_dict = self.snode_queue.get(timeout=1)[1]
                except:
                    stop=True
            self.stop_dict[ID] = stop
            if stop:
                self.que_lock.release()
                if all([self.stop_dict[i_proc] for i_proc in range(self.N_agents)]):
                    break
                else:
                    continue
            self.search_counter.value = self.search_counter.value + 1
            self.que_lock.release()
            self.gtimer.tic("test_transition")
            traj, new_state, error, succ = self.__test_transition(from_state, to_state, redundancy_dict=redundancy_dict,
                                                                  display=display, dt_vis=dt_vis, **kwargs)
            ret = False
            if succ:
                depth_new = len(snode.parents) + 1
                snode_new = SearchNode(
                    idx=0, state=new_state, parents=snode.parents + [snode.idx], leafs=[],
                    depth=depth_new, edepth=depth_new+self.tplan.get_optimal_remaining_steps(new_state),
                    redundancy=redundancy_dict)
                snode_new.set_traj(traj)
                snode_new = self.__process_snode(snode_new)
                snode.leafs += [snode_new.idx]
                self.snode_dict[snode.idx] = snode
                if self.tplan.check_goal(new_state):
                    ret = True
            simtime = self.gtimer.toc("test_transition")
            if verbose:
                print('node: {}->{} = {}'.format(from_state.onode, to_state.onode, "success" if succ else "fail"))
                if succ:
                    print('Remaining:{}->{} / branching: {}->{} ({}/{} s, steps/err: {}({} ms)/{})'.format(
                        int(self.tplan.get_optimal_remaining_steps(from_state)), int(self.tplan.get_optimal_remaining_steps(to_state)),
                        snode.idx, snode_new.idx if succ else "", round(time.time() - self.t0, 2), round(timeout_loop, 2),
                        len(traj), simtime,
                        error))
                    print('=' * 150)
            if terminate_on_first and ret:
                self.stop_now.value = 1
                break

        if self.stop_dict[ID]:
            term_reson = "node queue empty"
        elif self.snode_counter.value >= N_search:
            term_reson = "max search node count reached ({}/{})".format(self.snode_counter.value, N_search)
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
    # @brief update snode_counter and snode.idx
    # @param SearchNode
    def __update_idx(self, snode):
        self.dict_lock.acquire()
        snode.idx = self.snode_counter.value
        self.snode_dict[snode.idx] = snode
        self.snode_counter.value = self.snode_counter.value+1
        self.dict_lock.release()
        return snode

    ##
    # @brief add sampled leafs on queue
    # @param SearchNode
    def __process_snode(self, snode):
        self.__update_idx(snode)
        new_queue = self.tplan.get_leafs(snode, self.N_redundant_sample)
        self.que_lock.acquire()
        for qtem in new_queue:
            self.snode_queue.put(qtem)
        self.que_lock.release()
        return snode

    ##
    # @brief test transition
    # @param from_state         starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state           goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param redundancy_dict    redundancy in dictionary format {object name: {axis: value}}
    # @return Traj      Full trajectory as array of Q
    # @return end_state     new state at the end of transition (rnb-planning.src.pkg.planning.scene.State)
    # @return error     planning error
    # @return success   success/failure of planning result
    def __test_transition(self, from_state=None, to_state=None, redundancy_dict=None, display=False, dt_vis=1e-2,
                          error_skip=0, **kwargs):
        Traj, LastQ, error, success, binding_list = \
            self.mplan.plan_transition(from_state, to_state, redundancy_dict=redundancy_dict, **kwargs)

        if success:
            if display:
                self.pscene.gscene.show_motion(Traj, error_skip=error_skip, period=dt_vis)
            for bd in binding_list:
                self.pscene.rebind(bd, list2dict(LastQ, self.pscene.gscene.joint_names))

        node, obj_pos_dict = self.pscene.get_object_state()
        end_state = State(node, obj_pos_dict, list(LastQ), self.pscene)
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
    # @return list of SearchNode index list
    def find_schedules(self):
        self.idx_goal = []
        schedule_dict = {}
        for i in range(self.snode_counter.value):
            snode = self.snode_dict[i]
            state = snode.state
            if self.tplan.check_goal(state):
                self.idx_goal += [i]
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
        return schedule_dict

    ##
    # @brief find all schedules
    def print_snode_list(self):
        for i_s, snode in sorted(self.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

    ##
    # @brief sort schedules
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    ##
    # @brief get list of SearchNode from list of SearchNode index
    def idxSchedule2SnodeScedule(self, schedule, ZERO_JOINT_POSE=None):
        ZERO_JOINT_POSE = ZERO_JOINT_POSE or self.pscene.combined_robot.home_pose
        snode_schedule = [self.snode_dict[i_sc] for i_sc in schedule]
        snode_schedule.append(snode_schedule[-1].copy(self.pscene))
        snode_schedule[-1].state.Q = ZERO_JOINT_POSE
        snode_schedule[-1].set_traj(np.array([ZERO_JOINT_POSE]))
        return snode_schedule