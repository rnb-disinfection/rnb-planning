from .interface import *
from ...utils.utils import *
from collections import defaultdict

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

from multiprocessing import Process, Lock, cpu_count
from multiprocessing.managers import SyncManager
class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)

class HandleAstarSampler(SamplerInterface):
    DEFAULT_TRANSIT_COST = 1.0
    DQ_MAX = np.deg2rad(45)
    WEIGHT_DEFAULT = 2.0
    DSCALE = 1e4

    def __init__(self, *args, **kwargs):
        SamplerInterface.__init__(self, *args, **kwargs)
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.dict_lock = self.manager.Lock()
        self.que_lock = self.manager.Lock()

    @record_time
    def build_graph(self, update_handles=True):
        if update_handles:
            self.graph.update_handles()
        bindings = get_all_mapping(self.graph.handle_dict.keys(),
                                   self.graph.binder_dict.keys())  # all possible binding combinations
        handle_combinations = list(
            product(*[self.graph.handle_dict[obj] for obj in self.graph.object_list]))  # all possible handle combinations
        uniq_binders = self.graph.get_unique_binders()  # binders cannot be shared by multiple objects
        ctrl_binders = self.graph.get_controlled_binders()  # all controllable binders
        self.node_list = []
        self.node_dict = {}
        for binding in bindings:  # binding combination loop
            if all([np.sum([bd == ub for bd in binding.values()]) <= 1 for ub in uniq_binders]):
                for hc in handle_combinations:  # handle combination loop
                    node = []
                    add_ok = True
                    for i_o, obj in zip(range(len(self.graph.object_list)), self.graph.object_list):  # object loop
                        hndl = self.graph.object_dict[obj].action_points_dict[hc[i_o]]  # handle object
                        binder = self.graph.binder_dict[binding[obj]]  # binder object
                        if binder.check_type(hndl):  # match handle-binder type
                            node += [(obj, hc[i_o], binding[obj])]
                        else:
                            add_ok = False  # exclude if mismatch
                    if add_ok:
                        node = tuple(node)
                        self.node_list += [node]  # save all matched node
                        self.node_dict[node] = []
        fixed_idx_dict = {}
        ctrl_idx_dict = {}
        fixed_bdg_dict = {}
        ctrl_bdg_dict = {}
        fix_node_dict = defaultdict(list)
        for node in self.node_list:
            fixed_idx = [i_n for i_n in range(len(node)) if node[i_n][2] not in ctrl_binders]  # fixed binding index
            ctrl_idx = [i_n for i_n in range(len(node)) if node[i_n][2] in ctrl_binders]  # control binding index
            fixed_bdg = tuple([node[idx] for idx in fixed_idx])  # fixed binding
            ctrl_bdg = tuple([node[idx] for idx in ctrl_idx])  # control binding
            for i in range(len(fixed_bdg) + 1):
                for fixed_bdg_subset in combinations(fixed_bdg, i):
                    fix_node_dict[fixed_bdg_subset].append(node)  # node list sharing same fixed binding combination
            fixed_idx_dict[node] = fixed_idx
            ctrl_idx_dict[node] = ctrl_idx
            fixed_bdg_dict[node] = fixed_bdg
            ctrl_bdg_dict[node] = ctrl_bdg

        for node in self.node_list:
            ctrl_idx = ctrl_idx_dict[node]
            if ctrl_idx:
                fixed_idx = fixed_idx_dict[node]
                bdg_fixed = fixed_bdg_dict[node]
                bdg_ctrl = ctrl_bdg_dict[node]
                bdr_ctrl = [bdg[2] for bdg in bdg_ctrl]
                ptr_ctrl = [bdg[1] for bdg in bdg_ctrl]
                conflict_ptr_ctrl = [self.graph.object_dict[bdg[0]].get_conflicting_handles(bdg[1]) for bdg in bdg_ctrl]

                nodes_same_fix = fix_node_dict[bdg_fixed]
                nodes_diff_ctrl = [nd for nd in nodes_same_fix if
                                   [nd[idx][2] for idx in ctrl_idx] != bdr_ctrl]  # if any control binder is changed
                nodes_diff_pt = [nd for nd in nodes_diff_ctrl if
                                 all([hnew not in cprev for hnew, cprev in zip([nd[idx][1] for idx in ctrl_idx],
                                                                               conflict_ptr_ctrl)])]  # if control handle is not conflicting
                nodes_neighbor = [nd for nd in nodes_diff_pt if all(
                    [bdr_new not in bdr_ctrl for bdr_new in
                     [bd[2] for bd, bd0 in zip(nd, node) if
                      bd != bd0]])]  # if no changed binder is not in the previous control binder
                self.node_dict[node] += nodes_neighbor
                for nd_n in nodes_neighbor:
                    self.node_dict[nd_n] += [node]

        for node in self.node_dict.keys():
            if node in self.node_dict[node]:
                self.node_dict[node].remove(node)
            self.node_dict[node] = [node] + list(set(self.node_dict[node]))

    def score_graph(self, goal_node):
        came_from = {}
        node_cost_dict = {}
        frontier = PriorityQueue()
        if isinstance(goal_node, list):
            for goal in goal_node:
                frontier.put((0, goal))
                came_from[goal] = None
                node_cost_dict[goal] = 0
        else:
            frontier.put((0, goal_node))
            came_from[goal_node] = None
            node_cost_dict[goal_node] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            for next in self.node_dict[current]:
                if next == current:
                    continue
                new_cost = node_cost_dict[current] + self.DEFAULT_TRANSIT_COST
                if next not in node_cost_dict or new_cost < node_cost_dict[next]:
                    node_cost_dict[next] = new_cost
                    priority = new_cost
                    frontier.put((priority, next))
                    came_from[next] = current
        return node_cost_dict

    def get_valid_neighbor(self, node, margin=0):
        neighbor = self.node_dict[node]
        neighbor_valid = []
        for leaf in neighbor:
            if self.goal_cost_dict[leaf]<=self.goal_cost_dict[node]+margin:
                neighbor_valid += [leaf]
        return neighbor_valid

    def reset_valid_node(self, margin=0, node=None):
        if node == None:
            node = self.initial_state.node
            self.valid_node_dict = {goal:[] for goal in self.goal_nodes}
        if node in self.valid_node_dict or self.check_goal_by_score(node, self.goal_cost_dict):
            return
        neighbor = self.get_valid_neighbor(node, margin=margin)
        if node in self.valid_node_dict and self.valid_node_dict[node] == neighbor:
            return
        self.valid_node_dict[node] = neighbor
        for leaf in neighbor:
            new_margin = margin - max(0, self.goal_cost_dict[leaf]-self.goal_cost_dict[node])
            new_margin = max(0, new_margin)
            if leaf != node and new_margin>=0:
                self.reset_valid_node(margin=new_margin, node=leaf)

    @record_time
    def init_search(self, initial_state, goal_nodes, tree_margin, depth_margin):
        self.initial_state = initial_state
        self.goal_nodes = goal_nodes
        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.node), self.score_graph(goal_nodes)
        self.reset_valid_node(tree_margin)
        self.depth_min = self.goal_cost_dict[initial_state.node]
        self.max_depth = self.depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()

    def add_node_queue_leafs(self, snode):
        self.dict_lock.acquire()
        snode.idx = self.snode_counter.value
        self.snode_dict[snode.idx] = snode
        self.snode_counter.value = self.snode_counter.value+1
        self.dict_lock.release()
        state = snode.state
        leafs = self.valid_node_dict[state.node]
        if len(leafs) == 0:
            return snode
        leafs = leafs[:-1] + [leafs[-1]] * self.joint_motion_num
        for leaf in leafs:
            depth = len(snode.parents) + 1
            expected_depth = depth + self.goal_cost_dict[leaf]
            if expected_depth > self.max_depth:
                continue
            to_state = state.copy(self.graph)

            if leaf == state.node:
                dQ = (1 - 2 * np.random.rand(self.DOF)) * self.DQ_MAX
                to_state.Q = np.sum([state.Q, dQ], axis=0)
                to_state.Q = np.minimum(np.maximum(to_state.Q, self.graph.joint_limits[:,0]), self.graph.joint_limits[:,1])
            else:
                to_state.set_node(leaf, self.graph)
            # self.snode_queue.put((snode, state, to_state), expected_depth * self.DSCALE - depth) ## breadth-first
            self.snode_queue.put(((expected_depth - depth) * self.DSCALE + depth, (snode, state, to_state))) ## greedy
        return snode

    @record_time
    def search_graph(self, initial_state, goal_nodes,
                     tree_margin=0, depth_margin=0, joint_motion_num=10,
                     terminate_on_first=True, N_search=100, N_agents=None, multiprocess=False,
                     display=False, dt_vis=None, verbose=False, print_expression=False, **kwargs):

        self.joint_motion_num = joint_motion_num
        self.t0 = time.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, goal_nodes, tree_margin, depth_margin)

        snode_root = SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                leafs_P=[self.WEIGHT_DEFAULT] * len(self.valid_node_dict[initial_state.node]),
                                depth=0, edepth=self.goal_cost_dict[initial_state.node])

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
            self.add_node_queue_leafs(snode_root)

            self.proc_list = [Process(
                target=self.__search_loop,
                args=(id_agent, terminate_on_first, N_search, False, dt_vis, verbose, print_expression),
                kwargs=kwargs) for id_agent in range(N_agents)]
            for proc in self.proc_list:
                proc.start()

            for proc in self.proc_list:
                proc.join()
        else:
            self.snode_counter = SingleValue('i', 0)
            self.search_counter = SingleValue('i', 0)
            self.stop_now =  SingleValue('i', 0)
            self.snode_dict = {}
            self.stop_dict = {}
            self.snode_queue = PriorityQueue()
            self.add_node_queue_leafs(snode_root)

            self.__search_loop(0, terminate_on_first, N_search, display, dt_vis, verbose, print_expression, **kwargs)

    @record_time
    def __search_loop(self, ID, terminate_on_first, N_search,
                      display=False, dt_vis=None, verbose=False, print_expression=False, timeout_loop=600, **kwargs):
        loop_counter = 0
        self.stop_dict[ID] = False
        while self.snode_counter.value < N_search and (time.time() - self.t0) < timeout_loop and not self.stop_now.value:
            loop_counter += 1
            self.que_lock.acquire()
            stop = False
            if self.snode_queue.empty():
                stop = True
            else:
                try:
                    snode, from_state, to_state = self.snode_queue.get(timeout=1)[1]
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
            traj, new_state, error, succ = self.graph.test_transition(from_state, to_state, display=display, dt_vis=dt_vis,
                                                          print_expression=print_expression, **kwargs)
            ret = False
            if succ:
                depth_new = len(snode.parents) + 1
                snode_new = SearchNode(idx=0, state=new_state, parents=snode.parents + [snode.idx], leafs=[], leafs_P=[],
                                       depth=depth_new, edepth=depth_new+self.goal_cost_dict[new_state.node])
                snode_new.set_traj(traj)
                snode_new = self.add_node_queue_leafs(snode_new)
                snode.leafs += [snode_new.idx]
                self.snode_dict[snode.idx] = snode
                if self.check_goal(snode_new.state.node, self.goal_nodes):
                    ret = True
            simtime = self.gtimer.toc("test_transition")
            if verbose:
                print('\n{} - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} ({}/{} s, steps/err: {}({} ms)/{})'.format(
                    "success" if succ else "fail",
                    int(self.goal_cost_dict[from_state.node]), int(self.goal_cost_dict[to_state.node]),
                    int(self.init_cost_dict[from_state.node]), int(self.init_cost_dict[to_state.node]),
                    snode.idx, snode_new.idx if succ else "", round(time.time() - self.t0, 2), round(timeout_loop, 2),
                    len(traj), simtime,
                    error))
                print('node: {}->{}'.format(from_state.node, to_state.node))
                print('=' * 150)
            if terminate_on_first and ret:
                self.stop_now.value = 1
                break
        print("=========================================================================================================")
        print("=============================================== terminate ===============================================")
        print("=========================================================================================================")

    def find_schedules(self):
        self.idx_goal = []
        schedule_dict = {}
        for i in range(self.snode_counter.value):
            snode = self.snode_dict[i]
            state = snode.state
            if self.check_goal(state.node, self.goal_nodes):
                self.idx_goal += [i]
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
        return schedule_dict

    def quiver_snodes(self, figsize=(10,10)):
        import matplotlib.pyplot as plt
        N_plot = self.snode_counter.value
        snode_vec = [v for k,v in sorted(self.snode_dict.items(), key=lambda x: x)]
        cost_vec = [self.goal_cost_dict[snode.state.node] for snode in snode_vec[1:N_plot]]
        parent_vec = [self.goal_cost_dict[self.snode_dict[snode.parents[-1]].state.node] for snode in snode_vec[1:N_plot]]
        plt.figure(figsize=figsize)
        X = list(range(1,N_plot))
        plt.quiver(X, parent_vec,
                   [0]*(N_plot-1),
                   np.subtract(cost_vec, parent_vec),
                   angles='xy', scale_units='xy', scale=1)
        plt.plot(X, cost_vec,'.')
        plt.plot(X, parent_vec,'.')
        plt.axis([0,N_plot+1,-0.5,4.5])
