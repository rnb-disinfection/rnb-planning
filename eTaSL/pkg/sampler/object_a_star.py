from .interface import *
from ..utils.utils import *
from ..utils.joint_utils import *
from collections import defaultdict
from copy import deepcopy

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

class ObjectAstarSampler(SamplerInterface):
    DEFAULT_TRANSIT_COST = 1.0
    DQ_MAX = np.deg2rad(45)
    WEIGHT_DEFAULT = 2.0
    DSCALE = 1e4

    def __init__(self, *args, **kwargs):
        SamplerInterface.__init__(self, *args, **kwargs)
        self.dict_lock = Lock()
        self.que_lock = Lock()
        self.manager = PriorityQueueManager()
        self.manager.start()

    @record_time
    def build_graph(self, update_handles=True):
        graph = self.graph
        if update_handles:
            graph.update_handles()

        oname_list = graph.object_list
        boname_list = graph.object_binder_dict.keys()
        obj_binding_dict = get_available_binder_dict(graph, oname_list,
                                                     boname_list)  # matching possible binder dictionary
        binder_combinations = list(
            product(*[obj_binding_dict[oname] for oname in oname_list]))  # all possible binding combination list
        uniq_binders = graph.get_unique_binders()  # binders cannot be shared by multiple objects
        uniq_bo_list = [boname for boname in boname_list if
                        (len(graph.object_binder_dict[boname]) == 1 and all(
                            [bname in uniq_binders for bname in graph.object_binder_dict[boname]]))]
        ctrl_binders = graph.get_controlled_binders()  # all controllable binders
        ctrl_bo_list = [boname for boname in boname_list if
                        all([bname in ctrl_binders for bname in graph.object_binder_dict[boname]])]

        # filter out conflicting use of uniq binding object
        usage_conflicts = []
        self_binding = []
        for bc in binder_combinations:
            for ubo in uniq_bo_list:
                if sum([ubo == bo for bo in bc]) > 1:
                    usage_conflicts.append(bc)
                    break
            if any([oname == boname for oname, boname in zip(graph.object_list, bc)]):
                self_binding.append(bc)

        all_conflicts = list(set(self_binding + usage_conflicts))
        for conflict in all_conflicts:
            binder_combinations.remove(conflict)

        # make all node connections
        self.node_list = binder_combinations
        self.node_dict = {k: list() for k in self.node_list}
        for node in self.node_list:
            fixed_idx = [i_n for i_n in range(len(node)) if node[i_n] not in ctrl_bo_list]  # fixed binding index
            ctrl_idx = [i_n for i_n in range(len(node)) if node[i_n] in ctrl_bo_list]  # control binding index
            for leaf in self.node_list:
                if (all([node[fi] == leaf[fi] for fi in fixed_idx])  # fixed object is not changed
                        and all([node[ci] == leaf[ci] or (node[ci] not in leaf) for ci in
                                 ctrl_idx])  # controlled binder is empty or not changed
                        and sum([node[ci] != leaf[ci] for ci in ctrl_idx])==1  # at list one controlled binder is changed
                ):
                    self.node_dict[node].append(leaf)
                    self.node_dict[leaf].append(node)
        for node in self.node_list:
            self.node_dict[node] = list(set(self.node_dict[node]))

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
            if self.goal_cost_dict[leaf]<self.goal_cost_dict[node]+margin:
                neighbor_valid += [leaf]
        return neighbor_valid

    def reset_valid_node(self, margin=0, node=None):
        if node == None:
            node = self.initial_state.onode
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
        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.onode), self.score_graph(goal_nodes)
        self.reset_valid_node(tree_margin)
        self.depth_min = self.goal_cost_dict[initial_state.onode]
        self.max_depth = self.depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()

    def add_node_queue_leafs(self, snode):
        graph = self.graph
        self.dict_lock.acquire()
        snode.idx = self.snode_counter.value
        self.snode_dict[snode.idx] = snode
        self.snode_counter.value = self.snode_counter.value+1
        self.dict_lock.release()
        state = snode.state
        leafs = self.valid_node_dict[state.onode]
        Q_dict = joint_list2dict(state.Q, graph.joint_names)
        if len(leafs) == 0:
            return snode
        for leaf in leafs:
            depth = len(snode.parents) + 1
            expected_depth = depth + self.goal_cost_dict[leaf]
            if expected_depth > self.max_depth:
                continue
            available_binding_dict = {oname:
                                          get_available_bindings(graph, oname, boname, sbinding[1], sbinding[2],
                                                                 Q_dict=Q_dict)\
                                              if sboname!=boname else [sbinding[1:]]
                                      for oname, boname, sboname, sbinding
                                      in zip(graph.object_list, leaf, state.onode, state.node)}
            if not all([len(abds)>0 for abds in available_binding_dict.values()]):
                print("============== Non-available transition: Break =====================")
                break
            for _ in range(self.sample_num):
                to_state = state.copy(graph)
                to_node = tuple([(((oname,)+\
                                   random.choice(available_binding_dict[oname]))
                                  if sboname!=boname else sbinding)
                                 for oname, boname, sboname, sbinding
                                 in zip(graph.object_list, leaf, state.onode, state.node)])
                to_state.set_node(to_node, graph)
                redundancy_dict = {}
                for from_binding, to_binding in zip(state.node, to_node):
                    obj = graph.object_dict[from_binding[0]]
                    to_ap = obj.action_points_dict[to_binding[1]]
                    to_binder = graph.binder_dict[to_binding[2]]
                    redundancy_tot = combine_redundancy(to_ap, to_binder)
                    redundancy = sample_redundancy(redundancy_tot)
                    redundancy_dict[from_binding[0]] = redundancy

                # self.snode_queue.put((snode, state, to_state), expected_depth * self.DSCALE - depth) ## breadth-first
                self.snode_queue.put(((expected_depth - depth) * self.DSCALE + depth, (snode, state, to_state, redundancy_dict))) ## greedy
        return snode

    @record_time
    def search_graph(self, initial_state, goal_nodes,
                     tree_margin=0, depth_margin=0, sample_num=30,
                     terminate_on_first=True, N_search=100, N_loop=1000, N_agents=None, multiprocess=False,
                     display=False, dt_vis=None, verbose=False, print_expression=False, **kwargs):
        self.sample_num = sample_num
        self.t0 = time.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, list(set([node2onode(self.graph, gnode) for gnode in goal_nodes])), tree_margin, depth_margin)
        snode_root = SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                leafs_P=[self.WEIGHT_DEFAULT] * len(self.valid_node_dict[initial_state.onode]),
                                depth=0, edepth=self.goal_cost_dict[initial_state.onode])

        if multiprocess:
            if display:
                print("Cannot display motion in multiprocess")
                display = False
            if N_agents is None:
                N_agents = cpu_count()
            print("Use {}/{} agents".format(N_agents, cpu_count()))
            self.snode_counter = self.manager.Value('i', 0)
            self.search_counter = self.manager.Value('i', 0)
            self.stop_now = self.manager.Value('i', 0)
            self.snode_dict = self.manager.dict()
            self.snode_queue = self.manager.PriorityQueue()
            self.add_node_queue_leafs(snode_root)
            self.proc_list = [Process(
                target=self.__search_loop,
                args=(terminate_on_first, N_search, N_loop, False, dt_vis, verbose, print_expression),
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
            self.snode_queue = PriorityQueue()
            self.add_node_queue_leafs(snode_root)
            self.__search_loop(terminate_on_first, N_search, N_loop, display, dt_vis, verbose, print_expression, **kwargs)

    @record_time
    def __search_loop(self, terminate_on_first, N_search, N_loop,
                      display=False, dt_vis=None, verbose=False, print_expression=False,
                      traj_count=DEFAULT_TRAJ_COUNT, **kwargs):
        loop_counter = 0
        while self.snode_counter.value < N_search and loop_counter < N_loop and not self.stop_now.value:
            loop_counter += 1
            self.que_lock.acquire()
            if self.snode_queue.empty():
                break
            snode, from_state, to_state, redundancy_dict = self.snode_queue.get()[1]
            self.search_counter.value = self.search_counter.value + 1
            self.que_lock.release()
            self.gtimer.tic("test_transition")
            traj, new_state, error, succ = self.graph.test_transition(from_state, to_state,
                                                                      redundancy_dict=redundancy_dict,
                                                                      display=display, dt_vis=dt_vis,
                                                                      print_expression=print_expression, **kwargs)
            ret = False
            if succ:
                depth_new = len(snode.parents) + 1
                snode_new = SearchNode(
                    idx=0, state=new_state, parents=snode.parents + [snode.idx], leafs=[], leafs_P=[],
                    depth=depth_new, edepth=depth_new+self.goal_cost_dict[new_state.onode], redundancy=redundancy_dict)
                snode_new.set_traj(traj, traj_count=traj_count)
                snode_new = self.add_node_queue_leafs(snode_new)
                snode.leafs += [snode_new.idx]
                self.snode_dict[snode.idx] = snode
                if self.check_goal(snode_new.state.onode, self.goal_nodes):
                    ret = True
            simtime = self.gtimer.toc("test_transition")
            if verbose:
                print('\n{} - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} ({} s, steps/err: {}({} ms)/{})'.format(
                    "success" if succ else "fail",
                    int(self.goal_cost_dict[from_state.onode]), int(self.goal_cost_dict[to_state.onode]),
                    int(self.init_cost_dict[from_state.onode]), int(self.init_cost_dict[to_state.onode]),
                    snode.idx, snode_new.idx if succ else "", round(time.time() - self.t0, 2),
                    len(traj), simtime,
                    error))
                print('node: {}->{}'.format(from_state.onode, to_state.onode))
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
            if self.check_goal(state.onode, self.goal_nodes):
                self.idx_goal += [i]
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
        return schedule_dict

    def quiver_snodes(self, figsize=(10,10)):
        import matplotlib.pyplot as plt
        N_plot = self.snode_counter.value
        snode_vec = [v for k,v in sorted(self.snode_dict.items(), key=lambda x: x)]
        cost_vec = [self.goal_cost_dict[snode.state.onode] for snode in snode_vec[1:N_plot]]
        parent_vec = [self.goal_cost_dict[self.snode_dict[snode.parents[-1]].state.onode] for snode in snode_vec[1:N_plot]]
        plt.figure(figsize=figsize)
        X = list(range(1,N_plot))
        plt.quiver(X, parent_vec,
                   [0]*(N_plot-1),
                   np.subtract(cost_vec, parent_vec),
                   angles='xy', scale_units='xy', scale=1)
        plt.plot(X, cost_vec,'.')
        plt.plot(X, parent_vec,'.')
        plt.axis([0,N_plot+1,-0.5,4.5])


def get_available_binder_dict(graph, oname_list, bname_list):
    available_binder_dict = defaultdict(list)
    for bname in bname_list:
        for oname in oname_list:
            pass_now = False
            for binder_name in graph.object_binder_dict[bname]:
                binder = graph.binder_dict[binder_name]
                for ap in graph.object_dict[oname].action_points_dict.values():
                    if binder.check_type(ap):
                        available_binder_dict[oname].append(binder.object.name)
                        pass_now = True
                        break
                if pass_now:
                    break
    return available_binder_dict


def get_available_bindings(graph, oname, boname, ap_exclude, bd_exclude, Q_dict):
    obj = graph.object_dict[oname]
    ap_dict = obj.action_points_dict
    apk_list = ap_dict.keys()
    bd_list = [graph.binder_dict[bname] for bname in graph.object_binder_dict[boname]
               if graph.binder_dict[bname].check_available(Q_dict)]

    apk_exclude = obj.get_conflicting_handles(ap_exclude)
    ap_list = [ap_dict[apk] for apk in apk_list if apk not in apk_exclude]
    bd_exclude = graph.binder_dict[bd_exclude]
    if bd_exclude in bd_list:
        bd_list.remove(graph.binder_dict[bd_exclude])

    available_bindings = []
    for bd in bd_list:
        for ap in ap_list:
            if bd.check_type(ap):
                available_bindings.append((ap.name, bd.name))
    if not available_bindings:
        print("=================================")
        print("=================================")
        print("=================================")
        print("Not available:{}-{}".format(oname,boname))
        print("np_exclude:{}".format(ap_exclude))
        print("bd_exclude:{}".format(bd_exclude.name))
        print("=================================")
        print("=================================")
        print("=================================")
    return available_bindings

def combine_redundancy(to_ap, to_binder):
    redundancy_bd = to_binder.get_redundancy()
    redundancy_ap = to_ap.get_redundancy()
    redundancy_tot = deepcopy(redundancy_bd)
    for k in redundancy_ap.keys():
        if k in redundancy_tot:
            redundancy_tot[k] = tuple(np.add(redundancy_tot[k], redundancy_ap[k]))
        else:
            redundancy_tot[k] = redundancy_ap[k]
    return redundancy_tot

def sample_redundancy(redundancy_tot):
    return {k: random.uniform(*red) for k, red in redundancy_tot.items()}