from .constraint_action import *
from .constraint_object import *
from .etasl import *
from .geometry import *
from .global_config import *
from .joint_utils import *
from .ros_rviz import *
from .utils_graph import *
from .utils import *
from urdf_parser_py.urdf import URDF
from pkg.plot_utils import *
from threading import Thread
from multiprocessing import Process, Lock, Manager
from pkg.panda_ros_interface import *
from nrmkindy.indy_script import *

# try:
#     from queue import PriorityQueue
# except:
#     from Queue import PriorityQueue
from multiprocessing.managers import SyncManager
class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)

PROC_MODE = True
PRINT_LOG = False

class SearchNode:
    def __init__(self, idx, state, parents, leafs, leafs_P, depth=None, edepth=None):
        self.idx, self.state, self.parents, self.leafs, self.leafs_P, self.depth, self.edepth = \
            idx, state, parents, leafs, leafs_P, depth, edepth
class State:
    def __init__(self, node, obj_pos_dict, Q):
        self.node = node
        self.obj_pos_dict = obj_pos_dict
        self.Q = Q
    
    def get_tuple(self):
        return (self.node, self.obj_pos_dict, self.Q)
    
    def copy(self):
        return State(self.node, self.obj_pos_dict, self.Q)
        
    def __str__(self):
        return str((self.node, 
                    {k: str(np.round(v, 2)) for k, v in self.obj_pos_dict.items()} if self.obj_pos_dict is not None else None,  
                    str(np.round(self.Q, 2)) if self.Q is not None else None))
class ConstraintGraph:
    DQ_MAX = np.deg2rad(90)
    WEIGHT_DEFAULT = 2.0
#     WEIGHT_INIT = 5.0
    WEIGHT_GOAL = 2.0
    WEIGHT_MAX = 10.0
    JOINT_DECAY = 0.90
    NODE_DECAY = 0.9
    JOINT_MULTIPLIER = 2.0
    DEFAULT_TRANSIT_COST = 1.0
    MAX_AGENT_DELAY = 4
    MAX_LEN_SIM_Q = 50
    DSCALE = 1e4

    def __init__(self, urdf_path, joint_names, link_names, urdf_content=None,
                 connect_robots=False,
                 indy_ip='141.223.193.55', btype=BlendingType.DUPLICATE, 
                 indy_grasp=lambda:None, indy_release=lambda:None):
        self.joint_num = len(joint_names)
        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        set_parent_joint_map(urdf_content)
        self.joint_names = joint_names
        self.link_names = link_names
        self.collision_items_dict = {}
        self.binder_dict = {}
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}
        self.lock = Lock()
        self.sim_q_lock = Lock()
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.connect_robots = connect_robots

        if connect_robots:
            self.ps = PandaStateSubscriber(interface_name=PANDA_SIMULATION_INTERFACE)
            self.ps.start_subsciption()
            self.pc = PandaControlPublisher(interface_name=PANDA_SIMULATION_INTERFACE)

            self.indy_grasp = indy_grasp
            self.indy_release = indy_release
            try: end_script()
            except: pass
            self.indy_ip = indy_ip
            self.btype = btype
            config_script(NAME_INDY_7)
            start_script(indy_ip)
            blending_type(btype)
            task_vel(0.1, 0.2, 20, 20)
            self.indy_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'indy' in jname]
            self.panda_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'panda' in jname]
    
    def __del__(self):
        try: end_script()
        except: pass

    @record_time
    def set_simulation(self, nWSR=50, cputime=200, regularization_factor= 1e-4):
        # prepare ros
        self.pub, self.joints, self.rate = get_publisher(self.joint_names)
        # prepare visualization markers
        self.marker_list = set_markers(self.collision_items_dict, self.joints, self.joint_names, self.link_names, self.urdf_content)
        set_simulation_config(joint_names = self.joint_names, link_names = self.link_names, 
                              urdf_content = self.urdf_content, urdf_path = self.urdf_path,
                              collision_items_dict=self.collision_items_dict,
                              nWSR=nWSR, cputime=cputime, regularization_factor= regularization_factor)
        self.init_text = get_init_text()
        self.show_pose(np.zeros(len(self.joint_names)))

    @record_time
    def set_collision_items(self, collision_items_dict):
        self.collision_items_dict = collision_items_dict
        self.collision_items_list = []
        self.collision_items_idx_dict = {}
        for v in collision_items_dict.values():
            for ctem in v:
                self.collision_items_idx_dict[ctem.name] = len(self.collision_items_list)
                self.collision_items_list += [ctem]
        return self.collision_items_dict

    @record_time
    def add_collision_items(self, link_name, collision_Items):
        self.collision_items_dict[link_name] += collision_Items
        for ctem in collision_Items:
            self.collision_items_idx_dict[ctem.name] = len(self.collision_items_list)
            self.collision_items_list += [ctem]

    @record_time
    def add_binder(self, name, binder):
        self.binder_dict[name] = binder

    @record_time
    def register_binder(self, name, _type, link_name=None, object_name=None, **kwargs):
        if object_name is None:
            object_name = name

        _object = self.get_object_by_name(object_name)
        
        if _object:
            link_name = _object.link_name
        else:
            _object=None
            assert link_name is not None, "The object should be registered first or give link name"
            assert "point" in kwargs, "The object should be registered first or give interaction point"
        self.binder_dict[name] = _type(_object=_object, 
                                       name=name, link_name=link_name, 
                                       urdf_content=self.urdf_content, collision_items_dict=self.collision_items_dict, **kwargs)
        if _object is None:
            self.add_collision_items(link_name, [self.binder_dict[name].object])

    @record_time
    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0], 
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

    @record_time
    def register_object(self, name, _type, binding=None, **kwargs):
        _object = self.get_object_by_name(name)
        self.object_dict[name] = _type(_object, **kwargs)
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0], 
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

    @record_time
    def get_object_by_name(self, name):
        if name in self.collision_items_idx_dict:
            return self.collision_items_list[self.collision_items_idx_dict[name]]
        else:
            return None

    @record_time
    def update_handles(self):
        self.handle_dict = {}
        self.handle_list = []
        self.object_list = sorted(self.object_dict.keys())
        for k in self.object_list:
            v = self.object_dict[k]
            ap_list = v.get_action_points()
            self.handle_dict[k] = []
            for ap in ap_list.keys():
                self.handle_dict[k] += [ap]
                self.handle_list += [(k, ap)]

    @record_time
    def build_graph(self, update_handles=True):
        if update_handles:
            self.update_handles()
        self.node_dict = {}
        bindings = get_all_mapping(self.handle_dict.keys(), self.binder_dict.keys())
        handle_combinations = list(product(*[self.handle_dict[obj] for obj in self.object_list]))
        uniq_binders = self.get_unique_binders()
        ctrl_binders = self.get_controlled_binders()
        self.node_list = []
        self.node_dict = {}
        for binding in bindings:
            if all([np.sum([bd == ub for bd in binding.values()]) <= 1 for ub in uniq_binders]):
                for hc in handle_combinations:
                    node = []
                    add_ok = True
                    for i_o, obj in zip(range(len(self.object_list)), self.object_list):
                        hndl = self.object_dict[obj].action_points_dict[hc[i_o]]
                        binder = self.binder_dict[binding[obj]]
                        if binder.check_type(hndl):
                            node += [(obj, hc[i_o], binding[obj])]
                        else:
                            add_ok = False
                    if add_ok:
                        node = tuple(node)
                        self.node_list += [node]
                        self.node_dict[node] = []
        for node in self.node_list:
            fixed_in = [i_n for i_n in range(len(node)) if node[i_n][2] not in ctrl_binders]
            ctrl_in = [i_n for i_n in range(len(node)) if node[i_n][2] in ctrl_binders]
            if ctrl_in:
                bd_fixed = [node[idx] for idx in fixed_in]
                bd_ctrl = [node[idx][2] for idx in ctrl_in]
                pt_ctrl = [node[idx][1] for idx in ctrl_in]
                obj_ctrl = [node[idx][0] for idx in ctrl_in]
                nodes_same_fix = [nd for nd in self.node_list if [nd[idx] for idx in fixed_in] == bd_fixed]
                nodes_diff_ctrl = [nd for nd in nodes_same_fix if [nd[idx][2] for idx in ctrl_in] != bd_ctrl]
                nodes_diff_pt = [nd for nd in nodes_diff_ctrl if [nd[idx][1] for idx in ctrl_in] != pt_ctrl]
                nodes_neighbor = [nd for nd in nodes_diff_pt if len(
                    set([bd[2] for bd, bd0 in zip(nd, node) if bd != bd0]).intersection(bd_ctrl)) == 0]
                self.node_dict[node] += nodes_neighbor
                for nd_n in nodes_neighbor:
                    self.node_dict[nd_n] += [node]

        for node in self.node_dict.keys():
            if node in self.node_dict[node]:
                self.node_dict[node].remove(node)
            self.node_dict[node] = [node] + list(set(self.node_dict[node]))

    @record_time
    def get_unique_binders(self):
        uniq_binders = []
        for k_b, binder in self.binder_dict.items():
            if not binder.multiple:
                uniq_binders += [k_b]
        return uniq_binders

    @record_time
    def get_controlled_binders(self):
        controlled_binders = []
        for k_b, binder in self.binder_dict.items():
            if binder.controlled:
                controlled_binders += [k_b]
        return controlled_binders

    @record_time
    def set_object_state(self, state):
        bd_list = list(state.node)
        bd_list_done = []
        while bd_list:
            bd = bd_list.pop(0)
            binder = self.binder_dict[bd[2]]
            if binder.object in [self.object_dict[bd_tmp[0]].object for bd_tmp in bd_list]:
                bd_list += [bd] # prevent using previous info ( move back to end )
            else:
                obj = self.object_dict[bd[0]]
                frame = state.obj_pos_dict[bd[0]]
                binder.link_name = binder.object.link_name # sync linke name with parent
                obj.set_state(frame, binder.link_name, bd[1], bd[2], 
                              self.collision_items_dict)
                bd_list_done += [bd]

    @record_time
    def get_object_state(self):
        node = ()
        pose_dict = {}
        for k in self.object_list:
            v = self.object_dict[k]
            node += ((k,) + v.binding,)
            pose_dict[k] = v.object.get_frame()
        return node, pose_dict

    @record_time
    def simulate_transition(self, from_state=None, to_state=None, display=False, execute=False, error_skip=1e-4, lock=False,
                            vel_conv=1e-2, err_conv=1e-4, N=1, dt=1e-2, N_step=10, **kwargs):
        gtimer = GlobalTimer.instance()
        gtimer.tic("start set transition")
        if from_state is not None:
            pos_start = from_state.Q
            self.set_object_state(from_state)

        additional_constraints = ""
        binding_list = []
        if to_state.node is not None:
            for bd0, bd1 in zip(from_state.node, to_state.node):
                if bd0[2] != bd1[2]:
                    additional_constraints += self.binder_dict[bd1[2]].make_constraints(self.object_dict[bd1[0]], bd1[1])
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"

        if additional_constraints=="" and to_state.Q is not None and np.sum(np.abs(np.subtract(to_state.Q,from_state.Q)))>1e-2:
#             print('set joint constraint')
            additional_constraints=make_joint_constraints(joint_names=self.joint_names)
            kwargs.update(dict(inp_lbl=['target_%s'%jname for jname in self.joint_names], 
                               inp=list(to_state.Q)
                              ))                 

        if not (display or execute):
            if lock:
                self.lock.release()
            gtimer.toc("start set transition")
            gtimer.tic("set_simulate fun")
            e = set_simulate(self.init_text, additional_constraints=additional_constraints,
                             initial_jpos=np.array(pos_start), vel_conv=vel_conv, err_conv=err_conv, 
                             N=N, dt=dt, **kwargs)
            gtimer.toc("set_simulate fun")
            gtimer.tic("post")
            if lock:
                self.lock.acquire()
            if from_state is not None:
                self.set_object_state(from_state)
            if hasattr(e, 'error') and e.error<err_conv:
                success = True
                for bd in binding_list:
                    self.rebind(bd, e.joint_dict_last)

            else:
                success = False
        else:
            success = False
            thread_display = None
            thread_execute = None
            e = prepare_simulate(self.init_text, additional_constraints=additional_constraints,
                                 vel_conv=vel_conv, err_conv=err_conv)
            for i_sim in range(int(N/N_step)):
                e = do_simulate(e, initial_jpos=np.array(pos_start), N=N_step, dt=dt, **kwargs)
                pos_start = e.POS[-1]
                if display:
                    if thread_display is not None:
                        thread_display.join()
                    thread_display = Thread(target=show_motion, 
                                            args=(e.POS, self.marker_list, self.pub,
                                                  self.joints, self.joint_names),
                                            kwargs={'error_skip':1e-4, 'period':dt}
                                            )
                    thread_display.start()

                if execute:
                    if thread_execute is not None:
                        thread_execute.join()
                    thread_execute = Thread(target=self.execute, args=(e.POS,dt))
                    thread_execute.start()
                    
                if hasattr(e, 'error') and e.error<err_conv:
                    success = True
                    break
            if thread_display is not None:
                thread_display.join()
            if thread_execute is not None:
                thread_execute.join()
            gtimer.tic("post")
            if from_state is not None:
                self.set_object_state(from_state)
            if success:
                if execute:
                    self.execute_grip(to_state)
                for bd in binding_list:
                    self.rebind(bd, e.joint_dict_last)
        
        node, obj_pos_dict = self.get_object_state()
        end_state = State(node, obj_pos_dict, list(e.POS[-1]))
        gtimer.toc("post")
        # print(gtimer)
        return e, end_state, success

    @record_time
    def execute_grip(self, state):
        if self.connect_robots:
            indy_grip = False
            panda_grip = False
            for bd in state.node:
                bind_link_name = self.binder_dict[bd[2]].object.link_name
                if 'indy' in bind_link_name:
                    indy_grip = True
                elif 'panda' in bind_link_name:
                    panda_grip = True
            if indy_grip:
                self.indy_grasp()
            else:
                self.indy_release()
            if panda_grip:
                self.pc.close_finger()
            else:
                self.pc.open_finger()

    @record_time
    def execute(self, POS, dt=1e-2):
        if self.connect_robots:
            for pos in POS:
                amovej(JointPos(*np.rad2deg(pos[self.indy_idx])))
                self.pc.joint_move_arm(pos[self.panda_idx])
                timer.sleep(dt)

    @record_time
    def rebind(self, binding, joint_dict_last, execute=False):
        binder = self.binder_dict[binding[2]]
        object_tar = self.object_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict_last)
        for binder_sub in [k for k,v in self.binder_dict.items() if v.object == object_tar.object]:
            for binding_sub in [(k,v.binding[0]) for k, v in self.object_dict.items()
                                if v.binding[1] == binder_sub]:
                binding_sub += (binder_sub,)
                self.rebind(binding_sub, joint_dict_last)

    @record_time
    def score_graph(self, goal_node):
        frontier = PriorityQueue()
        frontier.put(goal_node, 0)
        came_from = {}
        node_cost_dict = {}
        came_from[goal_node] = None
        node_cost_dict[goal_node] = 0

        while not frontier.empty():
            current = frontier.get()

            for next in self.node_dict[current]:
                if next == current:
                    continue
                new_cost = node_cost_dict[current] + ConstraintGraph.DEFAULT_TRANSIT_COST
                if next not in node_cost_dict or new_cost < node_cost_dict[next]:
                    node_cost_dict[next] = new_cost
                    priority = new_cost
                    frontier.put(next, priority)
                    came_from[next] = current
        return node_cost_dict

    @record_time
    def get_valid_neighbor(self, node, margin=0):
        neighbor = self.node_dict[node]
        neighbor_valid = []
        for leaf in neighbor:
            if self.goal_cost_dict[leaf]<=self.goal_cost_dict[node]+margin:
                neighbor_valid += [leaf]
        return neighbor_valid

    @record_time
    def reset_valid_node(self, margin=0, node=None):
        if node == None:
            node = self.initial_state.node
            self.valid_node_dict = {self.goal_state.node:[]}
        if node == self.goal_state.node or node in self.valid_node_dict:
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
    def init_search(self, initial_state, goal_state, tree_margin, depth_margin):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.node), self.score_graph(goal_state.node)
        self.reset_valid_node(tree_margin)
        self.depth_min = self.goal_cost_dict[initial_state.node]
        self.max_depth = self.depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()

    @record_time
    def add_node_queue_leafs(self, snode):
        snode.idx = self.snode_counter.value
        self.snode_dict[snode.idx] = snode
        self.snode_counter.value = self.snode_counter.value+1
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
            to_state = state.copy()

            if leaf == state.node:
                dQ = (1 - 2 * np.random.rand(self.DOF)) * ConstraintGraph.DQ_MAX
                to_state.Q = np.sum([state.Q, dQ], axis=0)
            else:
                to_state.node = leaf
            self.snode_queue.put((snode, state, to_state), expected_depth * self.DSCALE - depth)
        return snode

    @record_time
    def search_graph(self, initial_state, goal_state,
                     tree_margin=0, depth_margin=0, joint_motion_num=10,
                     terminate_on_first=True, N_search=100, N_loop=1000,
                     display=False, verbose=False, **kwargs):
        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, goal_state, tree_margin, depth_margin)

        self.snode_counter = self.manager.Value('i', 0)
        self.stop_now =  self.manager.Value('i', 0)
        self.snode_dict = {}
        self.snode_queue = PriorityQueue()
        self.add_node_queue_leafs(SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                              leafs_P=[ConstraintGraph.WEIGHT_DEFAULT] * len(
                                                  self.valid_node_dict[initial_state.node])))
        self.__search_loop(terminate_on_first, N_search, N_loop, display, verbose, **kwargs)

    @record_time
    def search_graph_mp(self, initial_state, goal_state,
                        tree_margin=0, depth_margin=0, joint_motion_num=10,
                        terminate_on_first=True, N_search=100, N_loop=1000, N_agents=8,
                        display=False, verbose=False, **kwargs):
        if display:
            print("Cannot display motion in multiprocess")

        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, goal_state, tree_margin, depth_margin)

        self.snode_counter = self.manager.Value('i', 0)
        self.stop_now =  self.manager.Value('i', 0)
        self.snode_dict = self.manager.dict()
        self.snode_queue = self.manager.PriorityQueue()
        self.add_node_queue_leafs(SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                              leafs_P=[ConstraintGraph.WEIGHT_DEFAULT] * len(
                                                  self.valid_node_dict[initial_state.node])))

        self.proc_list = [Process(target=self.__search_loop, args=(terminate_on_first, N_search, N_loop, False, verbose), kwargs=kwargs) for id_agent in range(N_agents)]
        for proc in self.proc_list:
            proc.start()

        for proc in self.proc_list:
            proc.join()

    @record_time
    def __search_loop(self, terminate_on_first, N_search, N_loop, display, verbose, **kwargs):
        loop_counter = 0
        while self.snode_counter.value < N_search and loop_counter < N_loop and not self.stop_now.value:
            loop_counter += 1
            if self.snode_queue.empty():
                break
            snode, from_state, to_state = self.snode_queue.get()
            e, new_state, succ = self.simulate_transition(from_state, to_state, display=display, lock=False, **kwargs)
            ret = False
            if succ:
                snode_new = SearchNode(idx=0, state=new_state, parents=snode.parents + [snode.idx],
                                       leafs=[],
                                       leafs_P=[])
                snode_new = self.add_node_queue_leafs(snode_new)
                snode.leafs += [snode_new.idx]
                self.snode_dict[snode.idx] = snode
                if self.check_goal(snode_new.state, self.goal_state):
                    ret = True
            if verbose:
                print('\n{} - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} ({} s)'.format(
                    "success" if succ else "fail",
                    int(self.goal_cost_dict[from_state.node]), int(self.goal_cost_dict[to_state.node]),
                    int(self.init_cost_dict[from_state.node]), int(self.init_cost_dict[to_state.node]),
                    snode.idx, snode_new.idx if succ else "", round(timer.time() - self.t0, 2)))
                print('node: {}->{}'.format(from_state.node, to_state.node))
                print('=' * 150)
            if terminate_on_first and ret:
                self.stop_now.value = 1
                break

    @record_time
    def find_schedules(self):
        self.idx_goal = []
        schedule_dict = {}
        for i in range(self.snode_counter.value):
            snode = self.snode_dict[i]
            state = snode.state
            if self.check_goal(state, self.goal_state):
                self.idx_goal += [i]
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
        return schedule_dict

    @record_time
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    @record_time
    def replay(self, schedule, N=400, dt=0.005, execute=False,**kwargs):
        state_cur = self.snode_dict[schedule[0]].state
        for i_state in schedule[1:]:
            state_new = self.snode_dict[i_state].state
            print('')
            print('-'*20)
            print("{}-{}".format(i_state, state_new.node))
            e, new_state, succ = self.simulate_transition(state_cur, state_new, display=True, execute=execute, N=N, dt=dt,**kwargs)
            state_cur = state_new
        return e

    @record_time
    def check_goal(self, state, goal):
        return all([g is None or s==g for s,g in zip(state.get_tuple(), goal.get_tuple())])

    @record_time
    def print_snode_list(self):
        for i_s, snode in sorted(self.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

    @record_time
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

    @record_time
    def show_pose(self, pose, execute=False):
        show_motion([pose], self.marker_list, self.pub, self.joints, self.joint_names)
        if execute:
            self.execute([pose])
        