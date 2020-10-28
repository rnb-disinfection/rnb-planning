import sys
sys.setrecursionlimit(10000)

from .ros_rviz import *
from .utils_graph import *
from urdf_parser_py.urdf import URDF
from multiprocessing import Process, Lock, cpu_count
from .panda_ros_interface import *
from .panda_repeater import *
from .indy_repeater import indytraj_client
from .environment_builder import *
from .gjk import *
from copy import deepcopy

PANDA_ROS = False
PORT_REPEATER = 1189
CONTROL_FREQ = 100

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
TRAJ_COUNT = 1
TRAJ_RADII = np.deg2rad(30)

class SearchNode:
    def __init__(self, idx, state, parents, leafs, leafs_P, depth=None, edepth=None):
        self.idx, self.state, self.parents, self.leafs, self.leafs_P, self.depth, self.edepth = \
            idx, state, parents, leafs, leafs_P, depth, edepth
        self.traj = None

    def set_traj(self, traj_full):
        traj_step = max(1,len(traj_full)/TRAJ_COUNT)
        self.traj = np.array(list(reversed(traj_full[::-traj_step])))

    def get_traj(self):
        return self.traj

    def copy(self):
        return SearchNode(self.idx, State(self.state.node, self.state.obj_pos_dict, self.state.Q),
                          self.parents, self.leafs, self.leafs_P, self.depth, self.edepth)


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
    DQ_MAX = np.deg2rad(45)
    WEIGHT_DEFAULT = 2.0
    DEFAULT_TRANSIT_COST = 1.0
    DSCALE = 1e4

    def __init__(self, urdf_path, joint_names, link_names, urdf_content=None,
                 connect_panda=False, connect_indy=False,
                 indy_ip='192.168.0.63', indy_joint_vel_level=3, indy_task_vel_level=3,
                 indy_grasp_DO=8):
        self.joint_num = len(joint_names)
        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        self.ghnd = GeometryHandle.instance(urdf_content)
        set_parent_joint_map(urdf_content)
        set_link_adjacency_map(urdf_content)
        self.min_distance_map = set_min_distance_map(link_names, urdf_content)
        self.joint_names = joint_names
        self.link_names = link_names
        self.binder_dict = {}
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}
        self.dict_lock = Lock()
        self.que_lock = Lock()
        self.manager = PriorityQueueManager()
        self.manager.start()
        self.connect_panda = connect_panda
        self.connect_indy = connect_indy
        self.gtimer=GlobalTimer.instance()
        self.joint_limits = np.array([(self.urdf_content.joint_map[jname].limit.lower,
                                       self.urdf_content.joint_map[jname].limit.upper) for jname in self.joint_names])

        self.indy = None
        self.panda = None
        self.indy_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'indy' in jname]
        self.panda_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'panda' in jname]
        if connect_panda:
            if PANDA_ROS:
                self.ps = PandaStateSubscriber(interface_name=PANDA_SIMULATION_INTERFACE)
                self.ps.start_subsciption()
                self.pc = PandaControlPublisher(interface_name=PANDA_SIMULATION_INTERFACE)
            else:
                self.panda = PandaRepeater()

        if connect_indy:
            self.indy_speed = 180
            self.indy_acc = 360
            self.indy_ip = indy_ip
            self.indy_joint_vel_level = indy_joint_vel_level
            self.indy_task_vel_level = indy_task_vel_level
            self.indy = indytraj_client(server_ip=indy_ip, robot_name="NRMK-Indy7")
            with self.indy:
                self.indy.set_collision_level(5)
                self.indy.set_joint_vel_level(indy_joint_vel_level)
                self.indy.set_task_vel_level(indy_task_vel_level)
                self.indy.set_joint_blend_radius(20)
                self.indy.set_task_blend_radius(0.2)
            self.indy.indy_grasp_DO = indy_grasp_DO

    def reset_panda(self):
        if self.connect_panda:
            if PANDA_ROS:
                self.ps = PandaStateSubscriber(interface_name=PANDA_SIMULATION_INTERFACE)
                self.ps.start_subsciption()
                self.pc = PandaControlPublisher(interface_name=PANDA_SIMULATION_INTERFACE)
            else:
                self.panda.set_alpha_lpf(self.panda.alpha_lpf)
                self.panda.set_d_gain(self.panda.d_gain)
                self.panda.set_k_gain(self.panda.k_gain)

    def reset_indy(self):
        if self.connect_indy:
            self.indy = indytraj_client(server_ip=self.indy_ip, robot_name="NRMK-Indy7")
            with self.indy:
                self.indy.set_collision_level(5)
                self.indy.set_joint_vel_level(self.indy_joint_vel_level)
                self.indy.set_task_vel_level(self.indy_task_vel_level)
                self.indy.set_joint_blend_radius(20)
                self.indy.set_task_blend_radius(0.2)
            self.indy_grasp_DO = self.indy_grasp_DO

    def reset_robots(self):
        self.reset_panda()
        self.reset_indy()

    def __del__(self):
        try:
            pass
        except: pass

    @record_time
    def set_rviz(self):
        # prepare ros
        self.pub, self.joints, self.rate = get_publisher(self.joint_names, control_freq=CONTROL_FREQ)
        # prepare visualization markers
        self.marker_list = set_markers(self.ghnd, self.joints, self.joint_names)
        self.show_pose(np.zeros(len(self.joint_names)))

    @record_time
    def set_planner(self, planner):
        self.planner = planner
        planner.update_gtems()
        planner.set_object_dict(self.object_dict)
        planner.set_binder_dict(self.binder_dict)

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
                                       urdf_content=self.urdf_content, **kwargs)
        if _object is None:
            self.binder_dict[name].object

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

    def register_object_gen(self, objectPose_dict_mv, object_generators, binder_dict, object_dict, ref_tuple, link_name="world"):
        objectPose_dict_mv.update({ref_tuple[0]: ref_tuple[1]})
        xyz_rpy_mv_dict, put_point_dict, _ = calc_put_point(objectPose_dict_mv, object_generators, object_dict, ref_tuple)

        for mname, mgen in object_generators.items():
            if mname in xyz_rpy_mv_dict and mname not in self.ghnd.NAME_DICT:
                xyz_rpy = xyz_rpy_mv_dict[mname]
                mgen(*xyz_rpy, name=mname, link_name=link_name,
                     color=(0.3, 0.3, 0.8, 1), collision=True, fixed=False)

        for bname, bkwargs in binder_dict.items():
            if bname not in self.binder_dict:
                self.register_binder(name=bname, **bkwargs)

        for mtem, xyz_rpy in xyz_rpy_mv_dict.items():
            if mtem in put_point_dict and mtem not in self.object_dict:
                self.register_object(mtem, binding=(put_point_dict[mtem], "floor"), **object_dict[mtem])

        return put_point_dict


    @record_time
    def get_object_by_name(self, name):
        if name in self.ghnd.NAME_DICT:
            return self.ghnd.NAME_DICT[name]
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
                # obj_ctrl = [node[idx][0] for idx in ctrl_in]
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
                obj.set_state(frame, binder.link_name, bd[1], bd[2])
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
    def test_transition(self, from_state=None, to_state=None, display=False, dt_vis=1e-2, error_skip=0, **kwargs):
        self.gtimer = GlobalTimer.instance()

        if from_state is not None:
            self.set_object_state(from_state)

        binding_list = []
        if to_state.node is not None:
            for bd0, bd1 in zip(from_state.node, to_state.node):
                if bd0[2] != bd1[2]: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"

        Traj, LastQ, error, success = self.planner.plan_transition(from_state, to_state, binding_list, **kwargs)

        if display:
            self.show_motion(Traj, from_state, **{'error_skip':error_skip, 'period':dt_vis})

        if from_state is not None:
            self.set_object_state(from_state)

        if success:
            for bd in binding_list:
                self.rebind(bd, joint_list2dict(LastQ, self.joint_names))

        node, obj_pos_dict = self.get_object_state()
        end_state = State(node, obj_pos_dict, list(LastQ))
        return Traj, end_state, error, success

    def get_real_robot_pose(self):
        Q_indy = np.deg2rad(self.indy.connect_and(self.indy.get_joint_pos))
        if PANDA_ROS:
            raise(NotImplementedError("get pose for panda ros"))
        else:
            Q_panda = self.panda.get_qcur()

        Q_all = np.zeros(self.joint_num)
        Q_all[self.indy_idx] = Q_indy
        Q_all[self.panda_idx] = Q_panda
        return Q_all

    @record_time
    def execute_grip(self, state):
        indy_grip = False
        panda_grip = False
        for bd in state.node:
            bind_link_name = self.binder_dict[bd[2]].object.link_name
            if 'indy' in bind_link_name:
                indy_grip = True
            elif 'panda' in bind_link_name:
                panda_grip = True
        grasp_seq = [(self.indy_grasp, indy_grip), (self.panda_grasp, panda_grip)]
        grasp_seq = list(sorted(grasp_seq, key=lambda x: not x[1]))
        for grasp in grasp_seq:
            grasp[0](grasp[1])

    @record_time
    def indy_grasp(self, grasp=False):
        if self.connect_indy:
            self.indy.grasp(grasp, connect=True)

    @record_time
    def panda_grasp(self, grasp):
        if self.connect_panda:
            if PANDA_ROS:
                if grasp:
                    self.pc.close_finger()
                else:
                    self.pc.open_finger()
            else:
                self.panda.move_finger(grasp)
                
    def move_indy_async(self, *qval):
        self.indy.connect_and(self.indy.joint_move_to, qval)

    @record_time
    def rebind(self, binding, joint_dict_last):
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
        if isinstance(goal_node, list):
            score_dicts = [self.score_graph(goal) for goal in goal_node]
            score_dict = {}
            for k in score_dicts[0].keys():
                score_dict[k] = min([sdict[k] for sdict in score_dicts])
            return score_dict
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
            self.valid_node_dict = {goal:[] for goal in self.goal_nodes}
        if self.check_goal(node, self.goal_nodes) or node in self.valid_node_dict:
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

    @record_time
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
            to_state = state.copy()

            if leaf == state.node:
                dQ = (1 - 2 * np.random.rand(self.DOF)) * ConstraintGraph.DQ_MAX
                to_state.Q = np.sum([state.Q, dQ], axis=0)
                to_state.Q = np.minimum(np.maximum(to_state.Q, self.joint_limits[:,0]), self.joint_limits[:,1])
            else:
                to_state.node = leaf
            # self.snode_queue.put((snode, state, to_state), expected_depth * self.DSCALE - depth) ## breadth-first
            self.snode_queue.put((snode, state, to_state), (expected_depth - depth) * self.DSCALE + depth) ## greedy
        return snode

    @record_time
    def search_graph(self, initial_state, goal_nodes,
                     tree_margin=0, depth_margin=0, joint_motion_num=10,
                     terminate_on_first=True, N_search=100, N_loop=1000,
                     display=False, dt_vis=None, verbose=False, print_expression=False, **kwargs):
        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, goal_nodes, tree_margin, depth_margin)

        self.snode_counter = self.manager.Value('i', 0)
        self.search_counter = self.manager.Value('i', 0)
        self.stop_now =  self.manager.Value('i', 0)
        self.snode_dict = {}
        self.snode_queue = PriorityQueue()
        self.add_node_queue_leafs(SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                              leafs_P=[ConstraintGraph.WEIGHT_DEFAULT] * len(
                                                  self.valid_node_dict[initial_state.node])))
        self.__search_loop(terminate_on_first, N_search, N_loop, display, dt_vis, verbose, print_expression, **kwargs)

    @record_time
    def search_graph_mp(self, initial_state, goal_nodes,
                        tree_margin=0, depth_margin=0, joint_motion_num=10,
                        terminate_on_first=True, N_search=100, N_loop=1000, N_agents=None,
                        display=False, dt_vis=None, verbose=False, print_expression=False, **kwargs):
        if display:
            print("Cannot display motion in multiprocess")
        if N_agents is None:
            N_agents = cpu_count()
        print("Use {}/{} agents".format(N_agents, cpu_count()))

        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.init_search(initial_state, goal_nodes, tree_margin, depth_margin)

        self.snode_counter = self.manager.Value('i', 0)
        self.search_counter = self.manager.Value('i', 0)
        self.stop_now =  self.manager.Value('i', 0)
        self.snode_dict = self.manager.dict()
        self.snode_queue = self.manager.PriorityQueue()
        self.add_node_queue_leafs(SearchNode(idx=0, state=initial_state, parents=[], leafs=[],
                                              leafs_P=[ConstraintGraph.WEIGHT_DEFAULT] * len(
                                                  self.valid_node_dict[initial_state.node])))

        self.proc_list = [Process(
            target=self.__search_loop,
            args=(terminate_on_first, N_search, N_loop, False, dt_vis, verbose, print_expression),
            kwargs=kwargs) for id_agent in range(N_agents)]
        for proc in self.proc_list:
            proc.start()

        for proc in self.proc_list:
            proc.join()

    @record_time
    def __search_loop(self, terminate_on_first, N_search, N_loop,
                      display=False, dt_vis=None, verbose=False, print_expression=False, **kwargs):
        loop_counter = 0
        while self.snode_counter.value < N_search and loop_counter < N_loop and not self.stop_now.value:
            loop_counter += 1
            self.que_lock.acquire()
            if self.snode_queue.empty():
                break
            snode, from_state, to_state = self.snode_queue.get()
            self.search_counter.value = self.search_counter.value + 1
            self.que_lock.release()
            self.gtimer.tic("test_transition")
            traj, new_state, error, succ = self.test_transition(from_state, to_state, display=display, dt_vis=dt_vis,
                                                          print_expression=print_expression, **kwargs)
            ret = False
            if succ:
                snode_new = SearchNode(idx=0, state=new_state, parents=snode.parents + [snode.idx],
                                       leafs=[],
                                       leafs_P=[])
                snode_new.set_traj(traj)
                snode_new = self.add_node_queue_leafs(snode_new)
                snode.leafs += [snode_new.idx]
                self.snode_dict[snode.idx] = snode
                if self.check_goal(snode_new.state.node, self.goal_nodes):
                    ret = True
            simtime = self.gtimer.toc("test_transition")
            if verbose:
                print('\n{} - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} ({} s, steps/err: {}({} ms)/{})'.format(
                    "success" if succ else "fail",
                    int(self.goal_cost_dict[from_state.node]), int(self.goal_cost_dict[to_state.node]),
                    int(self.init_cost_dict[from_state.node]), int(self.init_cost_dict[to_state.node]),
                    snode.idx, snode_new.idx if succ else "", round(timer.time() - self.t0, 2),
                    len(traj), simtime,
                    error))
                print('node: {}->{}'.format(from_state.node, to_state.node))
                print('=' * 150)
            if terminate_on_first and ret:
                self.stop_now.value = 1
                break
        print("=====================")
        print("===== terminate =====")
        print("=====================")

    @record_time
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

    @record_time
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    @record_time
    def replay(self, schedule, N=400, dt=0.005,**kwargs):
        state_cur = self.snode_dict[schedule[0]].state
        for i_state in schedule[1:]:
            state_new = self.snode_dict[i_state].state
            print('')
            print('-'*20)
            print("{}-{}".format(i_state, state_new.node))
            traj, new_state, error, succ = self.test_transition(state_cur, state_new, display=True, N=N, dt=dt,**kwargs)
            state_cur = state_new
        return traj

    @record_time
    def check_goal(self, node, goals):
        return node in goals

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
    def show_pose(self, pose, **kwargs):
        show_motion([pose], self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    @record_time
    def show_motion(self, pose_list, from_state=None, **kwargs):
        if from_state is not None:
            self.set_object_state(from_state)
        show_motion(pose_list, self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    @record_time
    def get_traj(self, from_state, to_state,
                 T_step=5, control_freq=4e3, downsample_log2=5, playback_speed_log2=1,
                 **kwargs):
        dt = 1.0 / control_freq * (2 ** downsample_log2)
        N = int(float(T_step) / dt)
        e, new_state, error, succ = self.test_transition(from_state, to_state,
                                                      N=N, dt=dt,
                                                      display=False, dt_vis=dt, N_step=N,
                                                      **kwargs)
        print("{} Hz / {} sec : {}".format(1 / dt, N * dt, N))
        e_POS = e.POS
        for _ in range(downsample_log2):
            e_POS = interpolate_double(e_POS)
            dt = dt / 2

        for _ in range(playback_speed_log2):
            e_POS = interpolate_double(e_POS)
        return e_POS, dt, succ

    @record_time
    def get_traj_dat(self, e_POS, dt):
        e_VEL = differentiate(e_POS, dt)
        e_ACC = differentiate(e_VEL, dt)
        traj_data = np.concatenate([e_POS, e_VEL, e_ACC], axis=1)
        traj_data_list = traj_data[:].flatten().tolist()
        return traj_data_list

    def idxSchedule2SnodeScedule(self, schedule, ZERO_JOINT_POSE=None):
        snode_schedule = [self.snode_dict[i_sc] for i_sc in schedule]
        snode_schedule.append(snode_schedule[-1].copy())
        if ZERO_JOINT_POSE is None:
            ZERO_JOINT_POSE = [0]*self.joint_num
        snode_schedule[-1].state.Q = ZERO_JOINT_POSE
        snode_schedule[-1].set_traj(np.array([ZERO_JOINT_POSE]))
        return snode_schedule

    def update_obstacle(self, obsPos_dict):
        for k, v in obsPos_dict.items():
            _pos = v[:3, 3]
            for _p, _k in zip(_pos, ["obs_{name}_{axis}".format(name=k, axis=axis) for axis in "xyz"]):
                if _k in self.inp_lbl:
                    self.inp[self.inp_lbl.index(_k)] = _p

    def update_target_joint(self, idx_cur, idx_jnt, traj, joint_cur):
        error = np.sum(np.abs(joint_cur-traj[idx_cur]))
        # print("joints: {}".format(joint_cur))
        # print("traj: {}".format(traj[idx_cur]))
        # print("error: {}".format(error))
        if error < TRAJ_RADII:
            if idx_cur+1 < len(traj)-1:
                idx_cur += 1
                # print("traj: {}".format(traj[idx_cur]))
                self.inp[idx_jnt:idx_jnt+self.joint_num] = traj[idx_cur]
        return idx_cur

    def init_online_etasl(self, from_state, to_state, T_step, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5, **kwargs):
        dt = 1.0 / control_freq
        dt_sim = dt * playback_rate
        N = int(float(T_step) / dt_sim)

        if from_state is not None:
            pos_start = from_state.Q
            self.set_object_state(from_state)

        full_context, kwargs, binding_list = \
            self.get_transition_context(from_state, to_state,
                                        N=N, dt=dt_sim, **kwargs)

        idx_jnt = 0
        if from_state.node != to_state.node:
            joint_context = make_joint_constraints(self.joint_names, priority=2, K_joint=1, make_error=False)
            if "inp_lbl" not in kwargs:
                kwargs["inp_lbl"] = []
            kwargs["inp_lbl"] = list(kwargs["inp_lbl"]) + ["target_{joint_name}".format(joint_name=joint_name) for joint_name in self.joint_names]
            if "inp" not in kwargs:
                kwargs["inp"] = []
            idx_jnt = len(kwargs["inp"])
            kwargs["inp"] = list(kwargs["inp"]) + list(to_state.Q)
        else:
            joint_context = ""

        e_sim = get_simulation(full_context+joint_context)

        self.inp_lbl = kwargs['inp_lbl'] if 'inp_lbl' in kwargs else []
        self.inp = np.array(kwargs['inp'] if 'inp' in kwargs else [])
        e_sim.setInputTable(self.inp_lbl, inp=self.inp)

        self.pos_lbl = augment_jnames_dot(self.joint_names)
        initial_jpos_exp = augment_jvals_dot(pos_start, np.zeros_like(pos_start))
        e_sim.initialize(initial_jpos_exp, self.pos_lbl)

        pos = e_sim.simulate_begin(N, dt_sim)
        e_sim.DT = dt_sim

        return e_sim, pos, kwargs, binding_list, idx_jnt

    def execute_schedule_online(self, snode_schedule, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                vel_conv=0, err_conv=5e-4, T_step = 100, on_rviz=False,
                                dynamic_detector=None, rviz_pub=None, stop_count_ref=25):

        state_0 = snode_schedule[0].state
        from_Q = state_0.Q
        object_pose_cur = state_0.obj_pos_dict
        if not on_rviz:
            self.panda.set_k_gain(70)
            self.panda.set_d_gain(7)
            self.execute_grip(state_0)

        for i_s in range(len(snode_schedule) - 1):
            from_state = snode_schedule[i_s].state
            from_state.Q = from_Q
            from_state.obj_pos_dict = object_pose_cur
            to_snode = snode_schedule[i_s + 1]
            to_state = to_snode.state
            traj = to_snode.get_traj()
            idx_cur = 0
            self.set_object_state(from_state)

            e_sim, pos, kwargs, binding_list, idx_jnt = self.init_online_etasl(from_state, to_state, T_step,
                                                        control_freq=control_freq, playback_rate=playback_rate,
                                                        vel_conv=vel_conv, err_conv=err_conv, obs_K="40"
                                                        )

            Q0 = joint_(pos, self.joint_names)
            if not on_rviz:
                self.indy.joint_move_make_sure(np.rad2deg(Q0[self.indy_idx]), N_repeat=2, connect=True)
                print("wait for button input")
                self.indy.connect_and(self.indy.wait_di, 16)

            stop_count = 0

            with MultiTracker([self.indy, self.panda],
                              [self.indy_idx, self.panda_idx],
                              Q0, on_rviz=on_rviz) as mt:
                time.sleep(0.5)

                i_q = 0

                error_count = 0
                max_err_count = 3
                POS_CUR = from_state.Q
                VEL_CUR = np.zeros_like(POS_CUR)
                end_loop = False
                while True:
                    if on_rviz:
                        if self.indy is not None:
                            self.indy.rate_x1.sleep()
                        else:
                            self.rate.sleep()
                        all_sent = True
                    else:
                        all_sent = mt.move_possible_joints_x4(POS_CUR)

                    if all_sent:
                        if stop_count>0:
                            if stop_count>stop_count_ref:
                                break
                            else:
                                stop_count += 1
                                continue
                        i_q += 1
                        try:
                            obsPos_dict = dynamic_detector.get_dynPos_dict()
                            self.update_obstacle(obsPos_dict)
                            pos = e_sim.simulate_step(i_q, pos, dt=None, inp_cur=self.inp)
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                            # VEL_CUR = VEL_CUR + e_sim.VEL[i_q, 1::2] * e_sim.DT
                            # POS_CUR = POS_CUR + VEL_CUR * e_sim.DT
                            if rviz_pub is not None:
                                rviz_pub.update(obsPos_dict, POS_CUR)
                        except EventException as e:
                            print(e)
                            end_loop = True
                        except Exception as e:
                            error_count += 1
                            print("ERROR {}: {}".format(error_count, e))
                            if error_count > max_err_count:
                                print("MAX ERROR REACHED {}".format(error_count))
                                raise (e)
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                        idx_cur = self.update_target_joint(idx_cur, idx_jnt, traj, POS_CUR)
                        if i_q >= len(e_sim.TIME):
                            stop_count+=1
                            continue
                    if end_loop:
                        stop_count+=1
                        continue
            from_Q = POS_CUR.copy()
            for bd in binding_list:
                self.rebind(bd, joint_list2dict(POS_CUR, self.joint_names))
            object_pose_cur = self.get_object_state()[1]
            eout = e_sim.etasl.getOutput()
            if 'global.error' in eout and eout['global.error'] < err_conv*2:
                if not on_rviz:
                    self.execute_grip(to_state)
            else:
                print("FAIL ({}/{})".format(eout['global.error'] if 'global.error' in eout else "-" ,err_conv))
                break
        return e_sim

    def set_camera_config(self, aruco_map, dictionary, kn_config, rs_config, T_c21):
        self.aruco_map = aruco_map
        self.dictionary = dictionary
        self.kn_config = kn_config
        self.rs_config = rs_config
        self.cameraMatrix, self.distCoeffs = kn_config
        self.T_c21 = T_c21

    def draw_objects_graph(self, color_image, objectPose_dict, corner_dict, axis_len=0.1):
        return draw_objects(color_image, self.aruco_map, objectPose_dict, corner_dict, self.cameraMatrix,
                            self.distCoeffs, axis_len=axis_len)

    def sample_Trel(self, obj_name, obj_link_name, coord_link_name, coord_name, Teo, objectPose_dict_ref):
        aruco_map_new  = {k: self.aruco_map[k] for k in [obj_name, coord_name] if k not in objectPose_dict_ref}
        objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict, objectPoints_dict, point3D_dict, err_dict = \
            get_object_pose_dict_stereo(self.T_c21, self.kn_config, self.rs_config,
                                        aruco_map_new, self.dictionary)
        objectPose_dict.update(objectPose_dict_ref)
        T_co = objectPose_dict[obj_name]
        T_cp = objectPose_dict[coord_name]
        T_p0e = get_tf(obj_link_name, joint_list2dict(self.get_real_robot_pose(), self.joint_names), self.urdf_content,
                       from_link=coord_link_name)
        T_peo = Teo  # SE3(Rot_zyx(0,-np.pi/2,-np.pi/2), [0,0,0.091])
        T_po_cam = np.matmul(SE3_inv(T_cp), T_co)
        T_po_cal = np.matmul(T_p0e, T_peo)
        xyz_cam, rvec_cam = T2xyzrvec(T_po_cam)
        xyz_cal, rvec_cal = T2xyzrvec(T_po_cal)
        return xyz_cam, rvec_cam, xyz_cal, rvec_cal, color_image, objectPose_dict, corner_dict, err_dict

def get_goal_nodes(initial_node, obj_name, target_name, postfix="_p"):
    return [tuple([(opair[0], ppoint, target_name) \
                   if opair[0] == obj_name \
                   else opair \
                   for opair in initial_node]) \
            for ppoint in [dvkey+postfix
                           for dvkey in DIR_VEC_DICT.keys()]]
