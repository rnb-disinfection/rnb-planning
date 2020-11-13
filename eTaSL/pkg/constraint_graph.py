import sys
sys.setrecursionlimit(1000000)

from .geometry.ros_rviz import *
from urdf_parser_py.urdf import URDF
from multiprocessing import Process, Lock, cpu_count
from .controller.panda_ros_interface import *
from .controller.panda_repeater import *
from .controller.indy_repeater import indytraj_client
from .environment_builder import *

PANDA_ROS = False
PORT_REPEATER = 1189
CONTROL_FREQ = 100

try:
    from queue import PriorityQueue
except:
    from Queue import PriorityQueue

# from .utils.utils_graph import *

from multiprocessing.managers import SyncManager
class PriorityQueueManager(SyncManager):
    pass
PriorityQueueManager.register("PriorityQueue", PriorityQueue)

PROC_MODE = True
PRINT_LOG = False
DEFAULT_TRAJ_COUNT = 10

class SearchNode:
    def __init__(self, idx, state, parents, leafs, leafs_P, depth=None, edepth=None):
        self.idx, self.state, self.parents, self.leafs, self.leafs_P, self.depth, self.edepth = \
            idx, state, parents, leafs, leafs_P, depth, edepth
        self.traj = None
        self.traj_size = 0
        self.traj_length = 0

    def set_traj(self, traj_full, traj_count=DEFAULT_TRAJ_COUNT):
        self.traj_size = len(traj_full)
        self.traj_length = np.sum(differentiate(traj_full, 1)[:-1]) if self.traj_size > 1 else 0
        traj_step = max(1,self.traj_size/traj_count)
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
                 indy_ip='192.168.0.63', indy_joint_vel_level=3, indy_task_vel_level=3,
                 indy_grasp_DO=8, robots_on_scene=None, ref_tuple=('floor',None)):
        self.joint_num = len(joint_names)
        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        self.ghnd = GeometryHandle.instance()
        self.ghnd.set_urdf_content(urdf_content)
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
        self.gtimer=GlobalTimer.instance()
        self.joint_limits = np.array([(self.urdf_content.joint_map[jname].limit.lower,
                                       self.urdf_content.joint_map[jname].limit.upper) for jname in self.joint_names])
        self.marker_list = []
        self.robots_on_scene = robots_on_scene
        self.ref_tuple = ref_tuple

        self.indy_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'indy' in jname]
        self.panda_idx = [idx for idx, jname in zip(range(self.joint_num), self.joint_names) if 'panda' in jname]

        self.indy = None
        self.panda = None
        self.indy_speed = 180
        self.indy_acc = 360
        self.indy_ip = indy_ip
        self.indy_joint_vel_level = indy_joint_vel_level
        self.indy_task_vel_level = indy_task_vel_level
        self.indy_grasp_DO = indy_grasp_DO

    def reset_robots(self, connect_indy=None, connect_panda=None):
        self.reset_indy(connect_indy)
        self.reset_panda(connect_panda)

    def reset_panda(self, connect=None):
        if connect is not None:
            self.connect_panda = connect

        if self.connect_panda:
            if PANDA_ROS:
                self.ps = PandaStateSubscriber(interface_name=PANDA_SIMULATION_INTERFACE)
                self.ps.start_subsciption()
                self.pc = PandaControlPublisher(interface_name=PANDA_SIMULATION_INTERFACE)
            else:
                if self.panda:
                    self.panda.set_alpha_lpf(self.panda.alpha_lpf)
                    self.panda.set_d_gain(self.panda.d_gain)
                    self.panda.set_k_gain(self.panda.k_gain)
                else:
                    self.panda = PandaRepeater()

    def reset_indy(self, connect=None):
        if connect is not None:
            self.connect_indy = connect

        if self.connect_indy:
            self.indy = indytraj_client(server_ip=self.indy_ip, robot_name="NRMK-Indy7")
            with self.indy:
                self.indy.set_collision_level(5)
                self.indy.set_joint_vel_level(self.indy_joint_vel_level)
                self.indy.set_task_vel_level(self.indy_task_vel_level)
                self.indy.set_joint_blend_radius(20)
                self.indy.set_task_blend_radius(0.2)
            self.indy.indy_grasp_DO = self.indy_grasp_DO

    def __del__(self):
        try:
            pass
        except: pass

    def set_rviz(self, joint_pose=None):
        # prepare ros
        if not (hasattr(self, 'pub') and hasattr(self, 'joints') and hasattr(self, 'rate')):
            self.pub, self.joints, self.rate = get_publisher(self.joint_names, control_freq=CONTROL_FREQ)
        if joint_pose is None:
            joint_pose = self.joints.position
        # prepare visualization markers
        self.clear_markers()
        self.marker_list = set_markers(self.ghnd, self.joints, self.joint_names)
        self.show_pose(joint_pose)
        self.show_pose(joint_pose)

    def remove_geometry(self, gtem, from_ghnd=True):
        del_list = []
        for marker in self.marker_list:
            if marker.geometry == gtem:
                del_list.append(marker)
        for marker in del_list:
            marker.delete()
            self.marker_list.remove(marker)

        if from_ghnd:
            self.ghnd.remove(gtem)

    def add_geometry(self, gtem):
        self.marker_list += set_markers([gtem], self.joints, self.joint_names)

    def clear_markers(self):
        for mk in self.marker_list:
            mk.delete()
        self.marker_list = []
        if hasattr(self, 'highlight_dict'):
            for hkey, hset in self.highlight_dict.items():
                for k,v in hset.items():
                    self.remove_geometry(v)
                    del self.highlight_dict[hkey][k]
        else:
            self.highlight_dict = defaultdict(lambda: dict())

    def update_marker(self, gtem):
        joint_dict = {self.joints.name[i]: self.joints.position[i] for i in range(len(self.joint_names))}
        marks = [mk for mk in self.marker_list if mk.geometry == gtem]
        for mk in marks:
            mk.set_marker(joint_dict, create=False)
        return marks

    def set_planner(self, planner, bind=True):
        if bind:
            self.planner = planner
        planner.update_gtems()
        planner.set_object_dict(self.object_dict)
        planner.set_binder_dict(self.binder_dict)

    def add_binder(self, name, binder):
        self.binder_dict[name] = binder

    def register_binder(self, name, _type, link_name=None, object_name=None, **kwargs):
        if name in self.binder_dict:
            self.remove_binder(name)
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

    def remove_binder(self, bname):
        del self.binder_dict[bname]

    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

    def register_object(self, name, _type, binding=None, **kwargs):
        if name in self.object_dict:
            self.remove_object(name)

        _object = self.get_object_by_name(name)
        self.object_dict[name] = _type(_object, **kwargs)
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))

    def get_all_handles(self):
        handles = []
        for obj_hd in self.object_dict.values():
            handles += obj_hd.get_action_points().values()
        return handles

    def get_all_handle_dict(self):
        handle_dict = {}
        for obj_hd in self.object_dict.values():
            for hd in obj_hd.get_action_points().values():
                handle_dict[hd.name_constraint] = hd
        return handle_dict

    def delete_handle(self, htem):
        otem = self.object_dict[htem.object.name]
        del otem.action_points_dict[htem.name]
        if not otem.action_points_dict.keys():
            self.remove_object(htem.object.name)

    def remove_object(self, name):
        if name in self.object_dict:
            del self.object_dict[name]

    def register_object_gen(self, objectPose_dict_mv, binder_dict, object_dict, ref_tuple=None, link_name="world"):
        object_generators = {k: CallHolder(GeometryHandle.instance().create_safe,
                                           ["center", "rpy"], **v.get_kwargs()) for k, v in
                             self.aruco_map.items() if v.ttype in [TargetType.MOVABLE, TargetType.ONLINE]}
        if ref_tuple is None:
            ref_tuple = self.ref_tuple
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
                self.register_object(mtem, binding=(put_point_dict[mtem], ref_tuple[0]), **object_dict[mtem])

        return put_point_dict

    def set_cam_env_collision(self, xyz_rvec_cams, env_gen_dict):
        add_geometry_items(self.urdf_content, color=(0, 1, 0, 0.3), display=True, collision=True,
                           exclude_link=["panda1_link7"])
        add_cam_poles(self, xyz_rvec_cams)
        add_objects_gen(self, env_gen_dict)

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
        bindings = get_all_mapping(self.handle_dict.keys(),
                                   self.binder_dict.keys())  # all possible binding combinations
        handle_combinations = list(
            product(*[self.handle_dict[obj] for obj in self.object_list]))  # all possible handle combinations
        uniq_binders = self.get_unique_binders()  # binders cannot be shared by multiple objects
        ctrl_binders = self.get_controlled_binders()  # all controllable binders
        self.node_list = []
        self.node_dict = {}
        for binding in bindings:  # binding combination loop
            if all([np.sum([bd == ub for bd in binding.values()]) <= 1 for ub in uniq_binders]):
                for hc in handle_combinations:  # handle combination loop
                    node = []
                    add_ok = True
                    for i_o, obj in zip(range(len(self.object_list)), self.object_list):  # object loop
                        hndl = self.object_dict[obj].action_points_dict[hc[i_o]]  # handle object
                        binder = self.binder_dict[binding[obj]]  # binder object
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
        fix_node_dict = defaultdict(lambda: list())
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
                conflict_ptr_ctrl = [self.object_dict[bdg[0]].get_conflicting_handles(bdg[1]) for bdg in bdg_ctrl]

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

    def get_unique_binders(self):
        uniq_binders = []
        for k_b, binder in self.binder_dict.items():
            if not binder.multiple:
                uniq_binders += [k_b]
        return uniq_binders

    def get_controlled_binders(self):
        controlled_binders = []
        for k_b, binder in self.binder_dict.items():
            if binder.controlled:
                controlled_binders += [k_b]
        return controlled_binders

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

    def get_object_state(self):
        node = ()
        pose_dict = {}
        for k in self.object_list:
            v = self.object_dict[k]
            node += ((k,) + v.binding,)
            pose_dict[k] = v.object.get_frame()
        return node, pose_dict

    def get_slack_bindings(self, from_state, to_state):
        binding_list = []
        if to_state.node is not None:
            for bd0, bd1 in zip(from_state.node, to_state.node):
                if bd0[2] != bd1[2]: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"
        return binding_list

    def test_transition(self, from_state=None, to_state=None, display=False, dt_vis=1e-2, error_skip=0, **kwargs):
        self.gtimer = GlobalTimer.instance()

        if from_state is not None:
            self.set_object_state(from_state)

        binding_list = self.get_slack_bindings(from_state, to_state)

        success = True
        for binding in binding_list:
            if not self.binder_dict[binding[2]].check_available(joint_list2dict(from_state.Q, self.joint_names)):
                success = False
        if success:
            Traj, LastQ, error, success = self.planner.plan_transition(from_state, to_state, binding_list, **kwargs)
            if display:
                self.show_motion(Traj, from_state, **{'error_skip':error_skip, 'period':dt_vis})
        else:
            print("=====================================================")
            print("=====================================================")
            print("=====================================================")
            print("===============Unavailable binder====================")
            print("=====================================================")
            print("=====================================================")
            print("=====================================================")
            LastQ = from_state.Q
            Traj = np.array([LastQ])
            error = 1

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

    def indy_grasp(self, grasp=False):
        if self.connect_indy:
            self.indy.grasp(grasp, connect=True)

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

    def rebind(self, binding, joint_dict_last):
        binder = self.binder_dict[binding[2]]
        object_tar = self.object_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict_last)
        for binder_sub in [k for k,v in self.binder_dict.items() if v.object == object_tar.object]:
            for binding_sub in [(k,v.binding[0]) for k, v in self.object_dict.items()
                                if v.binding[1] == binder_sub]:
                binding_sub += (binder_sub,)
                self.rebind(binding_sub, joint_dict_last)

    def score_graph(self, goal_node):
        # if isinstance(goal_node, list):
        #     score_dicts = [self.score_graph(goal) for goal in goal_node]
        #     score_dict = {}
        #     for k in score_dicts[0].keys():
        #         score_dict[k] = min([sdict[k] for sdict in score_dicts])
        #     return score_dict
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
                new_cost = node_cost_dict[current] + ConstraintGraph.DEFAULT_TRANSIT_COST
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
            to_state = state.copy()

            if leaf == state.node:
                dQ = (1 - 2 * np.random.rand(self.DOF)) * ConstraintGraph.DQ_MAX
                to_state.Q = np.sum([state.Q, dQ], axis=0)
                to_state.Q = np.minimum(np.maximum(to_state.Q, self.joint_limits[:,0]), self.joint_limits[:,1])
            else:
                to_state.node = leaf
            # self.snode_queue.put((snode, state, to_state), expected_depth * self.DSCALE - depth) ## breadth-first
            self.snode_queue.put(((expected_depth - depth) * self.DSCALE + depth, (snode, state, to_state))) ## greedy
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

        self.snode_counter = SingleValue('i', 0)
        self.search_counter = SingleValue('i', 0)
        self.stop_now =  SingleValue('i', 0)
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
                      display=False, dt_vis=None, verbose=False, print_expression=False,
                      traj_count=DEFAULT_TRAJ_COUNT, **kwargs):
        loop_counter = 0
        while self.snode_counter.value < N_search and loop_counter < N_loop and not self.stop_now.value:
            loop_counter += 1
            self.que_lock.acquire()
            if self.snode_queue.empty():
                break
            snode, from_state, to_state = self.snode_queue.get()[1]
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
                snode_new.set_traj(traj, traj_count=traj_count)
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

    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

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

    def check_goal(self, node, goals):
        return node in goals

    def check_goal_by_score(self, node, goal_cost_dict):
        return goal_cost_dict[node] == 0

    def print_snode_list(self):
        for i_s, snode in sorted(self.snode_dict.items(), key=lambda x: x):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.snode_dict[snode.parents[-1]].state.node if snode.parents else ""))

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

    def show_pose(self, pose, **kwargs):
        show_motion([pose], self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    def show_motion(self, pose_list, from_state=None, **kwargs):
        if from_state is not None:
            self.set_object_state(from_state)
        show_motion(pose_list, self.marker_list, self.pub, self.joints, self.joint_names, **kwargs)

    def clear_highlight(self, hl_keys=[]):
        for hl_key, hl_set in self.highlight_dict.items():
            if hl_key in hl_keys or not hl_keys:
                for k,v in hl_set.items():
                    self.remove_geometry(v)
                    del self.highlight_dict[hl_key][k]

    def highlight_geometry(self, hl_key, gname, color=(1, 0.3, 0.3, 0.5)):
        if gname not in self.ghnd.NAME_DICT:
            return
        gtem = self.ghnd.NAME_DICT[gname]
        dims = gtem.dims if np.sum(gtem.dims) > 0.001 else (0.03, 0.03, 0.03)
        hname = "hl_" + gtem.name
        if hname in self.ghnd.NAME_DICT:
            return
        htem = self.ghnd.create_safe(gtype=gtem.gtype, name=hname, link_name=gtem.link_name,
                            center=gtem.center, dims=dims, rpy=Rot2rpy(gtem.orientation_mat), color=color,
                            collision=False)

        self.highlight_dict[hl_key][htem.name] = htem
        self.add_geometry(htem)

    def add_highlight_axis(self, hl_key, name, link_name, center, orientation_mat, color=None, axis="xyz", dims=(0.10, 0.01, 0.01)):
        if 'x' in axis:
            axtemx = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axx_" + name, link_name=link_name,
                                  center=center, dims=dims, rpy=Rot2rpy(orientation_mat), color=color or (1, 0, 0, 0.5),
                                  collision=False)
            self.add_geometry(axtemx)
            self.highlight_dict[hl_key][axtemx.name] = axtemx

        if 'y' in axis:
            axtemy = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axy_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(3, np.pi / 2))), color=color or (0, 1, 0, 0.5),
                                  collision=False)
            self.add_geometry(axtemy)
            self.highlight_dict[hl_key][axtemy.name] = axtemy

        if 'z' in axis:
            axtemz = self.ghnd.create_safe(gtype=GEOTYPE.ARROW, name="axz_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(2, -np.pi / 2))),
                                  color=color or (0, 0, 1, 0.5),
                                  collision=False)
            self.add_geometry(axtemz)
            self.highlight_dict[hl_key][axtemz.name] = axtemz
    ############################### AXIS ADDER ######################################

    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.handle.object
        if hasattr(handle, 'orientation_mat'):
            orientation_mat = np.matmul(hobj.orientation_mat, handle.orientation_mat)
            axis = "xyz"
            color = None
        elif hasattr(handle, 'direction'):
            orientation_mat = np.matmul(hobj.orientation_mat,
                                        Rotation.from_rotvec(calc_rotvec_vecs([1, 0, 0], handle.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.add_highlight_axis(hl_key, hobj.name, hobj.link_name, hobj.center, orientation_mat, color=color, axis=axis)

    def add_binder_axis(self, hl_key, binder, color=None):
        bobj = binder.effector.object
        if hasattr(binder, 'orientation'):
            orientation_mat = np.matmul(bobj.orientation_mat, binder.effector.orientation_mat)
            axis = "xyz"
        elif hasattr(binder, 'direction'):
            orientation_mat = np.matmul(bobj.orientation_mat, Rotation.from_rotvec(
                calc_rotvec_vecs([1, 0, 0], binder.effector.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.add_highlight_axis(hl_key, bobj.name, bobj.link_name, bobj.center, orientation_mat, color=color, axis=axis)

    def add_aruco_axis(self, hl_key, atem, axis_name=None):
        oname = atem.oname
        axis_name = axis_name or oname
        if oname in self.robots_on_scene:
            link_name = RobotType.get_base_link(self.robots_on_scene[oname], oname)
            Toff = atem.Toff
        else:
            aobj = self.ghnd.NAME_DICT[oname]
            link_name = aobj.link_name
            Toff = np.matmul(aobj.get_offset_tf(), atem.Toff)
        self.add_highlight_axis(hl_key, axis_name, link_name, Toff[:3,3], Toff[:3,:3], axis="xyz")

    def idxSchedule2SnodeScedule(self, schedule, ZERO_JOINT_POSE=None):
        snode_schedule = [self.snode_dict[i_sc] for i_sc in schedule]
        snode_schedule.append(snode_schedule[-1].copy())
        if ZERO_JOINT_POSE is None:
            ZERO_JOINT_POSE = [0]*self.joint_num
        snode_schedule[-1].state.Q = ZERO_JOINT_POSE
        snode_schedule[-1].set_traj(np.array([ZERO_JOINT_POSE]))
        return snode_schedule

    def execute_schedule_online(self, snode_schedule, planner, control_freq=DEFAULT_TRAJ_FREQUENCY, playback_rate=0.5,
                                on_rviz=False, dynamic_detector=None, rviz_pub=None, stop_count_ref=25,
                                T_step=30, **kwargs):

        state_0 = snode_schedule[0].state
        from_Q = state_0.Q
        object_pose_cur = state_0.obj_pos_dict
        N_step = T_step*control_freq
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
            end_traj = len(traj)-1

            if from_state is not None:
                self.set_object_state(from_state)

            binding_list = self.get_slack_bindings(from_state, to_state)

            pos, binding_list = \
                planner.init_online_plan(from_state, to_state, binding_list,
                                              control_freq=control_freq, playback_rate=playback_rate,
                                              T_step=T_step, **kwargs
                                              )

            Q0 = np.array(joint_dict2list(pos, self.joint_names))
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
                            planner.update_online(obsPos_dict)
                            pos, end_loop, error, success = planner.step_online_plan(i_q, pos, wp_action=idx_cur<end_traj)
                            # print("{i_s} : {idx_cur}<{end_traj}:{wp}".format(i_s=i_s, idx_cur=idx_cur, end_traj=end_traj, wp=idx_cur<end_traj))
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                            # VEL_CUR = VEL_CUR + e_sim.VEL[i_q, 1::2] * e_sim.DT
                            # POS_CUR = POS_CUR + VEL_CUR * e_sim.DT
                        except Exception as e:
                            error_count += 1
                            print("ERROR {}: {}".format(error_count, e))
                            if error_count > max_err_count:
                                print("MAX ERROR REACHED {}".format(error_count))
                                raise (e)
                            POS_CUR = np.array(joint_dict2list(pos, self.joint_names))
                        if rviz_pub is not None:
                            rviz_pub.update(obsPos_dict, POS_CUR)
                        idx_cur = planner.update_target_joint(idx_cur, traj, POS_CUR)
                        if i_q >= N_step:
                            stop_count+=1
                            continue
                    if end_loop:
                        stop_count+=1
                        continue
            from_Q = POS_CUR.copy()
            joint_dict = joint_list2dict(POS_CUR, self.joint_names)
            for bd in binding_list:
                self.rebind(bd, joint_dict)
            object_pose_cur = self.get_object_state()[1]
            if success:
                if not on_rviz:
                    self.execute_grip(to_state)
            else:
                print("FAIL ({})".format(error))
                break

    def set_camera_config(self, aruco_map, dictionary, kn_config, rs_config, T_c12):
        self.aruco_map = aruco_map
        self.dictionary = dictionary
        self.kn_config = kn_config
        self.rs_config = rs_config
        self.cameraMatrix, self.distCoeffs = kn_config
        self.T_c12 = T_c12

    def draw_objects_graph(self, color_image, objectPose_dict, corner_dict, axis_len=0.1):
        return draw_objects(color_image, self.aruco_map, objectPose_dict, corner_dict, self.cameraMatrix,
                            self.distCoeffs, axis_len=axis_len)

    def sample_Trel(self, obj_name, obj_link_name, coord_link_name, coord_name, Teo, objectPose_dict_ref):
        aruco_map_new  = {k: self.aruco_map[k] for k in [obj_name, coord_name] if k not in objectPose_dict_ref}
        objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict, objectPoints_dict, point3D_dict, err_dict = \
            get_object_pose_dict_stereo(self.T_c12, self.kn_config, self.rs_config,
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
