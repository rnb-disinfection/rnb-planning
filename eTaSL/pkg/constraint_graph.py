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

    def __init__(self, urdf_path, joint_names, link_names, urdf_content=None):
        self.urdf_path = urdf_path
        if urdf_content is None:
            urdf_content = URDF.from_xml_file(urdf_path)
        self.urdf_content = urdf_content
        self.joint_names = joint_names
        self.link_names = link_names
        self.collision_items_dict = {}
        self.binder_dict = {}
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}
        self.lock = Lock()
        self.sim_q_lock = Lock()
        self.manager = Manager()
        
    def set_simulation(self, nWSR=50, cputime=200, regularization_factor= 1e-4):
        # prepare ros
        self.pub, self.joints, self.rate = get_publisher(self.joint_names)
        # prepare visualization markers
        self.marker_list = set_markers(self.collision_items_dict, self.joints, self.joint_names, self.link_names, self.urdf_content)
        set_simulation_config(joint_names = self.joint_names, link_names = self.link_names, 
                              urdf_content = self.urdf_content, urdf_path = self.urdf_path,
                              collision_items_dict=self.collision_items_dict,
                              nWSR=nWSR, cputime=cputime, regularization_factor= regularization_factor)
        self.show_pose(np.zeros(len(self.joint_names)))
        
    def set_collision_items(self, collision_items_dict):
        self.collision_items_dict = collision_items_dict
        self.collision_items_list = []
        for v in collision_items_dict.values():
            self.collision_items_list += v
        
    def add_collision_items(self, link_name, collision_Items):
        self.collision_items_dict[link_name] += collision_Items
        self.collision_items_list += collision_Items
        
    def add_binder(self, name, binder):
        self.binder_dict[name] = binder
        
    def register_binder(self, name, _type, link_name=None, object_name=None, **kwargs):
        if object_name is None:
            object_name = name
        _objects = filter(lambda x: x.name == object_name, self.collision_items_list)
        
        if _objects:
            _object = _objects[0]
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
            
        
    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0], 
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))
        
    def register_object(self, name, _type, binding=None, **kwargs):
        _object = filter(lambda x: x.name == name, self.collision_items_list)[0]
        self.object_dict[name] = _type(_object, **kwargs)
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0], 
                                              joint_list2dict([0]*len(self.joint_names), joint_names=self.joint_names))
            
        
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
                obj.set_state(frame, binder.link_name, bd[1], bd[2], 
                              self.collision_items_dict)
                bd_list_done += [bd]
            
    def get_object_state(self):
        node = ()
        pose_dict = {}
        for k in self.object_list:
            v = self.object_dict[k]
            node += ((k,) + v.binding,)
            pose_dict[k] = v.object.get_frame()
        return node, pose_dict
    
    def simulate_transition(self, from_state=None, to_state=None, display=False, error_skip=1e-4, lock=False, 
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
            
        init_text = get_init_text()
        if not display:
            if lock:
                self.lock.release()
            gtimer.toc("start set transition")
            gtimer.tic("set_simulate fun")
            e = set_simulate(init_text, additional_constraints=additional_constraints, 
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
            e = prepare_simulate(init_text, additional_constraints=additional_constraints, 
                                 vel_conv=vel_conv, err_conv=err_conv)
            for _ in range(int(N/N_step)):
                e = do_simulate(e, initial_jpos=np.array(pos_start), N=N_step, dt=dt, **kwargs)
                pos_start = e.POS[-1]
                show_motion(e.POS, self.marker_list, self.pub, self.joints, self.joint_names, error_skip=1e-4)
                if hasattr(e, 'error') and e.error<err_conv:
                    success = True
                    break
            gtimer.tic("post")
            if from_state is not None:
                self.set_object_state(from_state)
            if success:
                for bd in binding_list:
                    self.rebind(bd, e.joint_dict_last)
        
        node, obj_pos_dict = self.get_object_state()
        end_state = State(node, obj_pos_dict, list(e.POS[-1]))
        gtimer.toc("post")
        print(gtimer)
        return e, end_state, success
    
    def rebind(self, binding, joint_dict_last):
        binder = self.binder_dict[binding[2]]
        object_tar = self.object_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict_last)
        for binder_sub in [k for k,v in self.binder_dict.items() if v.object == object_tar.object]:
            for binding_sub in [(k,v.binding[0]) for k, v in self.object_dict.items()
                                if v.binding[1] == binder_sub]:
                binding_sub += (binder_sub,)
                self.rebind(binding_sub, joint_dict_last)
        
        
        
    def build_graph(self):
        # self = graph
        self.node_dict = {}
        bindings = get_all_mapping(self.handle_dict.keys(), self.binder_dict.keys())
        handle_combinations = list(product(*[self.handle_dict[obj] for obj in self.object_list]))
        uniq_binders = self.get_unique_binders()
        ctrl_binders = self.get_controlled_binders()
        self.node_list = []
        self.node_dict = {}
        for binding in bindings:
            if all([np.sum([bd == ub for bd in  binding.values()])<=1 for ub in uniq_binders]):
                for hc in handle_combinations:
                    node = []
                    add_ok = True
                    for i_o, obj in zip(range(len(self.object_list)), self.object_list):
                        hndl = self.object_dict[obj].action_points_dict[hc[i_o]]
                        binder = self.binder_dict[binding[obj]]
                        if binder.check_type(hndl):
                            node += [(obj, hc[i_o], binding[obj]) ]
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
                nodes_same_fix = [nd for nd in  self.node_list if [nd[idx] for idx in fixed_in] == bd_fixed]
                nodes_diff_ctrl = [nd for nd in  nodes_same_fix if [nd[idx][2] for idx in ctrl_in] != bd_ctrl]
                nodes_diff_pt = [nd for nd in  nodes_diff_ctrl if [nd[idx][1] for idx in ctrl_in] != pt_ctrl]
                nodes_neighbor = [nd for nd in  nodes_diff_pt if len(set([bd[2] for bd, bd0 in zip(nd, node) if bd!=bd0]).intersection(bd_ctrl))==0]
                self.node_dict[node] += nodes_neighbor
                for nd_n in nodes_neighbor:
                    self.node_dict[nd_n] += [node]

        for node in self.node_dict.keys():
            if node in self.node_dict[node]:
                self.node_dict[node].remove(node)
            self.node_dict[node] = [node] + list(set(self.node_dict[node]))
            
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
                    
    def get_score(self, node, current_node, parent_node, decay_num, node_cost_dict):
        if node_cost_dict[node] == 0:
            return ConstraintGraph.WEIGHT_GOAL
#         elif node == current_node:
#             return (ConstraintGraph.JOINT_MULTIPLIER*ConstraintGraph.WEIGHT_DEFAULT*ConstraintGraph.NODE_DECAY**decay_num)/node_cost_dict[node]
        else:
            if current_node != parent_node and node == parent_node:
                return 0
            else:
                return (ConstraintGraph.WEIGHT_DEFAULT*ConstraintGraph.NODE_DECAY**decay_num)/node_cost_dict[node]
            
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
                
    def calc_combined_score(self, init_node, init_cost_dict, goal_node, goal_cost_dict, margin_multiplier=1):
        ig_half = max(init_cost_dict[goal_node], goal_cost_dict[init_node])/2
        cost_dict = {k: min(init_cost_dict[k], goal_cost_dict[k]) for k in init_cost_dict.keys()}
        score_dict = {k: (ConstraintGraph.WEIGHT_DEFAULT/v if v<=ig_half*margin_multiplier else 0) for k,v in cost_dict.items()
                     if k!= init_node and k!=goal_node}
        score_dict[init_node] = ConstraintGraph.WEIGHT_DEFAULT
        score_dict[goal_node] = ConstraintGraph.WEIGHT_GOAL
        return score_dict
    
    def get_combined_score(self, node, current_node, score_dict):
        if node == current_node:
            return min(ConstraintGraph.JOINT_MULTIPLIER*score_dict[node], ConstraintGraph.WEIGHT_MAX)
        else:
            return score_dict[node]
    
    def init_threads(self, N_agents):
        if PROC_MODE:
            self.thread_que = [Process(target=self.loop_test_random) for _ in range(N_agents)]
        else:
            self.thread_que = [Thread(target=self.loop_test_random) for _ in range(N_agents)]
        self.kwargs_que = self.manager.list([])
        for t in self.thread_que:
            t.start()
    
    def join_all(self):
        for t in self.thread_que:
            t.join()
                  
    def search_graph(self, initial_state, goal_state, search_num=100, display=True, N_sol=3, N_agents=8, Q_len=3, search_margin=0, **kwargs):
        self.time0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.initial_state = initial_state
        self.goal_state = goal_state
    
        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.node), self.score_graph(goal_state.node)
        self.score_dict = self.calc_combined_score(initial_state.node, self.init_cost_dict, goal_state.node, self.goal_cost_dict)
        self.reset_valid_node(search_margin)
        
        self.num_sol = self.manager.Value('i', 0)
        self.i_done = self.manager.Value('i', 0)
        self.N_sol_max = N_sol
        if N_agents>1:
            self.joint_multiplier_dict = self.manager.dict({k: ConstraintGraph.JOINT_MULTIPLIER for k in self.node_list})
            self.snode_vec = self.manager.list([
                SearchNode(0, state=initial_state, parents=[], leafs=[], 
                           leafs_P=[ConstraintGraph.WEIGHT_DEFAULT]*len(self.valid_node_dict[initial_state.node]))
            ])
            self.weight_vec = self.manager.list([np.sum(self.snode_vec[0].leafs_P)])
            self.init_threads(N_agents)
        else:
            self.joint_multiplier_dict = {k: ConstraintGraph.JOINT_MULTIPLIER for k in self.node_list}
            self.snode_vec = [
                SearchNode(0, state=initial_state, parents=[], leafs=[], 
                           leafs_P=[ConstraintGraph.WEIGHT_DEFAULT]*len(self.valid_node_dict[initial_state.node]))
            ]
            self.weight_vec = [np.sum(self.snode_vec[0].leafs_P)]
        
        for itor in range(1, search_num):
            if N_agents>1:
#                 print("queue-in: {}/{}".format(self.i_done.value,itor-1))
                while not (
                    (len(self.kwargs_que)<Q_len and # kwargs_que < Q_len
                     (self.i_done.value>=itor-1 or 
                      (itor>N_agents and self.i_done.value>=itor-max(N_agents/2,ConstraintGraph.MAX_AGENT_DELAY))) # i_done>=self.itor-1 or 
                    ) or
                    self.num_sol.value>=N_sol): # num_sol < N_sol
                    timer.sleep(0.01)
#                 print("diff_done: {}".format(self.i_done.value-itor+1))
#                 print("queue-pass: {}/{}".format(self.i_done.value,itor-1))
                if self.num_sol.value>=N_sol:
                    self.join_all()
                    break
                kwargs_th = dict(itor=itor,initial_state=initial_state, goal_state=goal_state, search_num=search_num,
                                display=display)
                kwargs_th.update(kwargs)
                self.kwargs_que += [kwargs_th]
            else:
                self.test_random(itor=itor,
                                 initial_state=initial_state, goal_state=goal_state, 
                                 search_num=search_num, display=display, **kwargs)
                if self.num_sol.value>=N_sol:
                    break
            self.itor = itor
                
    def loop_test_random(self):
        while self.num_sol.value < self.N_sol_max:
            kwargs = None
            self.lock.acquire()
            if len(self.kwargs_que)>0:
                kwargs = self.kwargs_que.pop(0)
            self.lock.release()
            if kwargs:
                self.test_random(**kwargs)
            timer.sleep(0.01)
            
            
    def test_random(self, itor, initial_state, goal_state, search_num=100, display=True, **kwargs):
        self.lock.acquire()
        weight_vec_cur = np.divide(self.weight_vec,np.sum(self.weight_vec))
        i_s = np.random.choice(len(self.snode_vec), p=weight_vec_cur)
        snode_cur = self.snode_vec[i_s]
        state = snode_cur.state
        leafs = self.valid_node_dict[state.node]
#         print("state.node{}: {}".format(i_s, state.node))
#         print("snode_cur.leafs_P{}: {}".format(i_s, snode_cur.leafs_P))
        leafP = list(snode_cur.leafs_P)
        leafP[0] = leafP[0]*self.joint_multiplier_dict[state.node]
        self.joint_multiplier_dict[state.node] = self.joint_multiplier_dict[state.node]*ConstraintGraph.JOINT_DECAY
#         print("{}:{}".format(state.node, self.joint_multiplier_dict[state.node]))
        leafP = np.divide(leafP,np.sum(leafP))
        i_l = np.random.choice(len(leafs), p=leafP)
        leafs_P_bak = snode_cur.leafs_P[i_l]
        snode_cur.leafs_P[i_l] = 0.0
        self.snode_vec[i_s] = snode_cur
        leaf = leafs[i_l]
        score_bak = snode_cur.leafs_P[i_l]
        to_state = state.copy()
        if leaf == state.node:
            snode_cur.leafs_P[i_l] = leafs_P_bak
            self.snode_vec[i_s] = snode_cur
            dQ = (1-2*np.random.rand(self.DOF))*ConstraintGraph.DQ_MAX
            to_state.Q = np.sum([state.Q, dQ], axis=0)
        else:
            to_state.node = leaf
#             print("snode_cur.leafs_O{}: {}".format(i_s, snode_cur.leafs_P))
        self.weight_vec[i_s] = np.sum(snode_cur.leafs_P)
        try:
            fatal = False
            e, new_state, succ = self.simulate_transition(state, to_state, display=display, lock=True, **kwargs)
            fatal = True
            if succ:
                snode_new = SearchNode(idx=len(self.snode_vec), state=new_state, parents=snode_cur.parents+[i_s], leafs=[], 
                               leafs_P=[self.get_score(node, new_state.node, state.node, len(snode_cur.parents), self.goal_cost_dict)
                                    for node in self.valid_node_dict[new_state.node]])
                self.snode_vec += [snode_new]
                idx_new = snode_new.idx
                snode_cur = self.snode_vec[i_s]
                snode_cur.leafs += [idx_new]
                self.snode_vec[i_s] = snode_cur
                print('\n{}/{} success - ({} s) / Goal cost:{}->{} / Init cost:{}->{} / Score: {} / branching: {}->{}'.format(
                    itor, search_num, round(timer.time()-self.time0, 2),
                    int(self.goal_cost_dict[state.node]), int(self.goal_cost_dict[new_state.node]),
                    int(self.init_cost_dict[state.node]), int(self.init_cost_dict[new_state.node]),  
                    score_bak, i_s, idx_new))
                print('State: {}->{}'.format(state.node, new_state.node))


                if self.check_goal(new_state, goal_state):
                    print("\n"+"#"*40+"\n"+"#"*40+"\n"+"#"*40)
                    print("############### goal reached - {} ################".format(idx_new))
                    print("#"*40+"\n"+"#"*40+"\n"+"#"*40)
                    print("succ: {}".format(succ))
                    print("state: {}".format(state))
                    print("to_state: {}".format(to_state))
                    print("new_state: {}".format(new_state))
                    self.weight_vec += [0.0]
                    self.num_sol.value += 1
                    self.lock.release()
                    self.i_done.value = max(self.i_done.value, itor)
                    return True
                else:
                    self.weight_vec += [np.sum(snode_new.leafs_P)]
            else:
                print('\n{}/{} fail - ({} s)'.format(itor, search_num, round(timer.time()-self.time0, 2)))
            print('-'*20)
            fatal = False
        except Exception as e:
            if fatal:
                print("\n"+"!"*40+"\n"+"!"*40+"\n"+"!"*40)
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!{}/{} fatal failure - ({} s)!!!!!!!!!!!!!!!!!!!!!!!!!!!!'.format(itor, search_num, round(timer.time()-self.time0, 2)))
                print("!"*40+"\n"+"!"*40+"\n"+"!"*40)
                print("state: {}".format(state))
                print("to_state: {}".format(to_state))
                print("new_state: {}".format(new_state))
                print(e)
                print('-'*20)
            else:
                print('\n{}/{} simulation fail - ({} s)'.format(itor, search_num, round(timer.time()-self.time0, 2)))
                print(e)
                print('-'*20)
                self.i_done.value = max(self.i_done.value, itor)
                return False
        self.lock.release()
        self.i_done.value = max(self.i_done.value, itor)
        return False
    
    def search_node(self, snode, depth=None, terminate_on_first=False, display=False, **kwargs):
        state = snode.state
        leafs = self.valid_node_dict[state.node]
        leafs = leafs[:-1]+[leafs[-1]]*self.joint_motion_num
    #     print('node')
    #     print(state.node)
    #     print('leafs')
    #     print(leafs)
        for leaf in leafs:
            if depth:
                expected_depth = len(snode.parents)+1+self.goal_cost_dict[leaf]
                if expected_depth>depth:
    #                 print("expect depth: {}/{} / branching: {}->".format(int(expected_depth),depth, snode.idx))
    #                 print('node: {}->{}'.format(state.node, leaf))
                    continue
            to_state = state.copy()
            if leaf == state.node:
                dQ = (1-2*np.random.rand(self.DOF))*ConstraintGraph.DQ_MAX
                to_state.Q = np.sum([state.Q, dQ], axis=0)
            else:
                to_state.node = leaf
            try:
                e, new_state, succ = self.simulate_transition(state, to_state, display=display, lock=False, **kwargs)
            except Exception as e:
                print(e)
                succ = False
            ret = False
            print('\n{} ({}) - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} ({} s)'.format(
                "success" if succ else "fail", e.error if hasattr(e, "error") else None,
                int(self.goal_cost_dict[state.node]), int(self.goal_cost_dict[leaf]),
                int(self.init_cost_dict[state.node]), int(self.init_cost_dict[leaf]),  
                snode.idx, len(self.snode_vec) if succ else "", round(timer.time()-self.t0,2)))
            print('node: {}->{}'.format(state.node, leaf))
            print('='*150)
            if succ:
                snode_new = SearchNode(idx=len(self.snode_vec), state=new_state, parents=snode.parents+[snode.idx], leafs=[], 
                               leafs_P=[])
                self.snode_vec += [snode_new]
                snode.leafs += [snode_new.idx]
                self.snode_vec[snode.idx] = snode
                ret = False
                if self.check_goal(snode_new.state, self.goal_state):
                    ret = True
                else:
                    ret = self.search_node(snode_new, depth, terminate_on_first=terminate_on_first, **kwargs)
            if terminate_on_first and ret:
                return True
        return ret


    def search_graph_ordered(self, initial_state, goal_state, display=False, 
                     terminate_on_first = True, tree_margin = 0, depth_margin = 0, joint_motion_num=10, **kwargs):
        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.initial_state = initial_state
        self.goal_state = goal_state

        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.node), self.score_graph(goal_state.node)
        self.score_dict = self.calc_combined_score(initial_state.node, self.init_cost_dict, goal_state.node, self.goal_cost_dict)
        self.reset_valid_node(tree_margin)
        depth_min = self.goal_cost_dict[initial_state.node]

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()
        self.snode_vec = [
            SearchNode(idx=0, state=initial_state, parents=[], leafs=[], 
                       leafs_P=[ConstraintGraph.WEIGHT_DEFAULT]*len(self.valid_node_dict[initial_state.node]))
        ]
        self.search_node(self.snode_vec[0], depth=depth_min+depth_margin, 
                    terminate_on_first=terminate_on_first, display=display, **kwargs)

    def searching_proc(self, id_agent, display=False, terminate_on_first=False, **kwargs):
        while not self.stop_now.value:
            if PRINT_LOG:
                print('process {} ({}s)'.format(id_agent,round(timer.time()-self.t0,2)))
            id_th = uuid.uuid1().int
            t1 = timer.time()
            self.sim_q_lock.acquire()
            t2 = timer.time()
            edep_candi = [k for k,v in self.sim_q_dict.items() if len(v)>0 and k<=self.depth_cur.value]
            if len(edep_candi)==0:
                self.sim_q_lock.release()
#                 print('len(edep_candi)==0')
                timer.sleep(1e-1)
                continue
            itor_now = self.itor.value+1
            self.itor.value = itor_now
            edep= np.min(edep_candi)
            sim_tem = self.sim_q_dict[edep][0]
            self.sim_q_dict[edep] = self.sim_q_dict[edep][1:]
            len_sim_q_dict = len(self.sim_q_dict[edep])
            if PRINT_LOG:
                print("edep {} remains: {}".format(edep, len(self.sim_q_dict[edep])))
            self.sim_q_lock.release()
            t3 = timer.time()

            self.lock.acquire()
            t4 = timer.time()
            self.working_count.value = self.working_count.value + 1
            snode, to_state = sim_tem
            state = snode.state
            t5 = timer.time()
            # print("t1: %.2f ms (%d)"%((t2-t1)*1000, id_th))
            try:
                e, new_state, succ = self.simulate_transition(state, to_state, display=display, lock=True, **kwargs)
            except Exception as e:
                print("simulation exception: {}".format(str(e)))
                self.lock.acquire()
                succ = False
            t6 = timer.time()
            # print("t2: %.2f ms (%d)"%((t3-t2)*1000, id_th))
            idx_new = len(self.snode_vec)
            print('\n{}. {} - Goal cost:{}->{} / Init cost:{}->{} / branching: {}->{} / edepth: {}/{} / {} s ({})'.format(
                itor_now, "success" if succ else "fail",
                int(self.goal_cost_dict[state.node]), int(self.goal_cost_dict[to_state.node]),
                int(self.init_cost_dict[state.node]), int(self.init_cost_dict[to_state.node]),  
                snode.idx, idx_new if succ else "", edep, int(self.depth_cur.value), round(timer.time()-self.t0,2), id_agent))
            print('node: {}->{}:{}'.format(state.node, to_state.node, new_state.node if succ else ""))
            print('='*150)
            if succ:
                snode_new = SearchNode(idx=idx_new, state=new_state, parents=snode.parents+[snode.idx], leafs=[], 
                               leafs_P=[], depth=snode.depth+1, edepth=edep)
                snode.leafs += [snode_new.idx]
                self.snode_vec[snode.idx] = snode
                self.done_vec += [False]
                self.snode_vec += [snode_new]
                self.time_vec += [timer.time()-self.t0]
                self.time_table += [[t2-t1, t3-t2, t4-t3, t5-t4, t6-t5]]
                self.itor_vec += [itor_now]
                self.sim_q_len_vec += [len_sim_q_dict]
                if terminate_on_first:
                    if self.check_goal(snode_new.state, self.goal_state):
                        print("stop now!")
                        self.stop_now.value = 1
            self.working_count.value = self.working_count.value - 1
            self.lock.release()
        timer.sleep(1e-1)
    #         print("t3: %.2f ms (%d)"%((t4-t3)*1000, id_th))

    def search_graph_ordered_mp(self, initial_state, goal_state, search_num=300, display=False, N_agents=8, 
                                terminate_on_first = False, tree_margin = 0, depth_margin = 0, joint_motion_num=10, 
                                expand_depth_on_thread=True, **kwargs):
        if display:
            print("Currently cannot display motion in multi-process searching")
            display = False
        self.joint_motion_num = joint_motion_num
        self.t0 = timer.time()
        self.DOF = len(initial_state.Q)
        self.initial_state = initial_state
        self.goal_state = goal_state

        self.init_cost_dict, self.goal_cost_dict = self.score_graph(initial_state.node), self.score_graph(goal_state.node)
        self.reset_valid_node(tree_margin)
        depth_min = self.goal_cost_dict[initial_state.node]
        self.stop_now = self.manager.Value('i',0)
        self.goal_idx_vec = self.manager.list()
        self.depth_cur = self.manager.Value('i',depth_min)
        self.max_depth = depth_min+depth_margin

        for k in self.valid_node_dict.keys():
            self.valid_node_dict[k].reverse()
        self.snode_vec = self.manager.list([
            SearchNode(idx=0, state=initial_state, parents=[], leafs=[], 
                       leafs_P=[ConstraintGraph.WEIGHT_DEFAULT]*len(self.valid_node_dict[initial_state.node]),
                       depth=0,edepth=depth_min
                      )
        ])
        self.done_vec = self.manager.list([False])
        self.time_vec = self.manager.list([0])
        self.time_table = self.manager.list([[0, 0, 0, 0, 0]])
        self.itor_vec = self.manager.list([0])
        self.sim_q_dict = self.manager.dict()
        self.sim_q_len_vec = self.manager.list([0])
        self.dp_q_dict = {}
        self.working_count = self.manager.Value('i',0)
        self.itor = self.manager.Value('i',0)
        self.search_num = search_num
        
        threading_method = Process if N_agents > 1 else Thread
        self.proc_list = [Process(target=self.searching_proc, args=(id_agent, display, terminate_on_first), kwargs=kwargs) for id_agent in range(N_agents)]
        for proc in self.proc_list:
            proc.start()
        self.dp_thread_list = []
        self.sv_walker = 0
        f = open("stopper.txt", "w")
        f.write("stopper!")
        f.close()

        while not self.stop_now.value:
            t1 = timer.time()
            if PRINT_LOG:
                print('main {} ({}s)'.format(self.sv_walker,round(timer.time()-self.t0,2)))
            len_snode = len(self.snode_vec)
        #     print('np.sum(self.done_vec)')
        #     print(np.sum(self.done_vec))
        #     print('len_snode')
        #     print(len_snode)
        #     print('self.sv_walker')
        #     print(self.sv_walker)
            if not os.path.isfile("stopper.txt"):
                self.stop_now.value = 1
                print("stopper file removed")
                timer.sleep(1e-1)
                break
            if self.itor.value > self.search_num:
                self.stop_now.value = 1
                print("max iteration reached")
                timer.sleep(1e-1)
                break
            self.lock.acquire()
#             print("done: {}".format(np.sum(self.done_vec)))
            if np.sum(self.done_vec) >= len_snode or self.sv_walker >= len_snode:
                if self.sv_walker >= len_snode:
                    self.sv_walker = 0
                if self.working_count.value==0:
                    edep_avail = [k for k,v in self.sim_q_dict.items() if len(v)>0]
                    thread_running = [t for t in self.dp_thread_list if t.is_alive()]
                    if (self.depth_cur.value not in edep_avail) and (not thread_running):
                        new_depth = min(edep_avail) if len(edep_avail)>0 else self.depth_cur.value+1
                        if new_depth<=self.max_depth:
                            self.depth_cur.value = new_depth
                            self.sv_walker = 0
                            print ("register depth_buffer = {}".format(self.depth_cur.value))
                            if self.depth_cur.value in self.dp_q_dict:
                                dp_list = self.dp_q_dict[self.depth_cur.value]
                                self.dp_q_dict[self.depth_cur.value] = []
                                dp_list = sorted(dp_list, key=lambda x: x[0].depth, reverse=True)
                                if PRINT_LOG:
                                    print("depth_Q depth: {}".format([x[0].depth for x in dp_list]))
                                if expand_depth_on_thread:
                                    self.dp_thread_list += [
                                        Thread(target=self.register_depth_buffer, args=(self.depth_cur.value, dp_list))]
                                    self.dp_thread_list[-1].start()
                                else:
                                    for node_leaf in dp_list:
                                        if self.stop_now.value:
                                            break
                                        self.register_sim(*node_leaf)
                            print ("current depth = {}".format(self.depth_cur.value))
                        else:
                            self.stop_now.value = 1
                self.lock.release()
                timer.sleep(1e-1)
                if PRINT_LOG:
                    print('sleep')
                continue
            self.lock.release()
            t2 = timer.time()
        #     print("main t1: %.2f ms"%((t2-t1)*1000))
            if self.done_vec[self.sv_walker]:
                self.sv_walker += 1
                continue
            snode = self.snode_vec[self.sv_walker]
            state = snode.state
            dep = snode.depth
            if PRINT_LOG:
                print("main: snode depth: {}/{} {} ({}s)".format(dep, int(self.depth_cur.value), state.node,round(timer.time()-self.t0,2)))
            if self.goal_cost_dict[state.node] == 0 or dep>=self.depth_cur.value:
                self.sv_walker += 1
                continue
            leafs = self.valid_node_dict[state.node]
            leafs = leafs[:-1]+[leafs[-1]]*self.joint_motion_num
            t3 = timer.time()
        #     print("main t2: %.2f ms"%((t3-t2)*1000))
            for leaf in leafs:
                t3 = timer.time()
                dep_lf = dep+1
                edep = int(dep_lf+self.goal_cost_dict[leaf])
                if edep>self.depth_cur.value:
                    if edep not in self.dp_q_dict:
                        self.dp_q_dict[edep] = []
                    self.dp_q_dict[edep].append((snode, leaf, dep_lf, edep))
                else:
                    self.register_sim(snode, leaf, dep_lf, edep)
                t4 = timer.time()
            self.done_vec[self.sv_walker] = True
            self.sv_walker += 1
        for proc in self.proc_list:
            proc.join()
        for t in self.dp_thread_list:
            t.join()
            
    def register_depth_buffer(self, edepth, dp_list):
        i_dp = 0
        while (not self.stop_now.value) and i_dp<len(dp_list):
            node_leaf = dp_list[i_dp]
            res = self.register_sim(*node_leaf)
            if res:
                i_dp += 1
        print("depth {} full registered".format(int(edepth)))
    
    def register_sim(self, snode, leaf, dep_lf, edep):
        state = snode.state
        if PRINT_LOG:
            print('extracting leaf {}<-{} ({}, {} s)'.format(leaf, state.node, edep, round(timer.time()-self.t0,2)))
        if edep not in self.sim_q_dict:
            self.sim_q_dict[edep] = []
        if len(self.sim_q_dict[edep]) > ConstraintGraph.MAX_LEN_SIM_Q:
            timer.sleep(1e-1)
            return False
        to_state = state.copy()
        if leaf == state.node:
            dQ = (1-2*np.random.rand(self.DOF))*ConstraintGraph.DQ_MAX
            to_state.Q = np.sum([state.Q, dQ], axis=0)
        else:
            to_state.node = leaf
        self.sim_q_lock.acquire()
        eqlist = self.sim_q_dict[edep]
        i_depth = len(filter(lambda x: x[0].depth>=dep_lf, eqlist))
        eqlist.insert(i_depth, (snode, to_state))
        self.sim_q_dict[edep] = eqlist
        self.sim_q_lock.release()
        if PRINT_LOG:
            print("sim_Q depth: {}".format([x[0].depth for x in eqlist]))
        return True
        
    def find_schedules(self):
        self.idx_goal = []
        schedule_dict = {}
        for i in range(len(self.snode_vec)):
            snode = self.snode_vec[i]
            state = snode.state
            if self.check_goal(state, self.goal_state):
                self.idx_goal += [i]
                schedule = snode.parents + [i]
                schedule_dict[i] = schedule
        return schedule_dict
    
    def sort_schedule(self, schedule_dict):
        return sorted(schedule_dict.values(), key=lambda x: len(x))

    def replay(self, schedule, N=400, dt=0.005,**kwargs):
        state_cur = self.snode_vec[schedule[0]].state
        for i_state in schedule[1:]:
            state_new = self.snode_vec[i_state].state
            print('')
            print('-'*20)
            print("{}-{}".format(i_state, state_new.node))
            e, new_state, succ = self.simulate_transition(state_cur, state_new, display=True, N=N, dt=dt,**kwargs)
            state_cur = state_new
        return e
                  
    def check_goal(self, state, goal):
        return all([g is None or s==g for s,g in zip(state.get_tuple(), goal.get_tuple())])
    
    def print_snode_list(self):
        for i_s, snode in zip(range(len(self.snode_vec)),self.snode_vec):
            print("{}{}<-{}{}".format(i_s, snode.state.node, snode.parents[-1] if snode.parents else "", self.snode_vec[snode.parents[-1]].state.node if snode.parents else ""))
            
    def quiver_snodes(self, figsize=(10,10)):
        import matplotlib.pyplot as plt
        N_plot = len(self.snode_vec)
        cost_vec = [self.goal_cost_dict[snode.state.node] for snode in self.snode_vec[1:N_plot]]
        parent_vec = [self.goal_cost_dict[self.snode_vec[snode.parents[-1]].state.node] for snode in self.snode_vec[1:N_plot]]
        plt.figure(figsize=figsize)
        X = list(range(1,N_plot))
        plt.quiver(X, parent_vec, 
                   [0]*(N_plot-1), 
                   np.subtract(cost_vec, parent_vec), 
                   angles='xy', scale_units='xy', scale=1)
        plt.plot(X, cost_vec,'.')
        plt.plot(X, parent_vec,'.')
        plt.axis([0,N_plot+1,-0.5,4.5])
        
    def show_pose(self, pose):
        show_motion([pose], self.marker_list, self.pub, self.joints, self.joint_names)
        