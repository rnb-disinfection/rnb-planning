from collections import defaultdict
from ..utils.utils import list2dict
from .constraint.constraint_common import combine_redundancy, sample_redundancy, calc_redundancy, fit_binding
from .constraint.constraint_subject import SubjectType
from ..utils.rotation_utils import SE3, Rot_rpy
from ..geometry.geotype import *
from itertools import product
import numpy as np
import random
from copy import deepcopy


##
# @class    State
# @brief    planning scene state
class State:
    ##
    # @param binding_state           tuple of binding state ((object name, binding point, binder), ..)
    # @param state_param    object pose dictionary {object name: 4x4 offset relative to attached link}
    # @param Q              robot joint configuration
    # @param pscene         PlanningScene instance
    def __init__(self, binding_state, state_param, Q, pscene):
        self.state_param = state_param if state_param is not None else defaultdict(lambda:None)
        self.Q = np.array(Q)
        self.set_binding_state(binding_state, pscene)

    def set_binding_state(self, binding_state, pscene):
        ## @brief tuple of binding state ((object name, binding point, binder), ..)
        self.binding_state = binding_state
        self.state_param = pscene.get_state_param_update(self.binding_state, self.state_param)
        ## @brief tuple of simplified binding state (binder geometry name 1, binder geometry name 2, ..)
        self.node = pscene.get_node(self.binding_state, self.state_param)

    def get_tuple(self):
        return (self.binding_state, self.state_param, self.Q)

    def copy(self, pscene):
        return State(deepcopy(self.binding_state), deepcopy(self.state_param), deepcopy(self.Q), pscene)



##
# @class    PlanningScene
# @brief    planning scene
class PlanningScene:
    ##
    # @param gscene instance of rnb-planning.src.pkg.geometry.geometry.GeometryScene
    # @param combined_robot instance of rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
    def __init__(self, gscene, combined_robot):
        ## @brief rnb-planning.src.pkg.geometry.geometry.GeometryScene
        self.gscene = gscene
        ## @brief rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
        self.combined_robot = combined_robot
        ## @brief {binder name: binder instance (rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor)}
        self.actor_dict = {}
        ## @brief {geometry name: binder name}
        self.geometry_actor_dict = defaultdict(list)
        ## @brief {object name: handle name)}
        self.handle_dict = {}
        ## @brief list of tuples (object name, handle name)
        self.handle_list = []
        ## @brief {action name, action instance (rnb-planning.src.pkg.planning.constraint.constraint_subject.Action)}
        self.subject_dict = {}
        ## @brief list of subject names in order
        self.subject_name_list = []
        ## @brief list of subject types in order
        self.subject_type_list = []
        ## @brief {actor name: corresponding robot name}
        self.actor_robot_dict = {}
        ## @brief {robot name: corresponding actor name}
        self.robot_actor_dict = {}
        # set robot chain from ComrinedRobot and GeometryScene
        self.set_robot_chain_dict()

    ##
    # @brief add a binder to the scene
    # @param rname indexed full name of corresponding robot
    # @param binder instance of subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    def add_binder(self, binder):
        rnames = [rname for rname, chain_dict in self.robot_chain_dict.items()
                 if binder.geometry.link_name in chain_dict["link_names"]]
        if len(rnames)==1:
            rname = rnames[0]
        elif len(rnames)==0:
            rname = None
        else:
            raise(RuntimeError(
                "binder link included in multiple robot chains - {} - {}".format(binder.name,
                                                                                 binder.geometry.link_name)))
        self.actor_robot_dict[binder.name] = rname
        if rname is not None:
            self.robot_actor_dict[rname] = binder.name
        self.actor_dict[binder.name] = binder
        self.geometry_actor_dict[binder.geometry.name].append(binder.name)

    ##
    # @brief remove a binder from the scene
    def remove_binder(self, bname):
        if bname in self.actor_dict:
            self.geometry_actor_dict[self.actor_dict[bname].geometry.name].remove(bname)
            del self.actor_dict[bname]
            rname = self.actor_robot_dict[bname]
            del self.actor_robot_dict[bname]
            if rname is not None:
                del self.robot_actor_dict[rname]
            for subject in self.subject_dict.values():
                if bname in subject.sub_binders_dict:
                    del subject.sub_binders_dict[bname]

    ##
    # @param bname binder name
    # @param gname name of parent object
    # @param _type type of binder, subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param point binding point offset from object (m)
    # @param rpy   orientation of binding point (rad)
    def create_binder(self, bname, gname, _type, point=None, rpy=(0, 0, 0)):
        self.remove_binder(bname)
        geometry = self.gscene.NAME_DICT[gname]
        binder = _type(bname, geometry=geometry, point=point, rpy=rpy)
        self.add_binder(binder)
        return binder

    ##
    # @brief get unique binders in dictionary
    def get_unique_binders(self):
        uniq_binders = []
        for k_b, binder in self.actor_dict.items():
            if not binder.multiple:
                uniq_binders += [k_b]
        return uniq_binders

    ##
    # @brief get unique binder geometries
    def get_unique_binder_geometries(self):
        uniq_binders = []
        for k_b, binder in self.actor_dict.items():
            if not binder.multiple:
                uniq_binders += [binder.geometry.name]
        return uniq_binders

    ##
    # @brief get controlled binders in dictionary
    # @param type_list rnb-planning.src.pkg.planning.constraint.constraint_common.ConstraintType
    def divide_binders_by_control(self, type_list=None):
        controlled_binders = []
        uncontrolled_binders = []
        for k_b, binder in self.actor_dict.items():
            if type_list is None or binder.ctype in type_list:
                if binder.controlled:
                    controlled_binders += [k_b]
                else:
                    uncontrolled_binders += [k_b]
        return controlled_binders, uncontrolled_binders

    ##
    # @brief set robot chain dictionary {robot name: {"tip_link": effector link name, "joint_names": joint name list}}
    def set_robot_chain_dict(self):
        base_dict = self.combined_robot.get_robot_base_dict()
        tip_dict = self.combined_robot.get_robot_tip_dict()
        ## @brief {robot name: {"tip_link": effector link name, "joint_names": joint name list}}
        self.robot_chain_dict = {}
        for rname in self.combined_robot.robot_names:
            effector_link = tip_dict[rname]
            base_link = base_dict[rname]
            joint_chain = self.gscene.urdf_content.get_chain(
                root=base_link, tip=effector_link, fixed=False, joints=True, links=False)
            link_chain = self.gscene.urdf_content.get_chain(
                root=base_link, tip=effector_link, fixed=False, joints=False, links=True)
            self.robot_chain_dict[rname] = {"tip_link": effector_link,
                                            "joint_names": joint_chain,
                                            "link_names": link_chain}

    ##
    # @brief add a subjct to the scene
    def add_subject(self, name, subject, binding=None):
        self.subject_dict[name] = subject
        if binding is not None:
            self.actor_dict[binding[1]].bind(self.subject_dict[name], binding[0],
                                              list2dict([0] * len(self.gscene.joint_names),
                                                        item_names=self.gscene.joint_names))
        self.update_subjects()

    ##
    # @brief remove a object from the scene
    def remove_subject(self, name):
        if name in self.subject_dict:
            del self.subject_dict[name]
        self.update_subjects()

    ##
    # @param oname object name
    # @param gname name of parent object
    # @param _type type of object, subclass of rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param binding point offset from object (m)
    def create_subject(self, oname, gname, _type, binding=None, **kwargs):
        self.remove_subject(oname)
        geometry = self.gscene.NAME_DICT[gname]
        _object = _type(oname, geometry, **kwargs)
        self.add_subject(oname, _object, binding)
        return _object

    ##
    # @brief set object states
    # @param state State
    def set_object_state(self, state):
        bd_list = list(state.binding_state)
        bd_list_done = []
        while bd_list:
            bd = bd_list.pop(0)
            obj = self.subject_dict[bd[0]]
            state_param = state.state_param[bd[0]]
            if None in bd:
                obj.set_state(bd, state_param)
                continue
            binder = self.actor_dict[bd[2]]
            binder_family = set(binder.geometry.get_family())
            remaining_subjects = [self.subject_dict[bd_tmp[0]].geometry.name for bd_tmp in bd_list]
            if binder_family.intersection(remaining_subjects):
                # If binder is a family of remaining subjects, move it to end to change it after the subject is updated
                bd_list += [bd]
            else:
                obj.set_state(bd, state_param)
                bd_list_done += [bd]
        for actor in self.actor_dict.values():
            actor.update_handle()

    ##
    # @brief get object states
    # @return binding_state tuple of binding state ((object name, binding point, binder), ..)
    # @return pose_dict object pose relative to attached links {name: 4x4 transformation}
    def get_object_state(self):
        binding_state = ()
        state_param = {}
        for k in self.subject_name_list:
            v = self.subject_dict[k]
            binding_state += (v.binding,)
            state_param[k] = v.get_state_param()
        return binding_state, state_param

    # get all existing type-checked nodes
    def get_all_nodes(self):
        node_components_list = []  # get all nodes
        for sname in self.subject_name_list:
            node_components_list.append(self.subject_dict[sname].get_all_node_components(self))
        type_checked_nodes = list(product(*node_components_list))

        no_self_binding_nodes = []  # filter out self-binding
        for node in type_checked_nodes:
            if all([a != b for a, b in zip(node, self.subject_name_list)]):
                no_self_binding_nodes.append(node)

        uniq_binder_geometries = self.get_unique_binder_geometries()  # binders cannot be shared by multiple objects
        available_nodes = []  # filter out duplicate unique binding
        for node in no_self_binding_nodes:
            if np.all([np.sum([a == gname for a in node]) <= 1 for gname in uniq_binder_geometries]):
                available_nodes.append(node)
        return available_nodes

    def get_state_param_update(self, binding_state, state_param):
        state_param_new = {}
        for sname, binding in zip(self.subject_name_list, binding_state):
            subject = self.subject_dict[sname]
            state_param_new[sname] = subject.get_state_param_update(binding, state_param[binding[0]])
        return state_param_new

    def get_node(self, binding_state, state_param):
        node = []
        for stype, binding in zip(self.subject_type_list, binding_state):
            node.append(stype.get_node_component(binding, state_param[binding[0]]))
        return tuple(node)

    def get_node_neighbor(self, node, include_self=False):
        neighbor_list = []
        len_node = len(node)
        for i, sname in enumerate(self.subject_name_list):  # make transition one by one
            new_node_component_list = self.subject_dict[sname].get_neighbor_node_component_list(node[i], self)
            for new_node_component in new_node_component_list:
                node_new = deepcopy(node)
                node_new = node_new[:i] + (new_node_component,) + (node_new[i + 1:] if len_node > 1 else ())
                if node_new != node or include_self:
                    neighbor_list.append(node_new)
        return neighbor_list

    ##
    # @brief get all handles in the scene
    # @return handles all handle items in the scene
    def get_all_handles(self):
        handles = []
        for obj_hd in self.subject_dict.values():
            handles += obj_hd.action_points_dict.values()
        return handles

    ##
    # @brief get all handles in the scene as a dictionary
    # @return handle_dict all handle items in the scene as a dictionary
    def get_all_handle_dict(self):
        handle_dict = {}
        for obj_hd in self.subject_dict.values():
            for hd in obj_hd.action_points_dict.values():
                handle_dict[hd.name_full] = hd
        return handle_dict

    ##
    # @brief delete specific handle by handle name
    def delete_handle(self, htem):
        otem = self.subject_dict[htem.geometry.name]
        del otem.action_points_dict[htem.name]
        if not otem.action_points_dict.keys():
            self.remove_subject(htem.geometry.name)

    ##
    # @brief    collect all objects' handle items and update to the scene,
    #           automatically called when object is added/removed.
    #           updates: handle_dict, handle_list, object_list.
    def update_subjects(self):
        self.handle_dict = {}
        self.handle_list = []
        self.subject_name_list = sorted(self.subject_dict.keys())
        self.subject_type_list = []
        for k in self.subject_name_list:
            v = self.subject_dict[k]
            self.subject_type_list.append(v.__class__)
            ap_dict = v.action_points_dict
            self.handle_dict[k] = []
            for ap in ap_dict.keys():
                self.handle_dict[k].append(ap)
                self.handle_list += [(k, ap)]

    ##
    # @brief change binding state
    # @param binding    binding tuple (object name, binding point, binder)
    # @param joint_dict joint pose in radian as dictionary
    def rebind(self, binding, joint_dict):
        object_tar = self.subject_dict[binding[0]]
        if binding[2] is None:
            object_tar.set_state(binding, None)
        else:
            binder = self.actor_dict[binding[2]]
            binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict)   # bind given binding
        for binder_sub in [k for k,v in self.actor_dict.items()
                           if v.geometry == object_tar.geometry]: # find bound object's sub-binder
            for binding_sub in [v.binding for v in self.subject_dict.values()
                                if v.binding[2] == binder_sub]: # find object bound to the sub-binder
                self.rebind(binding_sub, joint_dict) # update binding of the sub-binder too

    ##
    # @brief change binding state
    # @param binding    list of binding tuple [(object name, binding point, binder), ..]
    # @param Q          joint pose in radian array
    # @return           rebinded new state
    def rebind_all(self, binding_list, Q):
        for bd in binding_list:
            self.rebind(bd, list2dict(Q, self.gscene.joint_names))

        binding_state, state_param = self.get_object_state()
        end_state = State(binding_state, state_param, list(Q), self)
        return end_state

    ##
    # @brief get exact bindings to transit
    # @param from_state State
    # @param to_state   State
    def get_slack_bindings(self, from_state, to_state):
        binding_list = []
        if to_state.binding_state is not None:
            for bd0, bd1 in zip(from_state.binding_state, to_state.binding_state):
                # if bd0[2] != bd1[2]: # check if new transition (slack) ## this was when sweep was not added
                if bd0 != bd1: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    # print("bd0 :{}".format(bd0))
                    # print("bd1 :{}".format(bd1))
                    # assert bd0[1] == bd1[1] , "impossible transition" ## removed because sweep action, which use same binder
                    pass

        success = True
        if len(binding_list)>0:
            for binding in binding_list:
                if not self.actor_dict[binding[2]].check_available(
                        list2dict(from_state.Q, self.gscene.joint_names)):
                    success = False
        else:
            if from_state.Q is not None and to_state.Q is not None:
                if np.sum(np.abs(from_state.Q - to_state.Q))<1e-3:
                    success = False
        return binding_list, success

    ##
    # @brief get current scene state
    # @force_fit_binding    force each bindings to be perfectly matched geometrically
    def initialize_state(self, Q, force_fit_binding=False):
        ## calculate binder transformations
        Q_dict = list2dict(Q, self.gscene.joint_names)

        ## get current binding state
        binding_state = []
        for kobj in self.subject_name_list:
            binding = self.subject_dict[kobj].get_initial_binding(self.actor_dict, Q_dict)
            binding_state.append(binding)

        if force_fit_binding:
            objects_to_update = [obj for obj in self.subject_dict.values() if obj.stype == SubjectType.OBJECT]
            while objects_to_update:
                obj = objects_to_update.pop(0)
                oname, hname, bname, bgname = obj.get_initial_binding(self.actor_dict, self.combined_robot.home_dict)
                if any([bgname in box_tmp.geometry.get_family() for box_tmp in objects_to_update]):  # binder will be updated
                    objects_to_update.append(obj)  # move to end to use updated binder info
                    continue
                fit_binding(obj, obj.action_points_dict[hname], self.actor_dict[bname], self.combined_robot.home_dict)

        return self.rebind_all(binding_list=binding_state, Q=Q)

    ##
    # @brief get goal nodes that link object to target binder
    # @param initial_binding_state   initial binding state
    # @param obj            object name
    # @param binder_geo         target binder geometry name
    # @return type checked binding_states
    def get_goal_states(self, initial_binding_state, obj, binder_geo):
        _object = self.subject_dict[obj]
        type_checked_bindings = []
        for bname in self.geometry_actor_dict[binder_geo]:
            _binder = self.actor_dict[bname]
            for k, v in _object.action_points_dict.items():
                if _binder.check_type(v):
                    type_checked_bindings.append((obj, k, bname, binder_geo))
        binding_state_list = [tuple(binding if bd[0] == binding[0]\
                                    else bd for bd in initial_binding_state)\
                              for binding in type_checked_bindings]

        return [State(binding_state, None, None,self) for binding_state in binding_state_list]

    ##
    # @brief make goal state that with specific binding target
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param obj        object name
    # @param handle     handle name
    # @param binder     binder name
    def make_goal_state(self, from_state, obj, handle, binder):
        to_state = from_state.copy(self)
        to_state.binding_state = tuple([(obj, handle, binder, self.actor_dict[binder].geometry.name) if binding[0] == obj else binding for binding in to_state.binding_state])
        return to_state

    ##
    # @brief    get a dictionary of available bindings in object-binding geometry level =
    #           **object**-point-binder- **geometry**
    # @return   {object name: binder geometry name}
    def get_available_actor_dict(self):
        available_actor_dict = defaultdict(list)
        for bname in self.geometry_actor_dict.keys():
            for oname in self.subject_name_list:
                pass_now = False
                for binder_name in self.geometry_actor_dict[bname]:
                    binder = self.actor_dict[binder_name]
                    for ap in self.subject_dict[oname].action_points_dict.values():
                        if binder.check_type(ap):
                            available_actor_dict[oname].append(binder.geometry.name)
                            pass_now = True
                            break
                    if pass_now:
                        break
        return available_actor_dict

    ##
    # @brief get dictionary of available bindings for given transition
    # @param state current state
    # @param to_node target object-level node
    # @param Q_dict current joint configuration in dictionary
    # @return {object name: [(point name, binder name)]}
    def get_available_binding_dict(self, state, to_node, Q_dict=None):
        if Q_dict is None:
            Q_dict = list2dict(state.Q, self.gscene.joint_names)
        available_binding_dict = {}
        for oname, to_node_item, from_node_item, from_binding in zip(
                self.subject_name_list, to_node, state.node, state.binding_state):
            # bgname: binder geometry name
            # sbgname: state
            if from_node_item != to_node_item:
                available_binding_dict[oname] = self.subject_dict[oname].get_available_bindings(
                    from_binding, to_node_item, actor_dict=self.actor_dict, Q_dict=Q_dict)
            else:
                available_binding_dict[oname] = [from_binding[1:]]
        return available_binding_dict

    ##
    # @brief    sample next state for given transition
    # @param    state current state
    # @param    available_binding_dict pre-extracted available bindings for each object
    #           {object name: [(point name, binder name)]}
    # @param    to_node target object-level node
    # @param    binding_sampler random sampling function to be apllied to available binding list [(point name, binder name)]
    #                           default=random.choice
    # @param    sampler         sampling function to be applied to redundancy param (default=random.uniform)
    # @return   (sampled next state, sampled redundancy)
    def sample_leaf_state(self, state, available_binding_dict, to_node,
                          binding_sampler=random.choice, redundancy_sampler=random.uniform):
        to_state = state.copy(self)
        if state.node == to_node:
            dQ = (self.combined_robot.home_pose - to_state.Q)
            rcandis = [rname
                       for rname in self.combined_robot.robot_names
                       if np.sum(np.abs(dQ[self.combined_robot.idx_dict[rname]])) > 1e-4]
            if len(rcandis)>0:
                rname = random.choice(rcandis)
                to_state.Q[self.combined_robot.idx_dict[rname]] = \
                    self.combined_robot.home_pose[self.combined_robot.idx_dict[rname]]
                redundancy_dict = {}
                print("============= try go home ({}) ===================".format(rname))
                return to_state, redundancy_dict
        to_binding_state = tuple([(((oname,)+\
                           binding_sampler(available_binding_dict[oname]))
                          if sbgname!=bgname else sbinding)
                         for oname, bgname, sbgname, sbinding
                         in zip(self.subject_name_list, to_node, state.node, state.binding_state)])
        to_state.set_binding_state(to_binding_state, self)
        redundancy_dict = {}
        for from_binding, to_binding in zip(state.binding_state, to_binding_state):
            if None in to_binding:
                continue
            obj = self.subject_dict[from_binding[0]]
            to_ap = obj.action_points_dict[to_binding[1]]
            to_binder = self.actor_dict[to_binding[2]]
            redundancy_tot = combine_redundancy(to_ap, to_binder)
            redundancy = sample_redundancy(redundancy_tot, sampler=redundancy_sampler)
            redundancy_dict[from_binding[0]] = redundancy
        return to_state, redundancy_dict

    ##
    # @brief    add axis marker to handle
    def add_handle_axis(self, hl_key, handle, Toff=None, color=None):
        hobj = handle.geometry
        Toff_lh = handle.Toff_lh
        if Toff is not None:
            Toff_lh = np.matmul(Toff_lh, Toff)
        axis = "xyz"
        self.gscene.add_highlight_axis(hl_key, hobj.name, hobj.link_name, Toff_lh[:3,3], Toff_lh[:3,:3], color=color, axis=axis)

    ##
    # @brief    add axis marker to handle
    # @param binding tuple (subject name, handle name, binder name, binder geometry name)
    # @param redundancy_dict defined redundancy of transition in dictionary form, {object name: {axis: value}}
    def show_binding(self, binding, redundancy_dict):
        sname, hname, bname, bgname = binding
        redundancy = redundancy_dict[sname] if sname in redundancy_dict else {}
        if hname is not None:
            handle = self.subject_dict[sname].action_points_dict[hname]
            Toff = None
            if hname in redundancy:
                point_add_handle, rpy_add_handle = calc_redundancy(redundancy[hname], handle)
                Toff = SE3(Rot_rpy(rpy_add_handle), point_add_handle)
            self.add_handle_axis("{}_{}".format(sname, hname), handle, Toff=Toff)
        if bname is not None:
            binder = self.actor_dict[bname]
            Toff = None
            if bname in redundancy:
                point_add_binder, rpy_add_binder = calc_redundancy(redundancy[bname], binder)
                Toff = SE3(Rot_rpy(rpy_add_binder), point_add_binder)
            self.add_handle_axis("{}".format(bname), binder, Toff=Toff)