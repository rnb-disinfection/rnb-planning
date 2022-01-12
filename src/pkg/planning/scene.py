from collections import defaultdict
from ..utils.utils import list2dict, save_pickle
from .constraint.constraint_common import \
    combine_redundancy, sample_redundancy, fit_binding, \
    BindingTransform, BindingState
from .constraint.constraint_subject import *
from .constraint.constraint_actor import *
from ..utils.rotation_utils import SE3, Rot_rpy
from ..geometry.geometry import *
from ..geometry.geotype import *
from state import State
from itertools import product
import numpy as np
import random
from copy import deepcopy



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
        ## @brief {actor name: binder instance (rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor)}
        self.actor_dict = {}
        ## @brief {geometry name: actor name}
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
            if rname is not None and rname in self.robot_actor_dict:
                del self.robot_actor_dict[rname]
            for subject in self.subject_dict.values():
                if bname in subject.sub_binders_dict:
                    del subject.sub_binders_dict[bname]

    ##
    # @param bname actor name
    # @param gname name of parent object
    # @param _type type of binder, subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param point binding point offset from object (m). Put None to disable single point constraint and enable surface constraint.
    # @param rpy   orientation of binding point (rad)
    def create_binder(self, bname, gname, _type, point, rpy=(0, 0, 0), **kwargs):
        self.remove_binder(bname)
        geometry = self.gscene.NAME_DICT[gname]
        binder = _type(bname, geometry=geometry, point=point, rpy=rpy, **kwargs)
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
    # @brief get active binders in dictionary
    # @param type_list rnb-planning.src.pkg.planning.constraint.constraint_common.ConstraintType
    def separate_active_binders(self, type_list=None):
        controlled_binders = []
        uncontrolled_binders = []
        for k_b, binder in self.actor_dict.items():
            if type_list is None or binder.ctype in type_list:
                if binder.active:
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
    # @param binding BindingTransform
    def add_subject(self, name, subject, chain=None):
        self.subject_dict[name] = subject
        if chain is not None:
            self.actor_dict[chain.actor_name].bind(self.subject_dict[name], chain.handle_name,
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
    # @brief clear all subjects
    def clear_subjects(self):
        for sname in self.subject_name_list:
            self.remove_subject(sname)

    ##
    # @param oname object name
    # @param gname name of parent object
    # @param _type type of object, subclass of rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param chain Initial BindingChain
    def create_subject(self, oname, gname, _type, chain=None, **kwargs):
        self.remove_subject(oname)
        geometry = self.gscene.NAME_DICT[gname]
        _object = _type(oname, geometry, **kwargs)
        self.add_subject(oname, _object, chain)
        return _object

    ##
    # @brief set object states
    # @param state State
    def set_object_state(self, state):
        chain_list = state.binding_state.get_chains_sorted()
        chain_list_done = []
        chain_list_postponed = set()
        while chain_list:
            chain = chain_list.pop(0)
            obj = self.subject_dict[chain.subject_name]
            state_param = state.state_param[chain.subject_name]
            if None in chain:
                obj.set_state(state.binding_state[chain.subject_name], state_param)
                continue
            binder = self.actor_dict[chain.actor_name]
            binder_family = set(binder.geometry.get_family())
            remaining_subjects = [self.subject_dict[chain_tmp[0]].geometry.name for chain_tmp in chain_list]
            if binder_family.intersection(remaining_subjects)-chain_list_postponed:
                # If binder is a family of remaining subjects, move it to end to change it after the subject is updated
                chain_list.append(chain)
                chain_list_postponed = chain_list_postponed.union(binder_family)
            else:
                obj.set_state(state.binding_state[chain.subject_name], state_param)
                chain_list_done += [chain]
        for actor in self.actor_dict.values():
            actor.update_handle()

    ##
    # @brief get object states
    # @return binding_state tuple of binding state ((object name, binding point, binder, binder_geo), ..)
    # @return pose_dict object pose relative to attached links {name: 4x4 transformation}
    def get_object_state(self):
        binding_state = BindingState()
        state_param = {}
        for k in self.subject_name_list:
            v = self.subject_dict[k]
            sname, hname, aname, _ = v.binding.chain
            subject = self.subject_dict[sname]
            handle =  subject.action_points_dict[hname] if hname in subject.action_points_dict else hname
            actor = self.actor_dict[aname] if aname in self.actor_dict else aname
            binding_state[k] = BindingTransform(subject, handle, actor)
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

    def get_updated_state_param(self, binding_state, state_param):
        state_param_new = {}
        for sname in self.subject_name_list:
            btf = binding_state[sname]
            subject = self.subject_dict[sname]
            state_param_new[sname] = subject.get_updated_state_param(btf, state_param[btf.chain.subject_name])
        return state_param_new

    def get_node(self, binding_state, state_param):
        node = []
        for sname, stype in zip(self.subject_name_list, self.subject_type_list):
            btf = binding_state[sname]
            node.append(stype.get_node_component(btf, state_param[sname]))
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
        return sorted(set(neighbor_list))

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
    # @param chain    BindingChain (object name, binding point, binder)
    # @param joint_dict joint pose in radian as dictionary
    def rebind(self, chain, joint_dict, done_log=[]):
        object_tar = self.subject_dict[chain.subject_name]
        obj_fam = object_tar.geometry.get_family()
        if chain.actor_name is None:
            object_tar.set_state(BindingTransform(object_tar, None, None, None), None)
        else:
            binder = self.actor_dict[chain.actor_name]
            binder.bind(action_obj=object_tar, bind_point=chain.handle_name, joint_dict_last=joint_dict)   # bind given chain
        done_log = done_log + [(chain.subject_name, chain.actor_name)]
        for binder_sub in [k for k,v in self.actor_dict.items()
                           if v.geometry.name in obj_fam]: # find bound object's sub-binder
            for binding_sub in [v.binding.chain for v in self.subject_dict.values()
                                if v.binding.chain.actor_name == binder_sub]: # find object bound to the sub-binder
                if (binding_sub.subject_name, binding_sub.actor_name) not in done_log:
                    self.rebind(binding_sub, joint_dict, done_log=done_log) # update binding of the sub-binder too


    ##
    # @brief change binding state
    # @param binding    list of BindingChain [(object name, binding point, binder), ..]
    # @param Q          joint pose in radian array
    # @return           rebinded new state
    def rebind_all(self, chain_list, Q):
        for chain in chain_list:
            self.rebind(chain, list2dict(Q, self.gscene.joint_names))

        binding_state, state_param = self.get_object_state()
        end_state = State(binding_state, state_param, list(Q), self)
        return end_state

    ##
    # @brief get exact bindings to transit
    # @param from_state State
    # @param to_state   State
    def get_changing_subjects(self, from_state, to_state, check_available=True):
        subject_list = []
        btf_list = []
        if to_state.binding_state is not None:
            for sname in self.subject_name_list:
                btf0 = from_state.binding_state[sname]
                btf1 = to_state.binding_state[sname]
                # if bd0[2] != bd1[2]: # check if new transition (slack) ## this was when sweep was not added
                if btf0.get_chain() != btf1.get_chain(): # check if new transition (slack)
                    subject_list.append(sname)
                    btf_list.append(btf1)
                else:
                    # print("bd0 :{}".format(bd0))
                    # print("bd1 :{}".format(bd1))
                    # assert bd0[1] == bd1[1] , "impossible transition" ## removed because sweep action, which use same binder
                    pass

        success = True
        if check_available:
            for btf1 in btf_list:
                if not self.actor_dict[btf1.chain.actor_name].check_available(
                        list2dict(from_state.Q, self.gscene.joint_names)):
                    success = False
            # else: # commenting out this because there's no need to filter out no-motion transitions
            #     if from_state.Q is not None and to_state.Q is not None:
            #         if np.sum(np.abs(from_state.Q - to_state.Q))<1e-3:
            #             success = False
        return subject_list, success

    def is_constrained_transition(self, from_state, to_state, check_available=True):
        diffs, _ = self.get_changing_subjects(from_state, to_state, check_available=check_available)
        for obj_name in diffs:
            btf_from = from_state.binding_state[obj_name]
            btf_to = to_state.binding_state[obj_name]
            constraints = self.subject_dict[obj_name].make_constraints(btf_from.get_chain(),
                                                                       btf_to.get_chain())
            if len(constraints) > 0:
                return True
        return False

    ##
    # @brief get current scene state
    # @force_fit_binding    force each bindings to be perfectly matched geometrically
    def initialize_state(self, Q, force_fit_binding=False, Poffset=None):
        ## calculate binder transformations
        Q_dict = list2dict(Q, self.gscene.joint_names)

        ## get current binding state
        chain_list = []
        for kobj in self.subject_name_list:
            chain = self.subject_dict[kobj].get_initial_chain(self.actor_dict, Q_dict)
            chain_list.append(chain)

        if force_fit_binding:
            objects_to_update = [obj for obj in self.subject_dict.values() if obj.stype == SubjectType.OBJECT]
            while objects_to_update:
                obj = objects_to_update.pop(0)
                oname, hname, bname, bgname = obj.get_initial_chain(self.actor_dict, self.combined_robot.home_dict)
                if any([bgname in box_tmp.geometry.get_family() for box_tmp in objects_to_update]):  # binder will be updated
                    objects_to_update.append(obj)  # move to end to use updated binder info
                    continue
                fit_binding(obj, obj.action_points_dict[hname], self.actor_dict[bname], self.combined_robot.home_dict,
                            Poffset=Poffset)

        return self.rebind_all(chain_list=chain_list, Q=Q)

    ##
    # @brief    get a dictionary of available bindings in object-binding geometry level =
    #           **object**-point-binder- **geometry**
    # @return   {object name: actor root geometry name}
    def get_available_actor_dict(self):
        available_actor_dict = defaultdict(list)
        for bname in self.geometry_actor_dict.keys():
            for oname in self.subject_name_list:
                pass_now = False
                for binder_name in self.geometry_actor_dict[bname]:
                    binder = self.actor_dict[binder_name]
                    for ap in self.subject_dict[oname].action_points_dict.values():
                        if binder.check_pair(ap):
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
    # @return {object name: [(point name, actor name)]}
    def get_available_binding_dict(self, state, to_node, Q_dict=None):
        self.set_object_state(state)
        if Q_dict is None:
            Q_dict = list2dict(state.Q, self.gscene.joint_names)
        available_binding_dict = {}
        for oname, to_node_item, from_node_item in zip(
                self.subject_name_list, to_node, state.node):
            from_binding = state.binding_state[oname].get_chain()
            # bgname: actor root geometry name
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
    #           {object name: [(point name, actor name)]}
    # @param    to_node target object-level node
    # @param    binding_sampler random sampling function to be apllied to available binding list [(point name, actor name)]
    #                           default=random.choice
    # @param    sampler         sampling function to be applied to redundancy param (default=random.uniform)
    # @return   (sampled next state, sampled redundancy)
    def sample_leaf_state(self, state, available_binding_dict, to_node,
                          binding_sampler=random.choice, redundancy_sampler=random.uniform):
        self.set_object_state(state)
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
                # print("============= try go home ({}) ===================".format(rname))
                return to_state
        to_binding_state = BindingState()
        for sname, from_ntem, to_ntem in zip(self.subject_name_list, state.node, to_node):
            btf_from = state.binding_state[sname]
            if from_ntem == to_ntem:
                btf_to = deepcopy(btf_from)
            else:
                hname_to, bname_to, _ = binding_sampler(available_binding_dict[sname])
                obj = self.subject_dict[sname]
                to_ap = obj.action_points_dict[hname_to]
                to_binder = self.actor_dict[bname_to]
                redundancy_tot = combine_redundancy(to_ap, to_binder, state.Q)
                redundancy = sample_redundancy(redundancy_tot, sampler=redundancy_sampler)
                btf_to = BindingTransform(obj, to_ap, to_binder, redundancy)
            to_binding_state[sname] = btf_to
        to_state.set_binding_state(self, to_binding_state, to_state.state_param)
        return to_state

    ##
    # @brief    add axis marker to handle
    def add_handle_axis(self, hl_key, handle, Toff=None, color=None, axis="xyz", dims=(0.10, 0.01, 0.01), idx=""):
        hobj = handle.geometry
        Toff_lh = handle.Toff_lh
        if Toff is not None:
            Toff_lh = np.matmul(Toff_lh, Toff)
        self.gscene.add_highlight_axis(hl_key, hobj.name+idx, hobj.link_name, Toff_lh[:3,3], Toff_lh[:3,:3],
                                       color=color, axis=axis, dims=dims)

    ##
    # @brief    add axis marker to handle
    # @param btf BindingTransform
    def show_binding(self, btf, axis="xyz", color=None, dims=(0.10, 0.01, 0.01), idx=""):
        sname, hname, aname, agname = btf.get_chain()
        if hname is not None:
            handle = self.subject_dict[sname].action_points_dict[hname]
            if aname is not None:
                actor = self.actor_dict[aname]
                self.add_handle_axis("{}_{}".format(sname, hname), handle,
                                     Toff=np.matmul(btf.T_add_handle, SE3_inv(btf.T_add_actor)), axis=axis, color=color,
                                     dims=dims, idx=idx)
            else:
                self.add_handle_axis("{}_{}".format(sname, hname), handle, Toff=btf.T_add_handle, axis=axis, color=color,
                                     dims=dims, idx=idx)
        if aname is not None:
            actor = self.actor_dict[aname]
            if hname is None:
                self.add_handle_axis("{}".format(aname), actor, Toff=btf.T_add_actor, axis=axis, color=color,
                                     dims=dims, idx=idx)
            else:
                self.add_handle_axis("{}".format(aname), actor, Toff=np.identity(4), axis=axis, color=color,
                                     dims=dims, idx=idx)

    def get_scene_args(self, Q):
        gtem_args = self.gscene.get_gtem_args()
        binder_args = {aname: actor.get_args() for aname, actor in self.actor_dict.items()}
        subject_args = {sname: subject.get_args() for sname, subject in self.subject_dict.items()}
        subject_names = self.subject_name_list
        binding_state, state_param = self.get_object_state()
        state = State(binding_state, state_param, list(Q), self)
        return {"gtem_args": gtem_args,
                "binder_args": binder_args,
                "subject_args": subject_args,
                "subject_names": subject_names,
                "state": state}

    def recover_scene_args(self, scene_args):
        self.clear_subjects()
        for bname in sorted(self.actor_dict.keys()):
            self.remove_binder(bname)

        load_gtem_args(self.gscene, scene_args['gtem_args'])

        for bname, bargs in scene_args['binder_args'].items():
            self.create_binder(bargs["name"], bargs["gname"], _type=eval(bargs["type"]),
                               point=bargs["point"], rpy=bargs["rpy"])

        subject_args = scene_args['subject_args']
        for sname in scene_args['subject_names']:
            sargs = subject_args[sname]
            self.create_subject(sargs["name"], gname=sargs["gname"], _type=eval(sargs["type"]),
                                action_points_dict={aname:
                                                        eval(ap_args["type"])(ap_args["name"],
                                                                              self.gscene.NAME_DICT[ap_args["gname"]],
                                                                              point=ap_args["point"], rpy=ap_args["rpy"],
                                                                              name_full=ap_args["name_full"]
                                                                              )
                                                    for aname, ap_args in sargs["action_points_dict"].items()},
                                sub_binders_dict={bname: self.actor_dict[bname] for bname in sargs["sub_binders"]},
                                **sargs['kwargs'])

        self.set_object_state(scene_args['state'])
