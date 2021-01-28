from collections import defaultdict
from ..utils.utils import list2dict
from .constraint.constraint_common import combine_redundancy, sample_redundancy
import numpy as np
import random


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
        self.Q = Q
        self.set_binding_state(binding_state, pscene)

    def set_binding_state(self, binding_state, pscene):
        ## @brief tuple of binding state ((object name, binding point, binder), ..)
        self.binding_state = binding_state
        ## @brief tuple of simplified binding state (binder geometry name 1, binder geometry name 2, ..)
        self.node = State.get_node(pscene, self.binding_state, self.state_param)

    def get_tuple(self):
        return (self.binding_state, self.state_param, self.Q)

    def copy(self, pscene):
        return State(self.binding_state, self.state_param, self.Q, pscene)

    def __str__(self):
        return str((self.binding_state,
                    {k: str(np.round(v, 2)) for k, v in
                     self.state_param.items()} if self.state_param is not None else None,
                    str(np.round(self.Q, 2)) if self.Q is not None else None))

    @classmethod
    def get_node(cls, pscene, binding_state, state_param):
        node = []
        for stype, binding in zip(pscene.subject_type_list, binding_state):
            node.append(stype.get_node_component(binding, state_param[binding[0]]))
        return tuple(node)



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

    ##
    # @brief add a binder to the scene
    def add_binder(self, binder):
        self.actor_dict[binder.name] = binder
        self.geometry_actor_dict[binder.geometry.name].append(binder.name)

    ##
    # @brief remove a binder from the scene
    def remove_binder(self, bname):
        if bname in self.actor_dict:
            self.geometry_actor_dict[self.actor_dict[bname].geometry.name].remove(bname)
            del self.actor_dict[bname]

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
    # @brief get controlled binders in dictionary
    def get_controlled_binders(self):
        controlled_binders = []
        for k_b, binder in self.actor_dict.items():
            if binder.controlled:
                controlled_binders += [k_b]
        return controlled_binders

    ##
    # @brief add a object to the scene
    def add_object(self, name, _object, binding=None):
        self.subject_dict[name] = _object
        if binding is not None:
            self.actor_dict[binding[1]].bind(self.subject_dict[name], binding[0],
                                              list2dict([0] * len(self.gscene.joint_names),
                                                        item_names=self.gscene.joint_names))
        self.update_subjects()

    ##
    # @brief remove a object from the scene
    def remove_object(self, name):
        if name in self.subject_dict:
            del self.subject_dict[name]
        self.update_subjects()

    ##
    # @param oname object name
    # @param gname name of parent object
    # @param _type type of object, subclass of rnb-planning.src.pkg.planning.constraint.constraint_subject.ObjectBinding
    # @param binding point offset from object (m)
    def create_object(self, oname, gname, _type, binding=None, **kwargs):
        self.remove_object(oname)
        geometry = self.gscene.NAME_DICT[gname]
        _object = _type(oname, geometry, **kwargs)
        self.add_object(oname, _object, binding)
        return _object

    ##
    # @brief set object states
    # @param state State
    def set_object_state(self, state):
        bd_list = list(state.binding_state)
        bd_list_done = []
        while bd_list:
            bd = bd_list.pop(0)
            binder = self.actor_dict[bd[2]]
            if binder.geometry in [self.subject_dict[bd_tmp[0]].geometry for bd_tmp in bd_list]:
                bd_list += [bd] # prevent using previous info ( move back to end )
            else:
                obj = self.subject_dict[bd[0]]
                state_param = state.state_param[bd[0]]
                binder.link_name = state_param[0] # sync linke name with parent
                frame = state_param[1]
                obj.set_state(bd, (binder.link_name, frame))
                bd_list_done += [bd]

    ##
    # @brief get object states
    # @return binding_state tuple of binding state ((object name, binding point, binder), ..)
    # @return pose_dict object pose relative to attached links {name: 4x4 transformation}
    def get_object_state(self):
        binding_state = ()
        pose_dict = {}
        for k in self.subject_name_list:
            v = self.subject_dict[k]
            binding_state += (v.binding,)
            pose_dict[k] = (v.get_state_param())
        return binding_state, pose_dict

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
            self.remove_object(htem.geometry.name)

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
            ap_list = v.action_points_dict
            self.handle_dict[k] = []
            for ap in ap_list.keys():
                self.handle_dict[k].append(ap)
                self.handle_list += [(k, ap)]

    ##
    # @brief change binding state
    # @param binding    binding tuple (object name, binding point, binder)
    # @param joint_dict joint pose in radian as dictionary
    def rebind(self, binding, joint_dict):
        binder = self.actor_dict[binding[2]]
        object_tar = self.subject_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict) # bind given binding
        for binder_sub in [k for k,v in self.actor_dict.items()
                           if v.geometry == object_tar.geometry]: # find bound object's sub-binder
            for binding_sub in [v.binding for v in self.subject_dict.values()
                                if v.binding[2] == binder_sub]: # find object bound to the sub-binder
                self.rebind(binding_sub, joint_dict) # update binding of the sub-binder too

    ##
    # @brief get exact bindings to transit
    # @param from_state State
    # @param to_state   State
    def get_slack_bindings(self, from_state, to_state):
        binding_list = []
        if to_state.binding_state is not None:
            for bd0, bd1 in zip(from_state.binding_state, to_state.binding_state):
                if bd0[2] != bd1[2]: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"

        success = len(binding_list)>0
        for binding in binding_list:
            if not self.actor_dict[binding[2]].check_available(
                    list2dict(from_state.Q, self.gscene.joint_names)):
                success = False

        return binding_list, success

    ##
    # @brief get current scene state
    def get_state(self, Q):
        ## calculate binder transformations
        Q_dict = list2dict(Q, self.gscene.joint_names)
        binder_T_dict = {}
        binder_scale_dict = {}
        for k, binder in self.actor_dict.items():
            binder_T = binder.get_tf_handle(Q_dict)
            binder_scale_dict[k] = binder.geometry.dims
            if binder.point is not None:
                binder_scale_dict[k] = 0
            binder_T_dict[k] = binder_T

        ## get current binding state
        binding_state = []
        for kobj in self.subject_name_list:
            obj = self.subject_dict[kobj]
            Tobj = obj.geometry.get_tf(Q_dict)

            min_val = 1e10
            min_point = ""
            min_binder = ""
            ## find best binding between object and binders
            for kpt, bd in self.subject_dict[kobj].action_points_dict.items():
                handle_T = bd.get_tf_handle(Q_dict)
                point_cur = handle_T[:3, 3]
                direction_cur = handle_T[:3, 2]

                for kbd, Tbd in binder_T_dict.items():
                    if kobj == kbd or kobj == self.actor_dict[kbd].geometry.name \
                            or not self.actor_dict[kbd].check_type(bd):
                        continue
                    point_diff = Tbd[:3, 3] - point_cur
                    point_diff_norm = np.linalg.norm(np.maximum(np.abs(point_diff) - binder_scale_dict[kbd], 0))
                    dif_diff_norm = np.linalg.norm(direction_cur - Tbd[:3, 2])
                    bd_val_norm = point_diff_norm  # + dif_diff_norm
                    if bd_val_norm < min_val:
                        min_val = bd_val_norm
                        min_point = bd.name
                        min_binder = kbd
            binding_state.append((kobj, min_point, min_binder, self.actor_dict[min_binder].geometry.name))
        binding_state = tuple(binding_state)

        # calculate object pose relative to binder link
        state_param = {}
        for binding in binding_state:
            obj = self.subject_dict[binding[0]]
            binder = self.actor_dict[binding[2]]
            state_param[binding[0]] = (obj.geometry.link_name, obj.geometry.get_tf(Q_dict, from_link=binder.geometry.link_name))

        return State(binding_state, state_param, Q, self)

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
        to_state.binding_state = tuple([(obj, handle, binder) if binding[0] == obj else binding for binding in to_state.binding_state])
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
    # @brief get available bindings between object and binder geometry
    # @param oname name of object with action points
    # @param bgname name of binder's parent geometry (one geometry can hold multiple binders)
    # @param ap_exclude action point name to exclude (typically the currently bound one)
    # @param bd_exclude binding point name to exclude (typically the currently bound one)
    # @param Q_dict current joint configuration in dictionary format
    # @return list of binding (point name, binder name)
    def get_available_bindings(self, oname, bgname, ap_exclude, bd_exclude, Q_dict):
        obj = self.subject_dict[oname]
        ap_dict = obj.action_points_dict
        apk_list = ap_dict.keys()
        bd_list = [self.actor_dict[bname] for bname in self.geometry_actor_dict[bgname]
                   if self.actor_dict[bname].check_available(Q_dict) and bname != bd_exclude]

        apk_exclude = obj.get_conflicting_points(ap_exclude)
        ap_list = [ap_dict[apk] for apk in apk_list if apk not in apk_exclude]

        available_bindings = []
        for bd in bd_list:
            for ap in ap_list:
                if bd.check_type(ap):
                    available_bindings.append((ap.name, bd.name, bd.geometry.name))
        if not available_bindings:
            print("=================================")
            print("=================================")
            print("=================================")
            print("Not available:{}-{}".format(oname,bgname))
            print("np_exclude:{}".format(ap_exclude))
            print("bd_exclude:{}".format(bd_exclude))
            print("=================================")
            print("=================================")
            print("=================================")
        return available_bindings

    ##
    # @brief get dictionary of available bindings for given transition
    # @param state current state
    # @param to_node target object-level node
    # @param Q_dict current joint configuration in dictionary
    # @return {object name: [(point name, binder name)]}
    def get_available_binding_dict(self, state, to_node, Q_dict=None):
        if Q_dict is None:
            Q_dict = list2dict(state.Q, self.gscene.joint_names)
        return {oname:self.get_available_bindings(oname, bgname, sbinding[1], sbinding[2],
                                                                 Q_dict=Q_dict)\
                                              if sbgname!=bgname else [sbinding[1:]]
                                      for oname, bgname, sbgname, sbinding
                                      in zip(self.subject_name_list, to_node, state.node, state.binding_state)}

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
        to_binding_state = tuple([(((oname,)+\
                           binding_sampler(available_binding_dict[oname]))
                          if sbgname!=bgname else sbinding)
                         for oname, bgname, sbgname, sbinding
                         in zip(self.subject_name_list, to_node, state.node, state.binding_state)])
        to_state.set_binding_state(to_binding_state, self)
        redundancy_dict = {}
        for from_binding, to_binding in zip(state.binding_state, to_binding_state):
            obj = self.subject_dict[from_binding[0]]
            to_ap = obj.action_points_dict[to_binding[1]]
            to_binder = self.actor_dict[to_binding[2]]
            redundancy_tot = combine_redundancy(to_ap, to_binder)
            redundancy = sample_redundancy(redundancy_tot, sampler=redundancy_sampler)
            redundancy_dict[from_binding[0]] = redundancy
        return to_state, redundancy_dict

    ##
    # @brief    add axis marker to handle
    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.geometry
        Toff_lh = handle.Toff_lh
        axis = "xyz"
        self.gscene.add_highlight_axis(hl_key, hobj.name, hobj.link_name, Toff_lh[:3,3], Toff_lh[:3,:3], color=color, axis=axis)