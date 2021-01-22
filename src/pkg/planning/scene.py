from collections import defaultdict
from ..utils.utils import list2dict
import numpy as np
from ..controller.combined_robot import RobotSpecs


##
# @class    State
# @brief    planning scene state
class State:
    ##
    # @param node           tuple of binding state ((object name, binding point, binder), ..)
    # @param obj_pos_dict   object pose dictionary {object name: 4x4 offset relative to attached link}
    # @param Q              robot joint configuration
    # @param pscene         PlanningScene instance
    def __init__(self, node, obj_pos_dict, Q, pscene):
        self.obj_pos_dict = obj_pos_dict
        self.Q = Q
        self.set_node(node, pscene)

    def set_node(self, node, pscene):
        ## @brief tuple of binding state ((object name, binding point, binder), ..)
        self.node = node
        ## @brief tuple of simplified binding state (binder name 1, binder name 2, ..)
        self.onode = node2onode(pscene, self.node)

    def get_tuple(self):
        return (self.node, self.obj_pos_dict, self.Q)

    def copy(self, pscene):
        return State(self.node, self.obj_pos_dict, self.Q, pscene)

    def __str__(self):
        return str((self.node,
                    {k: str(np.round(v, 2)) for k, v in
                     self.obj_pos_dict.items()} if self.obj_pos_dict is not None else None,
                    str(np.round(self.Q, 2)) if self.Q is not None else None))

##
# @brief convert node to object node
def node2onode(pscene, node):
    return tuple([pscene.binder_dict[binding[2]].geometry.name for binding in node])


##
# @class    PlanningScene
# @brief    planning scene
class PlanningScene:
    ##
    # @param gscene instance of rnb-planning.src.pkg.geometry.geometry.GeometryScene
    # @param combined_robot instance of rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
    def __init__(self, gscene, combined_robot):
        ##
        # @brief rnb-planning.src.pkg.geometry.geometry.GeometryScene
        self.gscene = gscene
        ##
        # @brief rnb-planning.src.pkg.controller.combined_robot.CombinedRobot
        self.combined_robot = combined_robot
        self.binder_dict = {}
        self.object_binder_dict = defaultdict(list)
        self.handle_dict = {}
        self.handle_list = []
        self.object_dict = {}

    ##
    # @brief add a binder to the scene
    def add_binder(self, binder):
        self.binder_dict[binder.name] = binder
        self.object_binder_dict[binder.geometry.name].append(binder.name)

    ##
    # @brief remove a binder from the scene
    def remove_binder(self, bname):
        if bname in self.binder_dict:
            self.object_binder_dict[self.binder_dict[bname].geometry.name].remove(bname)
            del self.binder_dict[bname]

    ##
    # @param bname binder name
    # @param gname name of parent object
    # @param _type type of binder, subclass of rnb-planning.src.pkg.planning.constraint.constraint_action.Binding
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
        for k_b, binder in self.binder_dict.items():
            if not binder.multiple:
                uniq_binders += [k_b]
        return uniq_binders


    ##
    # @brief get controlled binders in dictionary
    def get_controlled_binders(self):
        controlled_binders = []
        for k_b, binder in self.binder_dict.items():
            if binder.controlled:
                controlled_binders += [k_b]
        return controlled_binders

    ##
    # @brief add a object to the scene
    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              list2dict([0] * len(self.gscene.joint_names),
                                                        item_names=self.gscene.joint_names))
        self.update_handles()

    ##
    # @brief remove a object from the scene
    def remove_object(self, name):
        if name in self.object_dict:
            del self.object_dict[name]
        self.update_handles()

    ##
    # @param oname object name
    # @param gname name of parent object
    # @param _type type of object, subclass of rnb-planning.src.pkg.planning.constraint.constraint_object.ObjectAction
    # @param binding point offset from object (m)
    def create_object(self, oname, gname, _type, binding=None, **kwargs):
        self.remove_object(oname)
        geometry = self.gscene.NAME_DICT[gname]
        _object = _type(geometry, **kwargs)
        self.add_object(oname, _object, binding)
        return _object

    ##
    # @brief set object states
    # @param state State
    def set_object_state(self, state):
        bd_list = list(state.node)
        bd_list_done = []
        while bd_list:
            bd = bd_list.pop(0)
            binder = self.binder_dict[bd[2]]
            if binder.geometry in [self.object_dict[bd_tmp[0]].geometry for bd_tmp in bd_list]:
                bd_list += [bd] # prevent using previous info ( move back to end )
            else:
                obj = self.object_dict[bd[0]]
                frame = state.obj_pos_dict[bd[0]]
                binder.link_name = binder.geometry.link_name # sync linke name with parent
                obj.set_state(frame, binder.link_name, bd[1], bd[2])
                bd_list_done += [bd]

    ##
    # @brief get object states
    # @return node tuple of binding state ((object name, binding point, binder), ..)
    # @return pose_dict object pose relative to attached links {name: 4x4 transformation}
    def get_object_state(self):
        node = ()
        pose_dict = {}
        for k in self.object_list:
            v = self.object_dict[k]
            node += ((k,) + v.binding,)
            pose_dict[k] = v.geometry.Toff
        return node, pose_dict

    ##
    # @brief get all handles in the scene
    # @return handles all handle items in the scene
    def get_all_handles(self):
        handles = []
        for obj_hd in self.object_dict.values():
            handles += obj_hd.action_points_dict.values()
        return handles

    ##
    # @brief get all handles in the scene as a dictionary
    # @return handle_dict all handle items in the scene as a dictionary
    def get_all_handle_dict(self):
        handle_dict = {}
        for obj_hd in self.object_dict.values():
            for hd in obj_hd.action_points_dict.values():
                handle_dict[hd.name_full] = hd
        return handle_dict

    ##
    # @brief delete specific handle by handle name
    def delete_handle(self, htem):
        otem = self.object_dict[htem.geometry.name]
        del otem.action_points_dict[htem.name]
        if not otem.action_points_dict.keys():
            self.remove_object(htem.geometry.name)

    ##
    # @brief    collect all objects' handle items and update to the scene,
    #           automatically called when object is added/removed.
    def update_handles(self):
        self.handle_dict = {}
        self.handle_list = []
        self.object_list = sorted(self.object_dict.keys())
        for k in self.object_list:
            v = self.object_dict[k]
            ap_list = v.action_points_dict
            self.handle_dict[k] = []
            for ap in ap_list.keys():
                self.handle_dict[k] += [ap]
                self.handle_list += [(k, ap)]

    ##
    # @brief change binding state
    # @param binding    binding tuple (object name, binding point, binder)
    # @param joint_dict joint pose in radian as dictionary
    def rebind(self, binding, joint_dict):
        binder = self.binder_dict[binding[2]]
        object_tar = self.object_dict[binding[0]]
        binder.bind(action_obj=object_tar, bind_point=binding[1], joint_dict_last=joint_dict)
        for binder_sub in [k for k,v in self.binder_dict.items() if v.geometry == object_tar.geometry]:
            for binding_sub in [(k,v.binding[0]) for k, v in self.object_dict.items()
                                if v.binding[1] == binder_sub]:
                binding_sub += (binder_sub,)
                self.rebind(binding_sub, joint_dict)

    ##
    # @brief get binding to transit
    # @param from_state State
    # @param to_state   State
    def get_slack_bindings(self, from_state, to_state):
        binding_list = []
        if to_state.node is not None:
            for bd0, bd1 in zip(from_state.node, to_state.node):
                if bd0[2] != bd1[2]: # check if new transition (slack)
                    binding_list += [bd1]
                else:
                    assert bd0[1] == bd1[1] , "impossible transition"

        success = len(binding_list)>0
        for binding in binding_list:
            if not self.binder_dict[binding[2]].check_available(
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
        for k, binder in self.binder_dict.items():
            binder_T = binder.get_tf_handle(Q_dict)
            binder_scale_dict[k] = binder.geometry.dims
            if binder.point is not None:
                binder_scale_dict[k] = 0
            binder_T_dict[k] = binder_T

        ## get current binding node
        node = []
        for kobj in self.object_list:
            obj = self.object_dict[kobj]
            Tobj = obj.geometry.get_tf(Q_dict)

            min_val = 1e10
            min_point = ""
            min_binder = ""
            ## find best binding between object and binders
            for kpt, bd in self.object_dict[kobj].action_points_dict.items():
                handle_T = bd.get_tf_handle(Q_dict)
                point_cur = handle_T[:3, 3]
                direction_cur = handle_T[:3, 2]

                for kbd, Tbd in binder_T_dict.items():
                    if kobj == kbd or kobj == self.binder_dict[kbd].geometry.name \
                            or not self.binder_dict[kbd].check_type(bd):
                        continue
                    point_diff = Tbd[:3, 3] - point_cur
                    point_diff_norm = np.linalg.norm(np.maximum(np.abs(point_diff) - binder_scale_dict[kbd], 0))
                    dif_diff_norm = np.linalg.norm(direction_cur - Tbd[:3, 2])
                    bd_val_norm = point_diff_norm  # + dif_diff_norm
                    if bd_val_norm < min_val:
                        min_val = bd_val_norm
                        min_point = bd.name
                        min_binder = kbd
            node.append((kobj, min_point, min_binder))
        node = tuple(node)

        # calculate object pose relative to binder link
        obj_pos_dict = {}
        for binding in node:
            obj = self.object_dict[binding[0]]
            binder = self.binder_dict[binding[2]]
            obj_pos_dict[binding[0]] = obj.geometry.get_tf(Q_dict, from_link=binder.geometry.link_name)

        return State(node, obj_pos_dict, Q, self)

    ##
    # @brief get goal nodes that link object to target binder
    # @param initial_node   initial node
    # @param obj            object name
    # @param binder         target binder name
    def get_goal_nodes(self, initial_node, obj, binder):
        _object = self.object_dict[obj]
        _binder = self.binder_dict[binder]
        bindings = []
        for k, v in _object.action_points_dict.items():
            if _binder.check_type(v):
                bindings.append((obj, k, binder))

        return tuple(tuple(binding if bd[0] == binding[0] \
                               else bd for bd in initial_node) \
                     for binding in bindings)

    ##
    # @brief make goal state that with specific binding target
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param obj        object name
    # @param handle     handle name
    # @param binder     binder name
    def make_goal_state(self, from_state, obj, handle, binder):
        to_state = from_state.copy(self)
        to_state.node = tuple([(obj, handle, binder) if binding[0] == obj else binding for binding in to_state.node])
        return to_state

    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.geometry
        Toff_lh = handle.Toff_lh
        axis = "xyz"
        self.gscene.add_highlight_axis(hl_key, hobj.name, hobj.link_name, Toff_lh[:3,3], Toff_lh[:3,:3], color=color, axis=axis)

    def add_aruco_axis(self, hl_key, atem, axis_name=None):
        oname = atem.oname
        axis_name = axis_name or oname
        if oname in self.combined_robot.get_robot_config_dict():
            link_name = RobotSpecs.get_base_name(self.combined_robot.get_robot_config_dict()[oname].type, oname)
            Toff = atem.Toff
        else:
            aobj = self.gscene.NAME_DICT[oname]
            link_name = aobj.link_name
            Toff = np.matmul(aobj.Toff, atem.Toff)
        self.gscene.add_highlight_axis(hl_key, axis_name, link_name, Toff[:3,3], Toff[:3,:3], axis="xyz")