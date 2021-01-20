from collections import defaultdict
from ..utils.utils import list2dict


##
# @class    State
# @brief    planning scene state
class State:
    ##
    # @param node           tuple of binding state ((object name, binding point, binder), ..)
    # @param obj_pos_dict   object pose dictionary {object name: 4x4 offset relative to attached link}
    # @param Q              robot joint configuration
    # @param scene          PlanningScene instance
    def __init__(self, node, obj_pos_dict, Q, scene):
        self.obj_pos_dict = obj_pos_dict
        self.Q = Q
        self.set_node(node, scene)

    def set_node(self, node, scene):
        ## @brief tuple of binding state ((object name, binding point, binder), ..)
        self.node = node
        ## @brief tuple of simplified binding state (binder name 1, binder name 2, ..)
        self.onode = node2onode(scene, self.node)

    def get_tuple(self):
        return (self.node, self.obj_pos_dict, self.Q)

    def copy(self, scene):
        return State(self.node, self.obj_pos_dict, self.Q, scene)

    def __str__(self):
        return str((self.node,
                    {k: str(np.round(v, 2)) for k, v in
                     self.obj_pos_dict.items()} if self.obj_pos_dict is not None else None,
                    str(np.round(self.Q, 2)) if self.Q is not None else None))

##
# @brief convert node to object node
def node2onode(graph, node):
    return tuple([graph.binder_dict[binding[2]].geometry.name for binding in node])


##
# @class    PlanningScene
# @brief    planning scene
class PlanningScene:
    ##
    # @param ghnd instance of rnb-planning.src.pkg.geometry.geometry.GeometryHandle
    def __init__(self, ghnd):
        self.ghnd = ghnd
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
        geometry = self.ghnd.NAME_DICT[gname]
        binder = _type(bname, geometry=geometry, point=point, rpy=rpy)
        self.add_binder(binder)
        return binder

    ##
    # @brief add a object to the scene
    def add_object(self, name, _object, binding=None):
        self.object_dict[name] = _object
        if binding is not None:
            self.binder_dict[binding[1]].bind(self.object_dict[name], binding[0],
                                              list2dict([0] * len(self.joint_names), item_names=self.joint_names))

    ##
    # @brief remove a object from the scene
    def remove_object(self, name):
        if name in self.object_dict:
            del self.object_dict[name]

    ##
    # @param oname object name
    # @param gname name of parent object
    # @param _type type of object, subclass of rnb-planning.src.pkg.planning.constraint.constraint_object.ObjectAction
    # @param binding point offset from object (m)
    def create_object(self, oname, gname, _type, binding=None, **kwargs):
        self.remove_object(oname)
        geometry = self.ghnd.NAME_DICT[gname]
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
    # @brief collect all objects' handle items and update to the scene
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