from collections import defaultdict
from ..utils.utils import list2dict

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