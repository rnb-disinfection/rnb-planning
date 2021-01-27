from __future__ import print_function

from .constraint_common import *
from ...constants import DIR_RPY_DICT, DIR_VEC_DICT

from abc import *
__metaclass__ = type


##
# @class DirectedPoint
# @brief ActionPoint with z-axis constraint
class DirectedPoint(ActionPoint):
    def get_redundancy(self):
        return {"w":(-np.pi,np.pi)}


##
# @class FramedPoint
# @brief ActionPoint with full orientation constraint
class FramedPoint(ActionPoint):
    def get_redundancy(self):
        return {}


################################# USABLE CLASS #########################################


##
# @class PlacePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_action.PlacePlane
class PlacePoint(DirectedPoint):
    ctype=ConstraintType.Place


##
# @class FramePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_action.FramedTool
class FramePoint(FramedPoint):
    ctype=ConstraintType.Frame


##
# @class VacuumPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_action.VacuumTool
class VacuumPoint(DirectedPoint):
    ctype=ConstraintType.Vacuum


##
# @class Grasp2Point
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_action.Gripper2Tool
class Grasp2Point(DirectedPoint):
    ctype=ConstraintType.Grasp2


##
# @class FixturePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_action.FixtureSlot
class FixturePoint(FramedPoint):
    ctype=ConstraintType.Fixture


##
# @class Action
# @brief Base class for action definitions
class Action:

    ##
    # @brief (prototype) set state
    # @param node_item state node item
    # @param state_param state parameter item
    @abstractmethod
    def set_state(self, node_item, state_param):
        pass

    ##
    # @brief (prototype) get node item
    # @return item for node tuple
    @abstractmethod
    def get_node_item(self):
        pass

    ##
    # @brief (prototype) get state param
    # @return item for state_param
    @abstractmethod
    def get_state_param(self):
        pass


##
# @class TaskAction
# @brief Base class for task definition
class TaskAction(Action):
    def __init__(self):
        raise(NotImplementedError("TaskAction is abstract class"))


##
# @class TaskAction
# @brief Base class for task definition
class SwipTask(Action):
    def __init__(self):
        raise(NotImplementedError("TaskAction is abstract class"))

    ##
    # @brief set state of task action
    # @param task_node state node item describing the task
    # @param state_param task state parameter
    def set_state(self, task_node, state_param):
        self.task_node = task_node
        self.state_param = state_param

    ##
    # @brief (prototype) get node item
    # @return item for node tuple
    def get_node_item(self):
        pass

    ##
    # @brief (prototype) get state param
    # @return item for state_param
    def get_state_param(self):
        pass


##
# @class ObjectAction
# @brief Base class for objects with defined action points (handles)
# @remark get_conflicting_handles and register_binders should be implemented with child classes
class ObjectAction(Action):
    def __init__(self):
        raise(NotImplementedError("ObjectAction is abstract class"))

    ##
    # @brief set object binding state and update location
    # @param binding (handle name, binder name)
    # @param link_frame (link name, offset transformation in 4x4 matrix)
    def set_state(self, binding, link_frame):
        link_name = link_frame[0]
        frame = link_frame[1]
        self.geometry.set_offset_tf(frame[:3, 3], frame[:3,:3])
        self.geometry.set_link(link_name)
        self.binding = binding
        for ap in self.action_points_dict.values():
            ap.update_handle()
        for bp in self.binder_points_dict.values():
            bp.update_handle()

    ##
    # @brief get binding (point_name, binder_name)
    # @return item for node tuple
    def get_node_item(self):
        return self.binding

    ##
    # @brief get state param (link_name, Toff)
    # @return item for state_param
    def get_state_param(self):
        return (self.geometry.link_name, self.geometry.Toff)

    ##
    # @brief function prototype to get conflicting handles to build efficient search tree
    # @param handle name
    @abstractmethod
    def get_conflicting_handles(self, hname):
        pass

    ##
    # @brief function prototype to register pre-defined binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_action.Binding
    @abstractmethod
    def register_binders(self, planning_scene, _type):
        pass

################################# USABLE CLASS #########################################


##
# @class CustomObject
# @brief Customizable object that handles can be defined by user
class CustomObject(ObjectAction):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    # @param binder_points_dict pre-defined binder points as dictionary
    def __init__(self, oname, geometry, action_points_dict, binder_points_dict=None):
        self.oname = oname
        if binder_points_dict is None:
            binder_points_dict = {}
        self.geometry = geometry
        self.action_points_dict = action_points_dict
        self.binder_points_dict = binder_points_dict

    ##
    # @brief only self conflict is expected
    # @param handle name
    def get_conflicting_handles(self, hname):
        return [hname]

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class SingleHandleObject
# @brief Object with single defined handle
class SingleHandleObject(ObjectAction):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_point pre-defined single action point
    # @param binder_points_dict pre-defined binder points as dictionary
    def __init__(self, oname, geometry, action_point, binder_points_dict=None):
        self.oname = oname
        if binder_points_dict is None:
            binder_points_dict = {}
        self.geometry = geometry
        self.action_points_dict = {action_point.name: action_point}
        self.binder_points_dict = binder_points_dict

    ##
    # @brief only self conflict is expected
    # @param handle name
    def get_conflicting_handles(self, hname):
        return [hname]

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class BoxObject
# @brief Box object with hexahedral action points
class BoxObject(ObjectAction):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param hexahedral If True, all hexahedral points are defined. Otherwise, only top and bottom points are defined
    # @param binder_points_dict pre-defined binder points as dictionary
    def __init__(self, oname, geometry, hexahedral=True, binder_points_dict=None):
        self.oname = oname
        if binder_points_dict is None:
            binder_points_dict = {}
        self.geometry = geometry
        Xhalf, Yhalf, Zhalf = np.divide(geometry.dims,2)
        self.binder_points_dict = binder_points_dict
        self.action_points_dict = {
            "top_p": PlacePoint("top_p", geometry, [0,0,Zhalf], [np.pi,0,0]),
            "bottom_p": PlacePoint("bottom_p", geometry, [0,0,-Zhalf], [0,0,0]),
            # "top_v": VacuumPoint("top_v", geometry, [0,0,Zhalf], [0,0,-1]),
            # "bottom_v": VacuumPoint("bottom_v", geometry, [0,0,-Zhalf], [0,0,1]),
            "top_g": Grasp2Point("top_g", geometry, [0,0,0], [np.pi,0,0]),
            "bottom_g": Grasp2Point("bottom_g", geometry, [0,0,0], [0,0,0]),
            "top_f": FramePoint("top_f", geometry, [0,0,Zhalf], [np.pi,0,0]),
            "bottom_f": FramePoint("bottom_f", geometry, [0,0,-Zhalf], [0,0,0])
        }
        self.hexahedral = hexahedral
        if hexahedral:
            self.action_points_dict.update({
                "right_p": PlacePoint("right_p", geometry, [Xhalf,0,0], [0,-np.pi/2,0]),
                "left_p": PlacePoint("left_p", geometry, [-Xhalf,0,0], [0,np.pi/2,0]),
                "front_p": PlacePoint("front_p", geometry, [0,-Yhalf,0], [-np.pi/2,0,0]),
                "back_p": PlacePoint("back_p", geometry, [0,Yhalf,0], [np.pi/2,0,0]),
                # "right_v": VacuumPoint("right_v", geometry, [Xhalf,0,0], [-1,0,0]),
                # "left_v": VacuumPoint("left_v", geometry, [-Xhalf,0,0], [1,0,0]),
                # "front_v": VacuumPoint("front_v", geometry, [0,-Yhalf,0], [0,1,0]),
                # "back_v": VacuumPoint("back_v", geometry, [0,Yhalf,0], [0,-1,0]),
                "right_g": Grasp2Point("right_g", geometry, [0,0,0], [0,-np.pi/2,0]),
                "left_g": Grasp2Point("left_g", geometry, [0,0,0], [0,np.pi/2,0]),
                "front_g": Grasp2Point("front_g", geometry, [0,0,0], [-np.pi/2,0,0]),
                "back_g": Grasp2Point("back_g", geometry, [0,0,0], [np.pi/2,0,0]),
                "right_f": FramePoint("right_f", geometry, [Xhalf,0,0], [0,-np.pi/2,0]),
                "left_f": FramePoint("left_f", geometry, [-Xhalf,0,0], [0,np.pi/2,0]),
                "front_f": FramePoint("front_f", geometry, [0,-Yhalf,0], [-np.pi/2,0,0]),
                "back_f": FramePoint("back_f", geometry, [0,Yhalf,0], [np.pi/2,0,0])
            })
            self.conflict_dict = {
                hname: [hname[:-1]+postfix for postfix in "pgf"] +
                       ([OPPOSITE_DICT[hname[:-2]]+"_"+postfix for postfix in "pgf"] \
                            if hname[-1] == "g" else [OPPOSITE_DICT[hname[:-2]]+"_g"])
                for hname in self.action_points_dict.keys()
            }

    ##
    # @brief get conflicting handles in hexahedral geometry
    # @param handle name
    def get_conflicting_handles(self, hname):
        return self.conflict_dict[hname]

    ##
    # @brief register hexahedral binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_action.Binding
    def register_binders(self, planning_scene, _type):
        gname = self.geometry.name
        dims = self.geometry.dims
        for k in DIR_RPY_DICT.keys():
            if not self.hexahedral and k not in ["top", "bottome"]:
                continue
            rpy = DIR_RPY_DICT[k]
            point = tuple(-np.multiply(DIR_VEC_DICT[k], dims)/2)
            planning_scene.create_binder(bname="{}_{}".format(gname, k), gname=gname, _type=_type,
                                         point=point, rpy=rpy)


##
# @brief convert constraint type to handle type
def ctype_to_htype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Grasp2Point
    elif cstr == ConstraintType.Frame.name:
        return FramePoint
    elif cstr == ConstraintType.Place.name:
        return PlacePoint
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumPoint


##
# @brief convert object type string to object type
def otype_to_class(ostr):
    if ostr == 'BoxObject':
        return BoxObject
    if ostr == 'SingleHandleObject':
        return SingleHandleObject
    if ostr == 'CustomObject':
        return CustomObject
