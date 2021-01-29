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
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.PlacePlane
class PlacePoint(DirectedPoint):
    ctype=ConstraintType.Place


##
# @class FramePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.FramedTool
class FramePoint(FramedPoint):
    ctype=ConstraintType.Frame


##
# @class VacuumPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.VacuumTool
class VacuumPoint(DirectedPoint):
    ctype=ConstraintType.Vacuum


##
# @class Grasp2Point
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.Gripper2Tool
class Grasp2Point(DirectedPoint):
    ctype=ConstraintType.Grasp2


##
# @class SweepPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.SweepTool
class SweepPoint(DirectedPoint):
    ctype=ConstraintType.Sweep


##
# @class FixturePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.FixtureSlot
class FixturePoint(FramedPoint):
    ctype=ConstraintType.Fixture


##
# @class    SubjectType
class SubjectType(Enum):
    OBJECT = 0
    TASK = 1


##
# @class    Subject
# @brief    Base class for state subject definitions
# @remark   Subject consist of parent geometry and action points.
#           action_points are reference points for action, which are bound to actors.
#           binding represents current binding state with an actor.
class Subject:
    ## @brief SubjectType
    stype = None
    def __init__(self):
        ## @brief name of object
        self.oname = None
        ## @brief geometry instance for the object (rnb-planning.src.pkg.geometry.geometry.GeometryItem)
        self.geometry = None
        ## @brief dictionary of action points {point name: rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint}
        self.action_points_dict = {}
        ## @brief object's binding state tuple (object name, point, actor, actor-geometry)
        self.binding = (None, None, None, None)
        ## @brief object's binding state tuple (object name, point, actor, actor-geometry)
        self.constrain = (None, None, None, None)
        raise(NotImplementedError("ObjectBinding is abstract class"))

    ##
    # @brief make constraints. by default, empty list.
    def make_constraints(self, tol=1e-3):
        return []

    ##
    # @brief get conflicting handles to build efficient search tree
    # @param handle name
    def get_conflicting_points(self, hname):
        return [hname]

    ##
    # @brief (prototype) set state param
    # @param binding (handle name, binder name)
    # @param state_param
    @abstractmethod
    def set_state(self, binding, state_param):
        pass

    ##
    # @brief (prototype) get state param
    # @return item for state_param
    @abstractmethod
    def get_state_param(self):
        pass

    ##
    # @brief get object-level state_param component
    @abstractmethod
    def get_state_param_update(self, binding, state_param):
        pass


    ##
    # @brief (prototype) get object-level node component
    @classmethod
    @abstractmethod
    def get_node_component(cls, binding_state, state_param):
        pass

    ##
    # @brief (prototype) get object-level node component
    @abstractmethod
    def get_neighbor_node_component_list(self, node, pscene):
        pass

    ##
    # @brief (prototype) get all object-level node component
    @classmethod
    def get_all_node_components(cls, pscene):
        pass


##
# @class TaskInterface
# @brief Base class for task definition
class TaskInterface(Subject):
    stype = SubjectType.TASK
    def __init__(self):
        raise(NotImplementedError("TaskAction is abstract class"))


##
# @class SweepTask
# @brief sweep action points in alphabetical order
class SweepTask(TaskInterface):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    def __init__(self, oname, geometry, action_points_dict):
        self.oname = oname
        self.geometry = geometry
        self.action_points_dict = action_points_dict
        self.action_points_order = sorted(self.action_points_dict.keys())
        self.state_param = np.zeros(len(self.action_points_order), dtype=np.bool)

    ##
    # @brief make sweep constraints (surface + normal)
    def make_constraints(self, tol=1e-3):
        return [MotionConstraint([self.geometry], True, True, tol)]

    ##
    # @brief set object binding state and update location
    # @param binding (handle name, binder name)
    # @param state_param list of done-mask
    def set_state(self, binding, state_param=None):
        self.binding = binding
        if state_param is not None:
            self.state_param = state_param.copy()

    ##
    # @brief (prototype) get state param
    # @return item for state_param
    def get_state_param(self):
        return self.state_param.copy()

    ##
    # @brief get object-level state_param component
    def get_state_param_update(self, binding, state_param):
        if binding[1] is not None:
            state_param[self.action_points_order.index(binding[1])] = True
        return state_param

    ##
    # @brief get object-level node component (finished waypoint count)
    @classmethod
    def get_node_component(cls, binding_state, state_param):
        if binding_state[1] is not None:
            return int(np.sum(state_param))
        else:
            return 0

    ##
    # @brief get object-level neighbor component (detach or next waypoint)
    def get_neighbor_node_component_list(self, node_tem, pscene):
        if node_tem < len(self.state_param):
            return [node_tem, node_tem+1]
        else:
            return [node_tem]

    ##
    # @brief get all object-level node component
    def get_all_node_components(self, pscene):
        return list(range(len(self.state_param)+1))


##
# @class ObjectBinding
# @brief Base class for objects with defined action points (handles)
# @remark get_conflicting_points and register_binders should be implemented with child classes
class ObjectBinding(Subject):
    stype = SubjectType.OBJECT
    ##
    # @brief set object binding state and update location
    # @param binding (object name, handle name, binder name, binder geometry name)
    # @param state_param (link name, offset transformation in 4x4 matrix)
    def set_state(self, binding, state_param):
        link_name = state_param[0]
        frame = state_param[1]
        self.geometry.set_offset_tf(frame[:3, 3], frame[:3,:3])
        self.geometry.set_link(link_name)
        for ap in self.action_points_dict.values():
            ap.update_handle()
        assert binding[0] == self.oname, "wrong binding given {} <- {}".format(self.oname, binding[0])
        self.binding = binding

    ##
    # @brief get state param (link_name, Toff)
    # @return item for state_param
    def get_state_param(self):
        return self.geometry.link_name, self.geometry.Toff

    ##
    # @brief function prototype to register pre-defined binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    @abstractmethod
    def register_binders(self, planning_scene, _type):
        pass

    ##
    # @brief get object-level state_param update
    def get_state_param_update(self, binding, state_param):
        return state_param

    ##
    # @brief get object-level node component (binder geometry name)
    @classmethod
    def get_node_component(cls, binding, state_param):
        return binding[3]

    ##
    # @brief    get object-level neighbor component (other available binder geometry name)
    #           other binding point in the scene
    # @param    node_tem    geometry name of binder currently attached
    def get_neighbor_node_component_list(self, node_tem, pscene):
        ctrl_binders, uctrl_binders = pscene.divide_binders_by_control([ap.ctype for ap in self.action_points_dict.values()])
        next_node_component_list = [pscene.actor_dict[bname].geometry.name for bname in ctrl_binders]
        if pscene.geometry_actor_dict[node_tem][0] in ctrl_binders: # if any of currently attached binder geometry's binder is controlled, it's controlled
            next_node_component_list += [pscene.actor_dict[bname].geometry.name for bname in uctrl_binders] # thus we can add move it to uncontrolled binders
        if node_tem in next_node_component_list:
            next_node_component_list.remove(node_tem)
        return next_node_component_list

    ##
    # @brief get all object-level node component
    def get_all_node_components(self, pscene):
        return [v.geometry.name for k,v in pscene.actor_dict.items() if any(v.check_type(ap) for ap in self.action_points_dict.values())]


################################# USABLE CLASS #########################################


##
# @class CustomObject
# @brief Customizable object that handles can be defined by user
class CustomObject(ObjectBinding):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    def __init__(self, oname, geometry, action_points_dict):
        self.oname = oname
        self.geometry = geometry
        self.action_points_dict = action_points_dict

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class SingleHandleObject
# @brief Object with single defined handle
class SingleHandleObject(ObjectBinding):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_point pre-defined single action point
    def __init__(self, oname, geometry, action_point):
        self.oname = oname
        self.geometry = geometry
        self.action_points_dict = {action_point.name: action_point}

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class BoxObject
# @brief Box object with hexahedral action points
class BoxObject(ObjectBinding):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param hexahedral If True, all hexahedral points are defined. Otherwise, only top and bottom points are defined
    def __init__(self, oname, geometry, hexahedral=True):
        self.oname = oname
        self.geometry = geometry
        Xhalf, Yhalf, Zhalf = np.divide(geometry.dims,2)
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
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
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
