from __future__ import print_function

from .constraint_common import *


##
# @class Actor
# @brief Base class for binder
# @remark get_redundancy and check_available should be implemented with child classes
class Actor(ActionPoint):
    active = None
    multiple = None

    def bind(self, action_obj, bind_point, joint_dict_last):
        Tbo = action_obj.geometry.get_tf(joint_dict_last)
        Tbal = get_tf(self.geometry.link_name, joint_dict_last, self.gscene.urdf_content)
        Tlao = np.matmul(np.linalg.inv(Tbal), Tbo)
        binding = BindingTransform(action_obj, action_obj.action_points_dict[bind_point], self, T_lao=Tlao)
        action_obj.set_state(binding, None)

    def check_pair(self, action_point):
        return action_point.ctype == self.ctype and action_point.key == self.key

    ##
    # @brief function prototype to check quick availability of action point when building search tree
    @abstractmethod
    def check_available(self, joint_dict):
        pass


##
# @class PointerActor
# @brief Base class for Pointer type binder. z-direction and contact are constrained.
# @remark not usable at this level
class PointerActor(Actor):

    ##
    # @brief currently support only x-y plane
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"w":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2),
                               "w":(-np.pi,np.pi)}

    ##
    # @brief place plane is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.geometry.is_controlled()


##
# @class FrameActor
# @brief Base class for Frame type binder. Full orientation and contact are constrained.
# @remark not usable at this level
class FrameActor(Actor):
    ##
    # @brief currently support only x-y plane
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2)}

    ##
    # @brief place plane is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.geometry.is_controlled()

################################# USABLE CLASS #########################################


##
# @class VacuumTool
# @brief Actor class for vacuum type tool. z-direction is constrained. (PointerActor)
class VacuumTool(PointerActor):
    active = True
    multiple = False
    ctype = ConstraintType.Vacuum


##
# @class Gripper2Tool
# @brief Actor class for 2-finger gripper type tool. z-direction is constrained. (PointerActor)
class Gripper2Tool(PointerActor):
    active = True
    multiple = False
    ctype = ConstraintType.Grasp2

    ##
    # @brief currently support only x-y plane
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"w":(-np.pi/4,np.pi/4)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2),
                               "w":(-np.pi/4,np.pi/4)}


##
# @class FramedTool
# @brief Actor class for fully constrained tool. (FrameActor)
class FramedTool(FrameActor):
    active = True
    multiple = False
    ctype = ConstraintType.Frame

    def __init__(self, *args, **kwargs):
        self.available = True
        FrameActor.__init__(self, *args, **kwargs)

    ##
    # @brief place frame is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.available


##
# @class PlacePlane
# @brief Actor class for placing plane. z-direction constrained. (PointerActor)
class PlacePlane(PointerActor):
    active = False
    multiple = True
    ctype = ConstraintType.Place
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    ##
    # @brief place plane is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT



##
# @class PlaceFrame
# @brief Actor class for placing frame. Fully constrained. (FrameActor)
class PlaceFrame(FrameActor):
    active = False
    multiple = True
    ctype = ConstraintType.Frame
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    ##
    # @brief place frame is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT

##
# @class AttachFramer
# @brief Actor class for placing frame. Fully constrained. (FrameActor)
class AttachFramer(FrameActor):
    active = False
    multiple = True
    ctype = ConstraintType.Frame
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    def __init__(self, *args, **kwargs):
        self.available = True
        FrameActor.__init__(self, *args, **kwargs)

    ##
    # @brief place frame is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.available


##
# @class FixtureSlot
# @brief Fixture slot to implement assembly object with bindings.
class FixtureSlot(PointerActor):
    active = False
    multiple = True
    ctype = ConstraintType.Fixture

    def check_available(self, joint_dict):
        return False


##
# @class SweepTool
# @brief Actor class for sweeping. z-direction constrained. (PointerActor)
class SweepTool(PointerActor):
    active = True
    multiple = False
    ctype = ConstraintType.Sweep
    VERTICAL_CUT = np.cos(np.deg2rad(10))


##
# @class SweepFramer
# @brief Actor class for sweeping. z-direction constrained. (FrameActor)
class SweepFramer(FrameActor):
    active = True
    multiple = False
    ctype = ConstraintType.Sweep
    VERTICAL_CUT = np.cos(np.deg2rad(10))

##
# @class KnobFramer
# @brief Actor class for sweeping. z-direction constrained. (FrameActor)
class KnobFramer(FrameActor):
    active = True
    multiple = False
    ctype = ConstraintType.Knob

##
# @class HingeFramer
# @brief Actor class for sweeping. z-direction constrained. (FrameActor)
class HingeFramer(FrameActor):
    active = True
    multiple = False
    ctype = ConstraintType.Hinge


##
# @class WayAgent
# @brief Actor class for waypoint reaching. z-direction constrained. (PointerActor)
class WayAgent(PointerActor):
    active = True
    multiple = False
    ctype = ConstraintType.Waypoint
    VERTICAL_CUT = np.cos(np.deg2rad(10))


##
# @class WayFramer
# @brief Actor class for waypoint reaching. z-direction constrained. (PointerActor)
class WayFramer(FrameActor):
    active = True
    multiple = False
    ctype = ConstraintType.Waypoint
    VERTICAL_CUT = np.cos(np.deg2rad(10))


##
# @brief convert constraint type to Constraint type
def ctype_to_btype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Gripper2Tool
    elif cstr == ConstraintType.Frame.name:
        return PlaceFrame
    elif cstr == ConstraintType.Place.name:
        return PlacePlane
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumTool
