from __future__ import print_function

from .constraint_common import *


##
# @class Binding
# @brief Base class for binder
# @remark get_redundancy and check_available should be implemented with child classes
class Binding(ActionPoint):
    controlled = None
    multiple = None
        
    def bind(self, action_obj, bind_point, joint_dict_last):
        Tbo = action_obj.geometry.get_tf(joint_dict_last)
        Tbt = get_tf(self.geometry.link_name, joint_dict_last, self.gscene.urdf_content)
        Tto = np.matmul(np.linalg.inv(Tbt), Tbo)
        action_obj.set_state(Tto, self.geometry.link_name,
                             bind_point, self.name)

    def check_type(self, action_point):
        return action_point.ctype == self.ctype

    @abstractmethod
    def get_redundancy(self):
        pass

    ##
    # @brief function prototype to check quick availability of action point when building search tree
    @abstractmethod
    def check_available(self):
        pass


##
# @class PointerBinding
# @brief Base class for Pointer type binder. z-direction and contact are constrained.
# @remark not usable at this level
class PointerBinding(Binding):
    ##
    # @brief currently support only x-y plane
    def get_redundancy(self):
        if self.point:
            return {"w":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            return {"x":(-dims[0]/2,dims[0]/2),
                    "y":(-dims[1]/2,dims[1]/2),
                    "w":(-np.pi,np.pi)}


##
# @class FrameBinding
# @brief Base class for Frame type binder. Full orientation and contact are constrained.
# @remark not usable at this level
class FrameBinding(Binding):
    ##
    # @brief currently support only x-y plane
    def get_redundancy(self):
        if self.point:
            return {}
        else:
            dims =self.geometry.dims
            return {"x":(-dims[0]/2,dims[0]/2),
                    "y":(-dims[1]/2,dims[1]/2)}

################################# USABLE CLASS #########################################


##
# @class VacuumTool
# @brief Binding class for vacuum type tool. z-direction is constrained. (PointerBinding)
class VacuumTool(PointerBinding):
    controlled = True
    multiple = False
    ctype = ConstraintType.Vacuum

    ##
    # @brief vacuum tool is always available
    def check_available(self, joint_dict):
        return True


##
# @class Gripper2Tool
# @brief Binding class for 2-finger gripper type tool. z-direction is constrained. (PointerBinding)
class Gripper2Tool(PointerBinding):
    controlled = True
    multiple = False
    ctype = ConstraintType.Grasp2

    ##
    # @brief gripper tool is always available
    def check_available(self, joint_dict):
        return True


##
# @class FramedTool
# @brief Binding class for fully constrained tool. (FrameBinding)
class FramedTool(FrameBinding):
    controlled = True
    multiple = False
    ctype = ConstraintType.Frame

    ##
    # @brief frame tool is always available
    def check_available(self, joint_dict):
        return True


##
# @class PlacePlane
# @brief Binding class for placing plane. z-direction constrained. (PointerBinding)
class PlacePlane(PointerBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Place
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    ##
    # @brief place plane is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT


##
# @class PlaceFrame
# @brief Binding class for placing frame. Fully constrained. (FrameBinding)
class PlaceFrame(FrameBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Frame
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    ##
    # @brief place frame is only available when vertical direction is in range of VERTICAL_CUT (10 deg)
    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT


##
# @class FixtureSlot
# @brief Fixture slot to implement assembly object with bindings.
class FixtureSlot(PointerBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Fixture

    def check_available(self, joint_dict):
        return False


##
# @brief convert constraint type to Binding type
def ctype_to_btype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Gripper2Tool
    elif cstr == ConstraintType.Frame.name:
        return PlaceFrame
    elif cstr == ConstraintType.Place.name:
        return PlacePlane
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumTool
