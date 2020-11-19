from __future__ import print_function

from ..geometry.geometry import *
from .constraint_common import *


class Binding(ActionPoint):
    controlled = None
    multiple = None
        
    def bind(self, action_obj, bind_point, joint_dict_last):
        Tbo = action_obj.object.get_tf(joint_dict_last)
        Tbt = get_tf(self.object.link_name, joint_dict_last, self.ghnd.urdf_content)
        Tto = np.matmul(np.linalg.inv(Tbt), Tbo)
        action_obj.set_state(Tto, self.object.link_name,
                             bind_point, self.name)

    def check_type(self, action_point):
        return action_point.ctype == self.ctype

    @abstractmethod
    def get_redundancy(self):
        pass

    @abstractmethod
    def check_available(self):
        pass
            
        
class PointerBinding(Binding):
    def get_redundancy(self):
        if self.point:
            return {"w":(-np.pi,np.pi)}
        else:
            dims =self.object.dims
            return {"x":(-dims[0]/2,dims[0]/2),
                    "y":(-dims[1]/2,dims[1]/2), ## currently support only x-y plane
                    "w":(-np.pi,np.pi)}
        
class FrameBinding(Binding):
    def get_redundancy(self):
        if self.point:
            return {}
        else:
            dims =self.object.dims
            return {"x":(-dims[0]/2,dims[0]/2),
                    "y":(-dims[1]/2,dims[1]/2)} ## currently support only x-y plane

################################# USABLE CLASS #########################################

class VacuumTool(PointerBinding):
    controlled = True
    multiple = False
    ctype = ConstraintType.Vacuum

    def check_available(self, joint_dict):
        return True

class Gripper2Tool(PointerBinding):
    controlled = True
    multiple = False
    ctype = ConstraintType.Grasp2

    def check_available(self, joint_dict):
        return True

class PlacePlane(PointerBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Place
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT
    
        
class PlaceFrame(FrameBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Frame
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    def check_available(self, joint_dict):
        return self.get_tf_handle(joint_dict)[2,2]>self.VERTICAL_CUT

def ctype_to_btype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Gripper2Tool
    elif cstr == ConstraintType.Frame.name:
        return PlaceFrame
    elif cstr == ConstraintType.Place.name:
        return PlacePlane
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumTool
