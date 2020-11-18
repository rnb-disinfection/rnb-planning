from __future__ import print_function

from ..geometry.binding import *
from .constraint_common import *


class Binding(object):
    controlled = None
    multiple = None
    def __init__(self, name, link_name, urdf_content, point=None, _object=None):
        self.name = name
        self.urdf_content = urdf_content
        self.point_offset = point
        if _object is None:
            assert point is not None, "Give _object or point"
            _object = GeometryHandle.instance().create_safe(name=self.name, link_name=link_name, gtype=GEOTYPE.SPHERE,
                                   center=point, dims=(0,0,0), collision=False, display=False, fixed=True)
            self.point = None
        else:
            if self.name == _object.name:
                self.point = None
            else:
                self.point = point
        self.object = _object
        self.effector = None
        
    def bind(self, action_obj, bind_point, joint_dict_last):
        Tbo = action_obj.object.get_tf(joint_dict_last)
        Tbt = get_tf(self.object.link_name, joint_dict_last, self.urdf_content)
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

    def __del__(self):
        if self.object:
            if self.name == self.object.name:
                GeometryHandle.instance().remove(self.object)
            else:
                self.object = None

        if self.effector:
            if self.effector.object:
                if self.name == self.effector.object.name:
                    GeometryHandle.instance().remove(self.object)
                else:
                    self.effector.object = None

            
        
class PointerBinding(Binding):
    def __init__(self, direction=None, **kwargs):
        super(PointerBinding, self).__init__(**kwargs)
        self.direction = direction
        self.effector = GeoPointer(direction=direction, _object=self.object)

    def get_redundancy(self):
        if self.point_offset:
            return {"w":(-np.pi,np.pi)}
        else:
            dims =self.object.get_dims()
            return {"x":(-dims[0]/2,dims[0]/2),
                    "y":(-dims[1]/2,dims[1]/2), ## currently support only x-y plane
                    "w":(-np.pi,np.pi)}
        
class FrameBinding(Binding):
    def __init__(self, direction=None, **kwargs):
        super(FrameBinding, self).__init__(**kwargs)
        self.direction = direction
        self.effector = GeoFrame(direction=direction, _object=self.object)

    def get_redundancy(self):
        if self.point_offset:
            return {}
        else:
            dims =self.object.get_dims()
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
        return np.matmul(self.effector.object.get_tf(joint_dict)[:3,:3], self.direction)[2]>PlacePlane.VERTICAL_CUT
    
        
class PlaceFrame(FrameBinding):
    controlled = False
    multiple = True
    ctype = ConstraintType.Frame
    VERTICAL_CUT = np.cos(np.deg2rad(10))

    def check_available(self, joint_dict):
        return np.matmul(self.effector.object.get_tf(joint_dict)[:3,:3], self.direction)[2]>PlaceFrame.VERTICAL_CUT

def ctype_to_btype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Gripper2Tool
    elif cstr == ConstraintType.Frame.name:
        return PlaceFrame
    elif cstr == ConstraintType.Place.name:
        return PlacePlane
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumTool
