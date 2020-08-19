from __future__ import print_function
import time as timer
import numpy as np

from .geometry import *
from .constraint_base import *


class Binding(object):
    controlled = None
    multiple = None
    def __init__(self, name, link_name, urdf_content, point=None, _object=None, collision_items_dict=None):
        self.name = name
        self.urdf_content = urdf_content
        self.collision_items_dict = collision_items_dict
        if _object is None:
            assert point is not None, "Give _object or point"
            _object = GeoSphere(name=self.name, center=point, radius=0, 
                                link_name=link_name, urdf_content=self.urdf_content,
                                collision=False, display=False)
            self.point = None
        else:
            self.point = point
        self.object = _object
        
    def bind(self, action_obj, bind_point, joint_dict_last, collision_items_dict=None):
        if collision_items_dict is None: collision_items_dict = self.collision_items_dict
        else: self.collision_items_dict = collision_items_dict
        Tbo = action_obj.object.get_tf(joint_dict_last)
        Tbt = get_tf(self.object.link_name, joint_dict_last, self.urdf_content)
        Tto = np.matmul(np.linalg.inv(Tbt), Tbo)
        action_obj.set_state(Tto, self.object.link_name,
                             bind_point, self.name, collision_items_dict=collision_items_dict)
        
    def make_constraints(self, action_obj, handle_name):
        return action_obj.make_action_constraints(handle_name, self.effector, point=self.point)
    
    def check_type(self, action_point):
        return action_point.handle.__class__.__name__ == self.effector.__class__.__name__
            
        
class PointerBinding(Binding):
    def __init__(self, direction=None, **kwargs):
        super(PointerBinding, self).__init__(**kwargs)
        self.direction = direction
        self.effector = GeoPointer(direction=direction, _object=self.object)
            
        
class FrameBinding(Binding):
    def __init__(self, orientation=None, **kwargs):
        super(FrameBinding, self).__init__(**kwargs)
        self.orientation = orientation
        self.effector = GeoFrame(orientation=orientation, _object=self.object)
        
class PlacePlane(PointerBinding):
    controlled = False
    multiple = True
        
class VacuumTool(PointerBinding):
    controlled = True
    multiple = False
    
        
class PlaceFrame(FrameBinding):
    controlled = False
    multiple = True
        