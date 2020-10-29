from __future__ import print_function

from ..geometry.binding import *


class Binding(object):
    controlled = None
    multiple = None
    def __init__(self, name, link_name, urdf_content, point=None, _object=None):
        self.name = name
        self.urdf_content = urdf_content
        if _object is None:
            assert point is not None, "Give _object or point"
            _object = GeometryItem(name=self.name, link_name=link_name, gtype=GEOTYPE.SPHERE,
                                   center=point, dims=(0,0,0), collision=False, display=False, fixed=True)
            self.point = None
        else:
            self.point = point
        self.object = _object
        
    def bind(self, action_obj, bind_point, joint_dict_last):
        Tbo = action_obj.object.get_tf(joint_dict_last)
        Tbt = get_tf(self.object.link_name, joint_dict_last, self.urdf_content)
        Tto = np.matmul(np.linalg.inv(Tbt), Tbo)
        action_obj.set_state(Tto, self.object.link_name,
                             bind_point, self.name)
    
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
        