from __future__ import print_function
import numpy as np
from scipy.spatial.transform import Rotation

from .constraint_base import *

class ActionPoint:
    def make_constraints(self, *args, **kwargs):
        raise NotImplementedError

class DirectedPoint(ActionPoint):
    def __init__(self, name, _object, point_dir):
        self.name = name
        self.object = _object
        self.point_dir = point_dir
        self.name_constraint = "pointer_{objname}_{name}".format(objname=self.object.name, name=self.name)
        self.update_handle()

    def update_handle(self):
        Toff = self.object.get_offset_tf()
        self.point = np.matmul(Toff, list(self.point_dir[0])+[1])[:3]
        self.direction = np.matmul(Toff[:3,:3], self.point_dir[1])
        if hasattr(self, "handle"):
            self.handle.object.set_link(self.object.link_name)
            self.handle.object.set_center(self.point)
            self.handle.set_direction(self.direction)
        else:
            self.handle = GeoPointer(direction=self.direction, 
                                      _object=GeoSphere(
                                          name=self.name_constraint, center=self.point, radius=0,
                                          link_name=self.object.link_name, urdf_content=self.object.urdf_content,
                                          collision=False, display=False)
                                     )
    
    def make_constraints(self, effector, point=None):
        return make_directed_point_constraint(self.handle, effector, self.name_constraint, point2=point)

class FramedPoint(ActionPoint):
    def __init__(self, name, _object, point_ori):
        self.name = name
        self.object = _object
        self.point_ori = point_ori
        self.name_constraint = "framer_{objname}_{name}".format(objname=self.object.name, name=self.name)
        self.update_handle()

    def update_handle(self):
        Toff = self.object.get_offset_tf()
        self.point = np.matmul(Toff, list(self.point_ori[0])+[1])[:3]
        self.orientation = Rotation.from_dcm(np.matmul(Toff[:3,:3], Rotation.from_rotvec(self.point_ori[1]).as_dcm())).as_rotvec()
        if hasattr(self, "handle"):
            self.handle.object.set_link(self.object.link_name)
            self.handle.object.set_center(self.point)
            self.handle.set_orientation(self.orientation)
        else:
            self.handle = GeoFrame(orientation=self.orientation,
                                   _object=GeoSphere(
                                       name=self.name_constraint, center=self.point, radius=0,
                                       link_name=self.object.link_name, urdf_content=self.object.urdf_content,
                                       collision=False, display=False)
                                   )
    
    def make_constraints(self, effector, point=None):
        return make_oriented_point_constraint(self.handle, effector, self.name_constraint, point2=point)
    
class ObjectAction:
    def __init__(self):
        raise NotImplementedError("ObjectAction is abstract class")
    
    def get_action_points(self):
        return self.action_points_dict
        
    def set_state(self, frame, link_name, bind_point, binder, collision_items_dict):
        collision_items_dict[self.object.link_name].remove(self.object)
        collision_items_dict[link_name] += [self.object]
        self.object.set_center(frame[:3])
        self.object.set_orientation(frame[3:])
        self.object.set_link(link_name)
        self.bind(bind_point, binder)
        for ap in self.action_points_dict.values():
            ap.update_handle()
    
    def bind(self, point, target):
        self.binding = (point, target)
        
    def make_action_constraints(self, point_name, target, point=None):
        const_txt = self.action_points_dict[point_name].make_constraints(target, point=point)
        return const_txt
        
class BoxAction(ObjectAction):
    def __init__(self, _object, hexahedral=False):
        self.object = _object
        Xhalf, Yhalf, Zhalf = np.array(_object.BLH)/2
        self.action_points_dict = {
            "top_p": DirectedPoint("top_p", _object, ([0,0,Zhalf], [0,0,-1])),
            "bottom_p": DirectedPoint("bottom_p", _object, ([0,0,-Zhalf], [0,0,1])),
            "top_f": FramedPoint("top_f", _object, ([0,0,Zhalf], [np.pi,0,0])),
            "bottom_f": FramedPoint("bottom_f", _object, ([0,0,-Zhalf], [0,0,0]))
        }
        if hexahedral:
            self.action_points_dict.update({
                "right_p": DirectedPoint("right_p", _object, ([Xhalf,0,0], [-1,0,0])),
                "left_p": DirectedPoint("left_p", _object, ([-Xhalf,0,0], [1,0,0])),
                "front_p": DirectedPoint("front_p", _object, ([0,-Yhalf,0], [0,1,0])),
                "back_p": DirectedPoint("back_p", _object, ([0,Yhalf,0], [0,-1,0])),
                "right_f": FramedPoint("right_f", _object, ([Xhalf,0,0], [0,-np.pi/2,0])),
                "left_f": FramedPoint("left_f", _object, ([-Xhalf,0,0], [0,np.pi/2,0])),
                "front_f": FramedPoint("front_f", _object, ([0,-Yhalf,0], [-np.pi/2,0,0])),
                "back_f": FramedPoint("back_f", _object, ([0,Yhalf,0], [np.pi/2,0,0]))
            })