from __future__ import print_function

from ..geometry.binding import *
from .constraint_common import *

from abc import *
__metaclass__ = type


class ActionPoint:
    def __init__(self, name, _object, point_dir):
        self.name = name
        self.object = _object
        self.set_point_dir(point_dir)

    def set_point_dir(self, point_dir):
        self.point_dir = point_dir

    @abstractmethod
    def update_handle(self):
        pass

    def __del__(self):
        if self.handle is not None:
            del self.handle

class DirectedPoint(ActionPoint):
    def __init__(self, name, _object, point_dir):
        ActionPoint.__init__(self, name, _object, point_dir)
        self.name_constraint = "pointer_{objname}_{name}".format(objname=self.object.name, name=self.name)
        self.update_handle()

    def update_handle(self):
        Toff = self.object.get_offset_tf()
        self.point = np.matmul(Toff, list(self.point_dir[0])+[1])[:3]
        self.direction = np.matmul(Toff[:3,:3], self.point_dir[1])
        if hasattr(self, "handle"):
            self.handle.object.set_link(self.object.link_name)
            self.handle.object.set_center(self.point)
            self.handle.set_binding_direction(self.direction)
        else:
            self.handle = GeoPointer(direction=self.direction, 
                                      _object=GeometryItem(
                                          gtype=GEOTYPE.SPHERE, name=self.name_constraint, link_name=self.object.link_name,
                                          center=self.point, dims=(0,0,0), collision=False, display=False, fixed=False)
                                     )

class FramedPoint(ActionPoint):
    def __init__(self, name, _object, point_dir):
        ActionPoint.__init__(self, name, _object, point_dir)
        self.name_constraint = "framer_{objname}_{name}".format(objname=self.object.name, name=self.name)
        self.update_handle()

    def set_point_dir(self, point_dir):
        self.point_dir = point_dir
        self.R_point_ori = Rot_rpy(self.point_dir[1])

    def update_handle(self):
        Toff = self.object.get_offset_tf()
        self.point = np.matmul(Toff, list(self.point_dir[0])+[1])[:3]
        self.orientation_mat = np.matmul(Toff[:3,:3], self.R_point_ori)
        if hasattr(self, "handle"):
            self.handle.object.set_link(self.object.link_name)
            self.handle.object.set_center(self.point)
            self.handle.set_binding_orientation_mat(self.orientation_mat)
        else:
            self.handle = GeoFrame(orientation_mat=self.orientation_mat,
                                   _object=GeometryItem(
                                       gtype=GEOTYPE.SPHERE, name=self.name_constraint, link_name=self.object.link_name,
                                       center=self.point, dims=(0,0,0), collision=False, display=False, fixed=False)
                                   )

################################# USABLE CLASS #########################################

class PlacePoint(DirectedPoint):
    ctype=ConstraintType.Place

class FramePoint(FramedPoint):
    ctype=ConstraintType.Frame

class VacuumPoint(DirectedPoint):
    ctype=ConstraintType.Vacuum

class Grasp2Point(DirectedPoint):
    ctype=ConstraintType.Grasp2
    
class ObjectAction:
    def __init__(self):
        raise NotImplementedError("ObjectAction is abstract class")
    
    def get_action_points(self):
        return self.action_points_dict
        
    def set_state(self, frame, link_name, bind_point, binder):
        self.object.set_offset_tf(frame[:3, 3], frame[:3,:3])
        self.object.set_link(link_name)
        self.bind(bind_point, binder)
        for ap in self.action_points_dict.values():
            ap.update_handle()
    
    def bind(self, point, target):
        self.binding = (point, target)

    @abstractmethod
    def get_conflicting_handles(self, hname):
        pass

################################# USABLE CLASS #########################################

class BoxAction(ObjectAction):
    def __init__(self, _object, hexahedral=False):
        self.object = _object
        Xhalf, Yhalf, Zhalf = np.divide(_object.dims,2)
        self.action_points_dict = {
            "top_p": PlacePoint("top_p", _object, ([0,0,Zhalf], [0,0,-1])),
            "bottom_p": PlacePoint("bottom_p", _object, ([0,0,-Zhalf], [0,0,1])),
            # "top_v": VacuumPoint("top_v", _object, ([0,0,Zhalf], [0,0,-1])),
            # "bottom_v": VacuumPoint("bottom_v", _object, ([0,0,-Zhalf], [0,0,1])),
            "top_g": Grasp2Point("top_g", _object, ([0,0,0], [0,0,-1])),
            "bottom_g": Grasp2Point("bottom_g", _object, ([0,0,0], [0,0,1])),
            "top_f": FramePoint("top_f", _object, ([0,0,Zhalf], [np.pi,0,0])),
            "bottom_f": FramePoint("bottom_f", _object, ([0,0,-Zhalf], [0,0,0]))
        }
        if hexahedral:
            self.action_points_dict.update({
                "right_p": PlacePoint("right_p", _object, ([Xhalf,0,0], [-1,0,0])),
                "left_p": PlacePoint("left_p", _object, ([-Xhalf,0,0], [1,0,0])),
                "front_p": PlacePoint("front_p", _object, ([0,-Yhalf,0], [0,1,0])),
                "back_p": PlacePoint("back_p", _object, ([0,Yhalf,0], [0,-1,0])),
                # "right_v": VacuumPoint("right_v", _object, ([Xhalf,0,0], [-1,0,0])),
                # "left_v": VacuumPoint("left_v", _object, ([-Xhalf,0,0], [1,0,0])),
                # "front_v": VacuumPoint("front_v", _object, ([0,-Yhalf,0], [0,1,0])),
                # "back_v": VacuumPoint("back_v", _object, ([0,Yhalf,0], [0,-1,0])),
                "right_g": Grasp2Point("right_g", _object, ([0,0,0], [-1,0,0])),
                "left_g": Grasp2Point("left_g", _object, ([0,0,0], [1,0,0])),
                "front_g": Grasp2Point("front_g", _object, ([0,0,0], [0,1,0])),
                "back_g": Grasp2Point("back_g", _object, ([0,0,0], [0,-1,0])),
                "right_f": FramePoint("right_f", _object, ([Xhalf,0,0], [0,-np.pi/2,0])),
                "left_f": FramePoint("left_f", _object, ([-Xhalf,0,0], [0,np.pi/2,0])),
                "front_f": FramePoint("front_f", _object, ([0,-Yhalf,0], [-np.pi/2,0,0])),
                "back_f": FramePoint("back_f", _object, ([0,Yhalf,0], [np.pi/2,0,0]))
            })
            self.conflict_dict = {
                hname: [hname[:-1]+postfix for postfix in "pgf"] +
                       ([OPPOSITE_DICT[hname[:-2]]+"_"+postfix for postfix in "pgf"] \
                            if hname[-1] == "g" else [OPPOSITE_DICT[hname[:-2]]+"_g"])
                for hname in self.action_points_dict.keys()
            }

    def get_conflicting_handles(self, hname):
        return self.conflict_dict[hname]