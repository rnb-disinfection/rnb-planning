from __future__ import print_function

from .constraint_common import *

from abc import *
__metaclass__ = type

class DirectedPoint(ActionPoint):
    def get_redundancy(self):
        return {"w":(-np.pi,np.pi)}

class FramedPoint(ActionPoint):
    def get_redundancy(self):
        return {}

################################# USABLE CLASS #########################################

class PlacePoint(DirectedPoint):
    ctype=ConstraintType.Place

class FramePoint(FramedPoint):
    ctype=ConstraintType.Frame

class VacuumPoint(DirectedPoint):
    ctype=ConstraintType.Vacuum

class Grasp2Point(DirectedPoint):
    ctype=ConstraintType.Grasp2

class FixturePoint(FramedPoint):
    ctype=ConstraintType.Fixture
    
class ObjectAction:
    def __init__(self):
        raise NotImplementedError("ObjectAction is abstract class")
        
    def set_state(self, frame, link_name, bind_point, binder):
        self.object.set_offset_tf(frame[:3, 3], frame[:3,:3])
        self.object.set_link(link_name)
        self.bind(bind_point, binder)
        for ap in self.action_points_dict.values():
            ap.update_handle()
        for bp in self.binder_points_dict.values():
            bp.update_handle()
    
    def bind(self, point, target):
        self.binding = (point, target)

    @abstractmethod
    def get_conflicting_handles(self, hname):
        pass

################################# USABLE CLASS #########################################
class CustomObject(ObjectAction):
    def __init__(self, _object, action_points_dict, binder_points_dict=None):
        if binder_points_dict is None:
            binder_points_dict = {}
        self.object = _object
        self.action_points_dict = action_points_dict
        self.binder_points_dict = binder_points_dict

    def get_conflicting_handles(self, hname):
        return [hname]

class SingleHandleObject(ObjectAction):
    def __init__(self, _object, action_point, binder_points_dict=None):
        if binder_points_dict is None:
            binder_points_dict = {}
        self.object = _object
        self.action_points_dict = {action_point.name: action_point}
        self.binder_points_dict = binder_points_dict

    def get_conflicting_handles(self, hname):
        return [hname]

class BoxAction(ObjectAction):
    def __init__(self, _object, hexahedral=True, binder_points_dict=None):
        if binder_points_dict is None:
            binder_points_dict = {}
        self.object = _object
        Xhalf, Yhalf, Zhalf = np.divide(_object.dims,2)
        self.binder_points_dict = binder_points_dict
        self.action_points_dict = {
            "top_p": PlacePoint("top_p", _object, [0,0,Zhalf], [np.pi,0,0]),
            "bottom_p": PlacePoint("bottom_p", _object, [0,0,-Zhalf], [0,0,0]),
            # "top_v": VacuumPoint("top_v", _object, [0,0,Zhalf], [0,0,-1]),
            # "bottom_v": VacuumPoint("bottom_v", _object, [0,0,-Zhalf], [0,0,1]),
            "top_g": Grasp2Point("top_g", _object, [0,0,0], [np.pi,0,0]),
            "bottom_g": Grasp2Point("bottom_g", _object, [0,0,0], [0,0,0]),
            "top_f": FramePoint("top_f", _object, [0,0,Zhalf], [np.pi,0,0]),
            "bottom_f": FramePoint("bottom_f", _object, [0,0,-Zhalf], [0,0,0])
        }
        if hexahedral:
            self.action_points_dict.update({
                "right_p": PlacePoint("right_p", _object, [Xhalf,0,0], [0,-np.pi/2,0]),
                "left_p": PlacePoint("left_p", _object, [-Xhalf,0,0], [0,np.pi/2,0]),
                "front_p": PlacePoint("front_p", _object, [0,-Yhalf,0], [-np.pi/2,0,0]),
                "back_p": PlacePoint("back_p", _object, [0,Yhalf,0], [np.pi/2,0,0]),
                # "right_v": VacuumPoint("right_v", _object, [Xhalf,0,0], [-1,0,0]),
                # "left_v": VacuumPoint("left_v", _object, [-Xhalf,0,0], [1,0,0]),
                # "front_v": VacuumPoint("front_v", _object, [0,-Yhalf,0], [0,1,0]),
                # "back_v": VacuumPoint("back_v", _object, [0,Yhalf,0], [0,-1,0]),
                "right_g": Grasp2Point("right_g", _object, [0,0,0], [0,-np.pi/2,0]),
                "left_g": Grasp2Point("left_g", _object, [0,0,0], [0,np.pi/2,0]),
                "front_g": Grasp2Point("front_g", _object, [0,0,0], [-np.pi/2,0,0]),
                "back_g": Grasp2Point("back_g", _object, [0,0,0], [np.pi/2,0,0]),
                "right_f": FramePoint("right_f", _object, [Xhalf,0,0], [0,-np.pi/2,0]),
                "left_f": FramePoint("left_f", _object, [-Xhalf,0,0], [0,np.pi/2,0]),
                "front_f": FramePoint("front_f", _object, [0,-Yhalf,0], [-np.pi/2,0,0]),
                "back_f": FramePoint("back_f", _object, [0,Yhalf,0], [np.pi/2,0,0])
            })
            self.conflict_dict = {
                hname: [hname[:-1]+postfix for postfix in "pgf"] +
                       ([OPPOSITE_DICT[hname[:-2]]+"_"+postfix for postfix in "pgf"] \
                            if hname[-1] == "g" else [OPPOSITE_DICT[hname[:-2]]+"_g"])
                for hname in self.action_points_dict.keys()
            }

    def get_conflicting_handles(self, hname):
        return self.conflict_dict[hname]

def ctype_to_htype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Grasp2Point
    elif cstr == ConstraintType.Frame.name:
        return FramePoint
    elif cstr == ConstraintType.Place.name:
        return PlacePoint
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumPoint

def otype_to_class(ostr):
    if ostr == 'BoxAction':
        return BoxAction
