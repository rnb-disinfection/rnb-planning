from .geometry import *

from abc import *
__metaclass__ = type


class GeoBinding:
    def __init__(self, _object, direction=None):
        assert direction is not None, "GeoBinding need direction"
        self.set_binding_direction(direction)
        self.object = _object

    def set_binding_direction(self, direction):
        self.direction = direction

    def get_binding_direction(self):
        return self.direction


    def __del__(self):
        if self.object is not None:
            self.object.ghnd.remove(self.object)


class GeoPointer(GeoBinding):
    pass

class GeoFrame(GeoBinding):
    def __init__(self, _object, direction=None, orientation_mat=None):
        assert not (direction is None and orientation_mat is None), "GeoFrame need direction"
        if direction is None:
            direction = Rot2rpy(orientation_mat)
        GeoBinding.__init__(self, _object, direction)

    def set_binding_direction(self, direction):
        self.direction = direction
        self.orientation_mat = Rot_rpy(direction)

    def set_binding_orientation_mat(self, orientation_mat):
        self.orientation_mat = orientation_mat
