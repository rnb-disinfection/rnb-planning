from .geometry import *

from abc import *
__metaclass__ = type


class GeoBinding:
    def __del__(self):
        if self.object is not None:
            self.object.ghnd.remove(self.object)


class GeoPointer(GeoBinding):
    def __init__(self, _object, direction=None):
        assert direction is not None, "GeoPoint need direction"
        self.__direction = direction
        self.direction = self.__direction
        self.object = _object

    def set_pointer_direction(self, direction):
        self.direction = direction

    def get_pointer_direction(self):
        return self.direction


class GeoFrame(GeoBinding):
    def __init__(self, _object, orientation=None, orientation_mat=None):
        assert not (orientation is None and orientation_mat is None), "GeoFrame need orientation"
        if orientation is not None:
            assert len(orientation) == 3, "GeoFrame orientation should be rotation vector"
            self.orientation_mat = Rotation.from_rotvec(orientation).as_dcm()
        else:
            self.orientation_mat = orientation_mat
        self.object = _object

    def set_frame_orientation_mat(self, orientation_mat):
        self.orientation_mat = orientation_mat

    def get_frame_orientation_mat(self):
        return self.orientation_mat
