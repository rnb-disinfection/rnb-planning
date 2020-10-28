from __future__ import print_function
import numpy as np
from collections import Iterable

from .utils import *
from .rotation_utils import *
from .joint_utils import get_tf, get_adjacent_links
from scipy.spatial.transform import Rotation
from enum import Enum

class GEOTYPE(Enum):
    SPHERE = 0
    SEGMENT = 1
    BOX = 2
    MESH = 3

POINT_DEFAULT = np.array([[0,0,0]])
SEG_DEFAULT = np.array([[0,0,1.0],[0,0,-1.0]])/2
BOX_DEFAULT = np.array([[[(i,j,k) for k in range(2)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5

DEFAULT_VERT_DICT = {
    GEOTYPE.SPHERE: POINT_DEFAULT,
    GEOTYPE.SEGMENT: SEG_DEFAULT,
    GEOTYPE.BOX: BOX_DEFAULT,
    GEOTYPE.MESH: BOX_DEFAULT
}


class GeometryHandle(Singleton, list):
    def __init__(self, urdf_content):
        self.urdf_content = urdf_content
        self.NAME_DICT = {}

    def append(self, geo):
        list.append(self, geo)
        assert geo.name not in self.NAME_DICT, "geometry name already registered"
        self.NAME_DICT[geo.name] = geo

    def clear(self):
        list.clear(self)
        self.NAME_DICT.clear()

    def remove(self, geo):
        list.remove(self, geo)
        del self.NAME_DICT[geo.name]


class GeometryItem(object):
    def __init__(self, gtype, name, link_name, dims, center, rpy=(0,0,0), color=(0,1,0,1), display=True, collision=True,
                 soft=False, online=False, K_col=None, uri="", scale=(1,1,1)):
        self.ghnd = GeometryHandle.instance()
        self.uri, self.scale = uri, scale
        self.gtype = gtype
        self.set_offset_tf(center=center, orientation_mat=Rot_rpy(rpy))
        self.dims = dims
        self.radius = np.mean(dims[:2])/2 if gtype in [GEOTYPE.SPHERE, GEOTYPE.SEGMENT] else 0
        self.length = dims[2]
        self.color = color
        self.display = display
        self.collision = collision
        self.soft = soft
        self.online = online
        self.K_col = K_col
        self.set_name(name)
        self.set_link(link_name)
        self.ghnd.append(self)
    
    def set_name(self, name):
        self.name = name
        
    def set_link(self, link_name):
        self.link_name = link_name
        self.adjacent_links = get_adjacent_links(self.link_name)

    def get_radius(self):
        raise NotImplementedError
        
    def get_dims(self):
        return self.dims

    def get_tf(self, joint_dict):
        T = get_tf(to_link=self.link_name, joint_dict=joint_dict, urdf_content=self.ghnd.urdf_content)
        T = np.matmul(T, self.get_offset_tf())
        return T
        
    def get_offset_tf(self):
        return self.Toff
        
    def get_center(self):
        return self.center
        
    def set_center(self, center):
        self.center = center
        
    def set_orientation_mat(self, orientation_mat):
        self.orientation_mat = orientation_mat

    def set_offset_tf(self, center=None, orientation_mat=None):
        self.center = center if center is not None else self.center
        self.orientation_mat = orientation_mat if orientation_mat is not None else self.orientation_mat
        self.Toff = SE3(self.orientation_mat, self.center)

    def get_frame(self):
        return SE3(self.orientation_mat, self.center)

    def get_off_max(self):
        Toff = self.get_offset_tf()
        Roff, Poff = Toff[:3, :3], Toff[:3, 3]
        return np.abs(Poff) + np.abs(np.matmul(Roff, self.get_dims()))/2

    def get_vertice_radius(self):
        return np.multiply(DEFAULT_VERT_DICT[self.gtype], self.dims), self.radius


class GeoPointer:
    def __init__(self, _object, direction=None):
        assert direction is not None, "GeoPoint need direction"
        self.__direction = direction
        self.direction = self.__direction
        self.object = _object

    def set_pointer_direction(self, direction):
        self.direction = direction

    def get_pointer_direction(self):
        return self.direction


class GeoFrame:
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


class GeoDefinition:
    def __init__(self, gtype, scale):
        self.gtype=gtype
        self.scale=scale

