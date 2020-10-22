from __future__ import print_function
import numpy as np
from collections import Iterable

from .utils import *
from .rotation_utils import *
from .joint_utils import get_tf, get_transformation, get_adjacent_links
from scipy.spatial.transform import Rotation

POINT_DEFAULT = np.array([[0,0,0]])
SEG_DEFAULT = np.array([[0,0,1.0],[0,0,-1.0]])/2
BOX_DEFAULT = np.array([[[(i,j,k) for k in range(2)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5

class GeometryItem(object):
    GLOBAL_GEO_LIST = []
    GLOBAL_GEO_DICT = {}
    def __init__(self, name, link_name, urdf_content, color=(0,1,0,1), display=True, collision=True,
                 soft=False, online=False, K_col=None):
        GeometryItem.GLOBAL_GEO_LIST += [self]
        GeometryItem.GLOBAL_GEO_DICT[name] = self
        self.color = color
        self.display = display
        self.collision = collision
        self.soft = soft
        self.online = online
        self.K_col = K_col
        self.urdf_content = urdf_content
        self.set_name(name)
        if self.urdf_content is not None:
            self.set_link(link_name)
    
    def set_name(self, name):
        self.name = name
        self.tf_name = "{}_tf".format(self.name)
        
    def set_link(self, link_name):
        self.link_name = link_name
        self.link = self.urdf_content.link_map[link_name]
        self.Tname = get_transformation(self.link_name)
        self.adjacent_links = get_adjacent_links(self.link_name)
        
    def get_representation(self, *args, **kwargs):
        raise NotImplementedError
        
    def get_radius(self):
        raise NotImplementedError
        
    def get_scale(self):
        raise NotImplementedError

    def get_tf_representation(self):
        orientation_mat = self.orientation_mat
        angle_option = ""
        if (3-np.sum(np.diag(orientation_mat)))>1e-4:
            zyxrot = Rot2zyx(orientation_mat)
            for i in range(len(zyxrot)):
                if abs(zyxrot[2-i])>1e-4:
                    angle_option = "*rotate_{axis}({val})".format(axis="xyz"[i], val=zyxrot[2-i])+angle_option
        center = self.get_center()
        center_option = ""
        if np.sum(np.abs(center))>1e-4:
            for i in range(len(center)):
                if abs(center[i])>1e-4:
                    center_option += "*translate_{axis}({val})".format(axis="xyz"[i], val=center[i])
        tf_text = "{tf_name} = {Tname}{center_option}{angle_option}".format(
            tf_name=self.tf_name, Tname=self.Tname, center_option=center_option, angle_option=angle_option)
        return tf_text
    
    def get_tf_name(self): # get_transformation
        return self.tf_name
                
        
    def get_tf(self, joint_dict):
        T = get_tf(to_link=self.link_name, joint_dict=joint_dict, urdf_content=self.urdf_content)
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
        return np.abs(Poff) + np.abs(np.matmul(Roff, self.get_scale()))/2

    def get_vertice_radius(self):
        raise NotImplementedError

        
class GeoSphere(GeometryItem):
    def __init__(self, center, radius, **kwargs):
        self.set_offset_tf(center, np.identity(3))
        self.radius = radius
        super(GeoSphere, self).__init__(**kwargs)
        
    def get_representation(self, point=None):
        if point is None:
            return "MultiSphere({{Vector({},{},{})}},{{ {} }})".format(0, 0, 0, self.radius)
        else:
            raise NotImplementedError
        
    def get_radius(self):
        return 0
        
    def get_scale(self):
        return [self.radius*2, self.radius*2, self.radius*2]

    def get_vertice_radius(self):
        return POINT_DEFAULT, self.radius

        
class GeoBox(GeometryItem):
    def __init__(self, center, BLH, orientation=(0,0,0), **kwargs):
        self.set_offset_tf(center, Rotation.from_rotvec(orientation).as_dcm())
        self.BLH = BLH
        super(GeoBox, self).__init__(**kwargs)
        
    def get_representation(self, point=None):
        if point is None:
            return "Box({B},{L},{H})".format(B=self.BLH[0], L=self.BLH[1], H=self.BLH[2])
        else:
            return "MultiSphere({{Vector({},{},{})}},{{ {} }})".format(*(tuple(point)+(0,)))
        
    def get_radius(self):
        return 0
        
    def get_scale(self):
        return self.BLH

    def get_vertice_radius(self):
        return np.multiply(BOX_DEFAULT, self.BLH), 0


class GeoSegment(GeometryItem):
    def __init__(self, point0, axis, length, radius, **kwargs):
        self.point0, self.axis, self.length, self.radius = \
            point0, axis, length, radius
        self.set_offset_tf(list(self.point0), self.get_axis_mat())
        if isinstance(self.axis, str):
            self.center["XYZ".index(axis)] += length/2
        super(GeoSegment, self).__init__(**kwargs)
    
    def get_axis_mat(self):
        if isinstance(self.axis, str):
            if self.axis == "X":
                return Rotation.from_rotvec((0,np.pi/2,0)).as_dcm()
            elif self.axis == "Y":
                return Rotation.from_rotvec((-np.pi/2,0,0)).as_dcm()
            elif self.axis == "Z":
                return Rotation.from_rotvec((0,0,0)).as_dcm()
        elif isinstance(self.axis, Iterable):
            if len(self.axis) == 3:
                return Rotation.from_euler('xyz', self.axis, degrees=False).as_dcm()
            elif len(self.axis) == 4:
                return Rotation.from_quat(self.axis).as_dcm()
        raise(NotImplementedError("Segment axis should be 'X', 'Y', 'Z', quaternion or rpy in radian"))
        
    def get_representation(self, point=None):
        if point is None:
            return "CapsuleZ({radius},{length})".format(radius=self.radius,length=self.length)
        else:
            raise NotImplementedError
        
    def get_radius(self):
        return self.radius
        
    def get_scale(self):
        return (self.radius*2, self.radius*2, self.length)

    def get_vertice_radius(self):
        return np.multiply(SEG_DEFAULT, self.length), self.radius

        
class GeoMesh(GeometryItem):
    def __init__(self, uri, BLH, scale=(1,1,1), center=(0,0,0), orientation_mat=np.identity(3), **kwargs):
        self.BLH, self.uri, self.scale = BLH, uri, scale
        self.set_offset_tf(center, orientation_mat)
        kwargs['collision'] = False
        super(GeoMesh, self).__init__(**kwargs)
        
    def get_representation(self, point=None):
        return "Box({B},{L},{H})".format(B=self.BLH[0], L=self.BLH[1], H=self.BLH[2])
        
    def get_radius(self):
        return 0
        
    def get_scale(self):
        return self.scale

    def get_vertice_radius(self):
        return np.multiply(BOX_DEFAULT, self.scale), 0

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