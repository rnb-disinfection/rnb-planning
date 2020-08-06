from __future__ import print_function
from scipy.spatial.transform import Rotation
import numpy as np

from .joint_utils import get_tf, get_transformation


class GeometryItem(object):
    def __init__(self, name, link_name, urdf_content, color=(0,1,0,1), display=True, collision=True):
        self.color = color
        self.display = display
        self.collision = collision
        self.urdf_content = urdf_content
        self.set_name(name)
        self.set_link(link_name)
    
    def get_adjacent_links(self):
        adjacent_links = [self.link_name]
        for k, v in self.urdf_content.joint_map.items():
            if v.parent == self.link_name:
                adjacent_links += [v.child]
            if v.child == self.link_name:
                adjacent_links += [v.parent]
        return list(set(adjacent_links))
    
    def set_name(self, name):
        self.name = name
        
    def set_link(self, link_name):
        self.link_name = link_name
        self.link = self.urdf_content.link_map[link_name]
        self.Tname = get_transformation(self.link_name)
        self.adjacent_links = self.get_adjacent_links()
        
    def get_representation(self, *args, **kwargs):
        raise NotImplementedError
        
    def get_radius(self):
        raise NotImplementedError
        
    def get_scale(self):
        raise NotImplementedError
    
    def get_transformation(self):
        orientation = self.get_orientation()
        angle_option = ""
        if np.sum(np.abs(orientation[:3]))>1e-4:
            xyzrot = Rotation.from_quat(orientation).as_euler("xyz")
            for i in range(len(xyzrot)):
                if abs(xyzrot[i])>1e-4:
                    angle_option = "*rotate_{axis}({val})".format(axis="xyz"[i], val=xyzrot[i])+angle_option
        center = self.get_center()
        center_option = ""
        if np.sum(np.abs(center))>1e-4:
            for i in range(len(center)):
                if abs(center[i])>1e-4:
                    center_option += "*translate_{axis}({val})".format(axis="xyz"[i], val=center[i])
        tf_text = "{Tname}{center_option}{angle_option}".format(
            Tname=self.Tname, center_option=center_option, angle_option=angle_option)
        return tf_text
                
        
    def get_tf(self, joint_dict):
        T = get_tf(to_link=self.link_name, joint_dict=joint_dict, urdf_content=self.urdf_content)
        T = np.matmul(T, self.get_offset_tf())
        return T
        
    def get_offset_tf(self):
        T = np.identity(4)
        T[:3,3] = self.get_center()
        T[:3,:3] = Rotation.from_quat(self.get_orientation()).as_dcm()
        return T
        
    def get_center(self):
        return self.center
        
    def get_orientation(self):
        return self.orientation
        
    def set_center(self, center):
        self.center = center
        
    def set_orientation(self, orientation):
        self.orientation = orientation
        
    def get_frame(self):
        return tuple(self.get_center())+tuple(self.get_orientation())

        
class GeoSphere(GeometryItem):
    def __init__(self, center, radius, **kwargs):
        self.center, self.radius = center, radius
        self.orientation= (0,0,0,1)
        super(GeoSphere, self).__init__(**kwargs)
        
    def get_representation(self, point=None):
        if point is None:
            return "MultiSphere({{Vector({},{},{})}},{{ {} }})".format(0, 0, 0, self.radius)
        else:
            raise NotImplementedError
        
    def get_radius(self):
        return 0
        
    def get_scale(self):
        return [self.radius, self.radius, self.radius]

        
class GeoBox(GeometryItem):
    def __init__(self, center, BLH, **kwargs):
        self.center, self.BLH = center, BLH
        self.orientation= (0,0,0,1)
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


class GeoSegment(GeometryItem):
    def __init__(self, point0, axis, length, radius, **kwargs):
        self.point0, self.axis, self.length, self.radius = \
            point0, axis, length, radius
        self.orientation=self.get_axis_quat()
        self.center = list(self.point0)
        self.center["XYZ".index(axis)] += length/2
        super(GeoSegment, self).__init__(**kwargs)
    
    def get_axis_quat(self):
        if self.axis == "X":
            return Rotation.from_rotvec((0,np.pi/2,0)).as_quat()
        elif self.axis == "Y":
            return Rotation.from_rotvec((-np.pi/2,0,0)).as_quat()
        elif self.axis == "Z":
            return Rotation.from_rotvec((0,0,0)).as_quat()
        
    def get_representation(self, point=None):
        if point is None:
            return "CapsuleZ({radius},{length})".format(radius=self.radius,length=self.length)
        else:
            raise NotImplementedError
        
    def get_radius(self):
        return self.radius
        
    def get_scale(self):
        return (self.radius*2, self.radius*2, self.length)
    
class GeoPointer:
    def __init__(self, _object, direction=None):
        assert direction is not None, "GeoPoint need direction"
        self.__direction = direction
        self.direction = self.__direction
        self.object = _object
    
    def set_direction(self, direction):
        self.direction = direction
    
    def get_direction(self):
        return self.direction
    
class GeoFrame:
    def __init__(self, _object, orientation=None):
        assert orientation is not None, "GeoFrame need orientation"
        assert orientation is not None, "GeoFrame orientation should be rotation vector"
        self.__orientation = orientation
        self.orientation = self.__orientation
        self.object = _object
    
    def set_orientation(self, orientation):
        self.orientation = orientation
    
    def get_orientation(self):
        return self.orientation

        
class GeoMesh(GeometryItem):
    def __init__(self, uri, BLH, scale=(1,1,1), center=(0,0,0), orientation=(0,0,0,1), **kwargs):
        self.BLH, self.uri, self.center, self.scale = BLH, uri, center, scale
        self.orientation= (0,0,0,1)
        super(GeoMesh, self).__init__(**kwargs)
        
    def get_representation(self, point=None):
        return "Box({B},{L},{H})".format(B=self.BLH[0], L=self.BLH[1], H=self.BLH[2])
        
    def get_radius(self):
        return 0
        
    def get_scale(self):
        return self.scale