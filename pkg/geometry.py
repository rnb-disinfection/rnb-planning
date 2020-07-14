from enum import Enum

class GeoType(Enum):
    SPHERE = 0
    LINE = 1
    BOX = 2
    CYLINDER = 3
    MESH = 4

class GeometryItem:
    def __init__(self, name, gtype, dims, color=(0,1,0,1), display=True, collision=True, scale=(1,1,1), uri=None):
        self.name = name
        self.gtype = gtype
        self.dims = dims
        self.scale = scale
        self.uri = uri
        self.color, self.display, self.collision = color, display, collision
    
    def get_rviz_scale(self):
        return self.dims
        
class GeometryFrame:
    def __init__(self, Toff, link_name):
        self.Toff, self.link_name = Toff, link_name