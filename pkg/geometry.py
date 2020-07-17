from enum import Enum
import trimesh
import numpy as np

ICS_DEFAULT = np.array(trimesh.creation.icosphere(1,0.5).vertices)
N_mesh = len(ICS_DEFAULT)
CYL_DEFAULT = np.array(trimesh.creation.cylinder(0.5,1,sections = int((N_mesh-2)/2)).vertices)
BOX_DEFAULT = np.array(trimesh.creation.box((1,1,1)).vertices)
BOX_DEFAULT = np.pad(BOX_DEFAULT,((0,N_mesh-len(BOX_DEFAULT)),(0,0)))

class GeoType(Enum):
    SPHERE = 0
    LINE = 1
    BOX = 2
    CYLINDER = 3
    MESH = 4

class GeometryItem:
    def __init__(self, name, gtype, dims, color=(0,1,0,1), display=True, collision=True, uri=None):
        self.name = name
        self.color, self.display, self.collision = color, display, collision
        self.set_shape(gtype, dims, uri)
        
    def set_shape(self, gtype, dims, uri):
        self.gtype = gtype
        self.dims = dims
        self.uri = uri
        if gtype == GeoType.SPHERE:
            self.vertice = ICS_DEFAULT*np.reshape(dims, (1,3))
        elif gtype == GeoType.LINE:
            self.vertice = CYL_DEFAULT*np.reshape(dims, (1,3))
        elif gtype == GeoType.BOX:
            self.vertice = BOX_DEFAULT*np.reshape(dims, (1,3))
        elif gtype == GeoType.CYLINDER:
            self.vertice = CYL_DEFAULT*np.reshape(dims, (1,3))
        elif gtype == GeoType.MESH:
            self.vertice = BOX_DEFAULT*np.reshape(dims, (1,3))
        
    def get_vertice(self):
        return self.vertice
    
    def get_rviz_scale(self):
        return self.dims
        
class GeometryFrame:
    def __init__(self, Toff, link_name):
        self.Toff, self.link_name = Toff, link_name
        
        
# import matplotlib.pyplot as plt
# import mpl_toolkits.mplot3d as mplot3d

# fig = plt.figure(figsize=(15, 5))

# sub = fig.add_subplot(1,3,1,projection="3d")
# x,y,z = np.transpose(ICS_DEFAULT)
# sub.plot(x,y,z,'.')

# sub = fig.add_subplot(1,3,2,projection="3d")
# x,y,z = np.transpose(CYL_DEFAULT)
# sub.plot(x,y,z,'.')

# sub = fig.add_subplot(1,3,3,projection="3d")
# x,y,z = np.transpose(BOX_DEFAULT)
# sub.plot(x,y,z,'.')

# cyl = trimesh.creation.cylinder(0.5,1,sections = int((N_mesh-2)/2))
# np.savetxt("face_cyl.csv", cyl.faces, delimiter=",")