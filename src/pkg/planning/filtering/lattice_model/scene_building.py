from ....utils.gjk import get_point_list, get_gjk_distance
from ...constraint.constraint_subject import CustomObject, Grasp2Point, PlacePoint, SweepPoint, SweepTask, CylinderObject
import numpy as np
from ....utils.utils import *
from ....utils.rotation_utils import *
from ....geometry.geometry import *

##
# @class ObstacleBase
# @brief base class for obstacle generators
class ObstacleBase:
    RTH_MIN = None ## R: center ~ nearest point
    RTH_MAX = None
    RPY_MIN = None
    RPY_MAX = None
    DIM_MIN = None
    DIM_MAX = None
    GTYPE = None
    COLOR = (0.7,0.7,0.7,1)
    
    def __init__(self, gscene, name, sampler=np.random.uniform, DIM=None, RTH=None, RPY=None):
        self.name = name
        self.DIM = sampler(self.DIM_MIN, self.DIM_MAX) if DIM is None else DIM
        self.RTH = sampler(self.RTH_MIN, self.RTH_MAX) if RTH is None else RTH
        self.RPY = sampler(self.RPY_MIN, self.RPY_MAX) if RPY is None else RPY
        self.RPY[2] += self.RTH[1]
        self.XYZ = np.array(cyl2cart(*self.RTH))
        verts_rotated = np.matmul(Rot_rpy(self.RPY), (DEFAULT_VERT_DICT[self.GTYPE]*self.DIM).transpose())
        xy_normed = self.XYZ[:2]/(np.linalg.norm(self.XYZ[:2])+1e-6)
        verts_r_compo = np.dot(xy_normed, verts_rotated[:2,:])
        self.XYZ[:2] -= xy_normed[:2]*np.min(verts_r_compo)
        self.RTH[0] -= np.min(verts_r_compo)
        self.geometry = gscene.create_safe(gtype=self.GTYPE, name=self.name, link_name="base_link", 
                                  dims=self.DIM, center=tuple(self.XYZ), rpy=self.RPY,
                                  color=self.COLOR, display=True, collision=True, fixed=True)
        self.subgeo_list = []
        
    def is_overlapped_with(self, gtem):
        gscene = gtem.gscene
        res = False
        for gname in gtem.get_family():
            gchild = gscene.NAME_DICT[gname]
            verts, radii = gchild.get_vertice_radius()
            verts_global = np.add(np.matmul(verts, gchild.orientation_mat.transpose()), gchild.center)
            for gname_me in self.geometry.get_family():
                gchild_me = gscene.NAME_DICT[gname_me]
                verts_me, raddii_me = gchild_me.get_vertice_radius()
                verts_me_global = np.add(np.matmul(verts_me, gchild_me.orientation_mat.transpose()), 
                                         gchild_me.center)
                res = get_gjk_distance(get_point_list(verts_global), get_point_list(verts_me_global))-radii-raddii_me < 1e-4
                if res:
                    break
            if res:
                break
        return res
        
##
# @class WorkPlane
# @brief working plane. target and obstacle objects are generated on this plane
class WorkPlane(ObstacleBase):
    RTH_MIN = (0.3, -np.pi/2, -0.2)
    RTH_MAX = (0.8, +np.pi/2, +0.5)
    RPY_MIN = (0, 0, -np.pi/6)
    RPY_MAX = (0, 0, +np.pi/6)
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.8, 1.5, 0.1)
    GTYPE = GEOTYPE.BOX
    COLOR=  (0.8,0.8,0.2,0.5)
    
    def __init__(self, gscene, name, floor_height=None, *args, **kwargs):
        assert floor_height is not None, "floor_height needed"
        self.RTH_MIN = self.RTH_MIN[:2]+(floor_height,)
        self.H = 0.3
        ObstacleBase.__init__(self, gscene, name, *args, **kwargs)
        
    def is_overlapped_with(self, gtem):
        verts, radii = gtem.get_vertice_radius()
        verts_global = np.add(np.matmul(verts, gtem.orientation_mat.transpose()), gtem.center)
        verts_wp = np.multiply(DEFAULT_VERT_DICT[self.GTYPE], tuple(self.DIM[:2])+(self.H,))
        verts_wp_global = np.add(np.matmul(verts_wp, self.geometry.orientation_mat.transpose()), 
                                 np.add(self.geometry.center, (0,0,self.H/2)))
        return get_gjk_distance(get_point_list(verts_global), get_point_list(verts_wp_global))-radii < 1e-4
        
    
##
# @class Box
# @brief box with the top and the front side open
class Box(WorkPlane):
    RPY_MIN = (0, 0, -np.pi/2)
    RPY_MAX = (0, 0, +np.pi/2)
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.6, 0.6, 0.05)
    COLOR =  (0.8,0.8,0.2,0.5)
    H_RANGE = (0.3, 0.6)
    THICKNESS = 0.05
    def __init__(self, gscene, name, H=None, **kwargs):
        WorkPlane.__init__(self, gscene=gscene, name=name, **kwargs)
        self.H = np.random.uniform(*self.H_RANGE) if H is None else H

        ## back wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name+"_bw", link_name="base_link", 
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(self.DIM[0]/2+self.THICKNESS/2, 0,self.H/2), rpy=(0,0,0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## left wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name+"_lw", link_name="base_link", 
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, -self.DIM[1]/2-self.THICKNESS/2, self.H/2), rpy=(0,0,0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## right wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name+"_rw", link_name="base_link", 
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, self.DIM[1]/2+self.THICKNESS/2, self.H/2), rpy=(0,0,0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        
##
# @class SideBox
# @brief box with a side face open
class SideBox(Box):
    H_RANGE = (0.3, 0.6)
    def __init__(self, gscene, name, **kwargs):
        Box.__init__(self, gscene=gscene, name=name, **kwargs)

        ## top
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name+"_tp", link_name="base_link", 
            dims=(self.DIM[0], self.DIM[1], self.THICKNESS), center=(0, 0, self.H+self.THICKNESS/2), rpy=(0,0,0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

##
# @class TopBox
# @brief box with the top face open
class TopBox(Box):
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.6, 0.6, 0.05)
    H_RANGE = (0.3, 0.6)
    def __init__(self, gscene, name, **kwargs):
        Box.__init__(self, gscene=gscene, name=name, **kwargs)

        ## front wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name+"_fw", link_name="base_link", 
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(-self.DIM[0]/2-self.THICKNESS/2, 0,self.H/2), rpy=(0,0,0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))
        
##
# @class Floor
# @brief Floor - lowerbound of the workspace
class Floor(ObstacleBase):
    RTH_MIN = (0.0, 0, -0.5)
    RTH_MAX = (0.0, 0, +0.0)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (3, 3, 0.1)
    DIM_MAX = (3, 3, 0.1)
    GTYPE = GEOTYPE.BOX
    
##
# @class Ceiling
# @brief Ceiling - upperbound of the workspace
class Ceiling(ObstacleBase):
    RTH_MIN = (0.0, 0, 1.0)
    RTH_MAX = (0.0, 0, 2)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (3, 3, 0.1)
    DIM_MAX = (3, 3, 0.1)
    GTYPE = GEOTYPE.BOX
    COLOR=  (0.7,0.7,0.7,0.5)
    
##
# @class Wall
# @brief define horizontal boundary of the workspace
class Wall(ObstacleBase):
    RTH_MIN = (0.3, -np.pi, 0)
    RTH_MAX = (2.0, np.pi, 0)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.1, 6, 3)
    DIM_MAX = (0.1, 6, 3)
    GTYPE = GEOTYPE.BOX
    COLOR=  (0.7,0.7,0.7,0.5)
    
##
# @class Pole
# @brief occasional poles that obstruct robot motion
class Pole(ObstacleBase):
    RTH_MIN = (0.3, -np.pi, 0)
    RTH_MAX = (0.8, +np.pi, 0)
    RPY_MIN = (0, 0, -np.pi/6)
    RPY_MAX = (0, 0, +np.pi/6)
    DIM_MIN = (0.1, 0.1, 4)
    DIM_MAX = (0.3, 0.3, 4)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.7,0.7,0.7,0.3)
    
##
# @class Bar
# @brief occasional poles that obstruct robot motion
class Bar(ObstacleBase):
    RTH_MIN = (0.3, -np.pi, 0)
    RTH_MAX = (0.8, +np.pi, 1.5)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.1, 4, 0.1)
    DIM_MAX = (0.3, 4, 0.3)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.7,0.7,0.7,0.3)
    
##
# @class PlaneObstacle
# @brief Obstacles on the workplane
class PlaneObject(ObstacleBase):
    RTH_MIN = (0.3, -np.pi/2, -0.2)
    RTH_MAX = (0.8, +np.pi/2, +0.5)
    RPY_MIN = (0, 0, -np.pi)
    RPY_MAX = (0, 0, +np.pi)
    DIM_MAX = (0.06, 0.3, 0.3)
    GTYPE = GEOTYPE.BOX
    COLOR =  (0.2,0.2,0.8,0.5)
    def __init__(self, gscene, name, workplane, GRIP_DEPTH, CLEARANCE, XYZ_LOC=None, **kwargs):
        self.GRIP_DEPTH = GRIP_DEPTH
        self.DIM_MIN = (0.02, GRIP_DEPTH, GRIP_DEPTH)
        self.CLEARANCE = CLEARANCE
        ObstacleBase.__init__(self, gscene=gscene, name=name, **kwargs)
        verts, radii = self.geometry.get_vertice_radius()
        verts_rot = np.matmul(self.geometry.orientation_mat, verts.transpose()) ## verices with global orientaion
        verts_rot_loc = np.matmul(workplane.geometry.Toff[:3,:3].transpose(), verts_rot) ## verices with local orientaion
        max_verts = np.max(verts_rot_loc, axis=-1)
        min_verts = np.min(verts_rot_loc, axis=-1)
        if XYZ_LOC is None:
            self.XYZ_LOC = np.random.uniform(np.negative(workplane.DIM)/2-min_verts+radii,np.array(workplane.DIM)/2-max_verts-radii)
            self.XYZ_LOC[2] = workplane.DIM[2]/2 + self.DIM[2]/2 + CLEARANCE
        else:
            self.XYZ_LOC = self.XYZ_LOC
        self.XYZ = np.matmul(workplane.geometry.Toff[:3,:3], self.XYZ_LOC) + workplane.geometry.Toff[:3,3]
        self.geometry.set_offset_tf(center = self.XYZ)
        self.RTH = cart2cyl(*self.XYZ)
        gscene.update_marker(self.geometry)
        
        
def clear_class(gscene, key, Nmax):
    for iw in range(Nmax):
        gname = "{}_{}".format(key, iw)
        if gname in gscene.NAME_DICT:
            gscene.remove(gscene.NAME_DICT[gname])

            
def redistribute_class(gscene, obstacle_class, key, Nmax, workplane_avoid=None):
    clear_class(gscene, key, Nmax)
        
    obs_list = []
    for iw in range(np.random.choice(Nmax)):
        obs = obstacle_class(gscene, "{}_{}".format(key, iw))
        while workplane_avoid is not None and workplane_avoid.is_overlapped_with(obs.geometry):
            obs = obstacle_class(gscene, "{}_{}".format(key, iw))
        obs_list.append(obs)
    return obs_list

            
def disperse_objects(gscene, object_class, key, Nmax, workplane_on, GRIP_DEPTH, CLEARANCE=1e-3):
    clear_class(gscene, key, Nmax)
        
    obs_list = []
    for iw in range(np.random.choice(Nmax)+1):
        obs = object_class(gscene, "{}_{}".format(key, iw), workplane_on, GRIP_DEPTH=GRIP_DEPTH, CLEARANCE=CLEARANCE)
        remove_this = False
        for obs_pre in obs_list:
            if obs_pre.is_overlapped_with(obs.geometry):
                remove_this = True
                break
        if remove_this:
            gscene.remove(obs.geometry)
        else:
            obs_list.append(obs)
    return obs_list


def add_object(pscene, obj, HANDLE_THICKNESS=1e-6, HANDLE_COLOR = (1,0,0,0.3)):
    gscene = pscene.gscene
    GRIP_DEPTH = obj.GRIP_DEPTH
    if gscene.NAME_DICT[obj.name].gtype == GEOTYPE.BOX:
        handles = []
        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_tp_a", link_name="base_link",
                           dims=(obj.DIM[1], GRIP_DEPTH, HANDLE_THICKNESS), center=(0,0,obj.DIM[2]/2-GRIP_DEPTH/2), rpy=(np.pi/2,0,np.pi/2),
                               color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                           parent=obj.name)
        )

        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_tp_b", link_name="base_link",
                           dims=(obj.DIM[1], GRIP_DEPTH, HANDLE_THICKNESS), center=(0,0,obj.DIM[2]/2-GRIP_DEPTH/2), rpy=(np.pi/2,0,-np.pi/2),
                               color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                           parent=obj.name)
        )

        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_ft_a", link_name="base_link",
                           dims=(obj.DIM[2], GRIP_DEPTH,HANDLE_THICKNESS), center=(0,obj.DIM[1]/2-GRIP_DEPTH/2,0), rpy=(0,np.pi/2,0),
                               color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                           parent=obj.name)
        )

        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_ft_b", link_name="base_link",
                           dims=(obj.DIM[2], GRIP_DEPTH,HANDLE_THICKNESS), center=(0,obj.DIM[1]/2-GRIP_DEPTH/2,0), rpy=(0,-np.pi/2,0),
                               color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                           parent=obj.name)
        )

        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_bk_a", link_name="base_link",
                           dims=(obj.DIM[2], GRIP_DEPTH,HANDLE_THICKNESS), center=(0,-obj.DIM[1]/2+GRIP_DEPTH/2,0), rpy=(-np.pi,-np.pi/2,0),
                           color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                       parent=obj.name)
        )

        handles.append(
            gscene.create_safe(gtype=GEOTYPE.BOX, name=obj.name+"_hdl_bk_b", link_name="base_link",
                           dims=(obj.DIM[2], GRIP_DEPTH,HANDLE_THICKNESS), center=(0,-obj.DIM[1]/2+GRIP_DEPTH/2,0), rpy=(-np.pi,+np.pi/2,0),
                               color=HANDLE_COLOR, display=True, collision=False, fixed=False,
                           parent=obj.name)
        )

        action_points_dict = {obj.name+"_placement": PlacePoint(obj.name+"_placement", obj.geometry, [0,0,-obj.DIM[2]/2-obj.CLEARANCE], [0,0,0])}
        action_points_dict.update({handle.name: Grasp2Point(handle.name, handle, None, (0,0,0)) for handle in handles})
        obj_pscene = pscene.create_subject(oname=obj.name, gname=obj.name, _type=CustomObject,
                                     action_points_dict=action_points_dict)
    elif gscene.NAME_DICT[obj.name].gtype == GEOTYPE.CYLINDER:
            obj_pscene = pscene.create_subject(oname=obj.name, gname=obj.name, _type=CylinderObject)
            handles = sorted([v for k, v in obj_pscene.action_points_dict.items() if k not in ["top_p", "bottom_p"]], key=lambda x: x.name)
    return obj_pscene, handles

## Done