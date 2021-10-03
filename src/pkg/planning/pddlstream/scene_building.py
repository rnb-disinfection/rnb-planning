from __future__ import print_function
import os

from ..constraint.constraint_subject import CustomObject, Grasp2Point, PlacePoint, SweepPoint, SweepTask
from ..filtering.lattice_model.scene_building import *


##
# @class WorkPlane
# @brief working plane. target and obstacle objects are generated on this plane
class WorkPlane(ObstacleBase):
    RTH_MIN = (0.3, -np.pi / 2, -0.1)
    RTH_MAX = (0.5, +np.pi / 2, +0.4)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.4, 0.6, 0.05)
    DIM_MAX = (0.4, 0.6, 0.05)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.8, 0.8, 0.2, 1)
    THICKNESS = 0.05

    def __init__(self, gscene, name, floor_height=None, edge_margin=0.05, *args, **kwargs):
        assert floor_height is not None, "floor_height needed"
        if floor_height > self.RTH_MIN[2]:
            self.RTH_MIN = self.RTH_MIN[:2] + (floor_height,)
        self.H = 0.4
        ObstacleBase.__init__(self, gscene, name, *args, **kwargs)
        if edge_margin > 1e-6:
            gtem = gscene.NAME_DICT[name]
            gtem_new = gscene.copy_from(gtem, new_name=name + "_bot")
            gtem.set_dims(np.subtract(gtem_new.dims, (edge_margin * 2, edge_margin * 2, 0)))
            gscene.update_marker(gtem)
            gscene.update_marker(gtem_new)

    def is_overlapped_with(self, gtem, margin=1e-4):
        verts, radii = gtem.get_vertice_radius()
        verts_global = np.add(np.matmul(verts, gtem.orientation_mat.transpose()), gtem.center)
        verts_wp = np.multiply(DEFAULT_VERT_DICT[self.GTYPE],
                               tuple(np.add(self.DIM[:2], 0.2)) + (self.H + 0.1,))
        verts_wp_global = np.add(np.matmul(verts_wp, self.geometry.orientation_mat.transpose()),
                                 np.add(self.geometry.center, (0, 0, self.H / 2)))
        return get_gjk_distance(get_point_list(verts_global), get_point_list(verts_wp_global)) - radii < margin


##
# @class Box
# @brief box with the top and the front side open
class Box(WorkPlane):
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (0.3, 0.3, 0.05)
    DIM_MAX = (0.6, 0.6, 0.05)
    COLOR = (0.8, 0.8, 0.2, 0.5)
    H_RANGE = (0.3, 0.6)

    def __init__(self, gscene, name, H=None, **kwargs):
        WorkPlane.__init__(self, gscene=gscene, name=name, **kwargs)
        self.H = np.random.uniform(*self.H_RANGE) if H is None else H

        ## back wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_bw", link_name="base_link",
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(self.DIM[0] / 2 + self.THICKNESS / 2, 0, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## left wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_lw", link_name="base_link",
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, -self.DIM[1] / 2 - self.THICKNESS / 2, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))

        ## right wall
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_rw", link_name="base_link",
            dims=(self.DIM[0], self.THICKNESS, self.H), center=(0, self.DIM[1] / 2 + self.THICKNESS / 2, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))


##
# @class SideBox
# @brief box with a side face open
class SideBox(Box):
    RTH_MIN = (0.3, -np.pi / 2, -0.1)
    RTH_MAX = (0.5, +np.pi / 2, +0.4)
    RPY_MIN = (0, 0, np.pi / 6)
    RPY_MAX = (0, 0, np.pi / 6)
    DIM_MIN = (0.4, 0.6, 0.05)
    DIM_MAX = (0.4, 0.6, 0.05)
    COLOR = (0.2, 0.2, 0.8, 1)
    H_RANGE = (0.4, 0.4)

    def __init__(self, gscene, name, **kwargs):
        Box.__init__(self, gscene=gscene, name=name, **kwargs)

        ## top
        self.subgeo_list.append(gscene.create_safe(
            gtype=self.GTYPE, name=self.name + "_tp", link_name="base_link",
            dims=(self.DIM[0], self.DIM[1], self.THICKNESS), center=(0, 0, self.H + self.THICKNESS / 2), rpy=(0, 0, 0),
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
            gtype=self.GTYPE, name=self.name + "_fw", link_name="base_link",
            dims=(self.THICKNESS, self.DIM[1], self.H), center=(-self.DIM[0] / 2 - self.THICKNESS / 2, 0, self.H / 2),
            rpy=(0, 0, 0),
            color=self.COLOR, display=True, collision=True, fixed=True,
            parent=self.name))


##
# @class Floor
# @brief Floor - lowerbound of the workspace
class Floor(ObstacleBase):
    RTH_MIN = (0.0, 0, -0.5)
    RTH_MAX = (0.0, 0, -0.5)
    RPY_MIN = (0, 0, 0)
    RPY_MAX = (0, 0, 0)
    DIM_MIN = (3, 3, 0.1)
    DIM_MAX = (3, 3, 0.1)
    GTYPE = GEOTYPE.BOX


##
# @class Pole
# @brief occasional poles that obstruct robot motion
class Pole(ObstacleBase):
    RTH_MIN = (0.3, -np.pi, 0)
    RTH_MAX = (0.3, +np.pi, 0)
    RPY_MIN = (0, 0, -np.pi / 6)
    RPY_MAX = (0, 0, +np.pi / 6)
    DIM_MIN = (0.1, 0.1, 4)
    DIM_MAX = (0.1, 0.1, 4)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.7, 0.7, 0.7, 0.3)


##
# @class PlaneObstacle
# @brief Obstacles on the workplane
class PlaneObject(ObstacleBase):
    RTH_MIN = (0.3, -np.pi / 2, -0.2)
    RTH_MAX = (0.8, +np.pi / 2, +0.5)
    RPY_MIN = (0, 0, -np.pi)
    RPY_MAX = (0, 0, +np.pi)
    DIM_MIN = (0.04, 0.1, 0.2)
    DIM_MAX = (0.04, 0.1, 0.2)
    GTYPE = GEOTYPE.BOX
    COLOR = (0.2, 0.8, 0.2, 1)

    def __init__(self, gscene, name, workplane, GRIP_DEPTH, CLEARANCE, XYZ_LOC=None, WALL_MARGIN=0.0, **kwargs):
        self.GRIP_DEPTH = GRIP_DEPTH
        ObstacleBase.__init__(self, gscene=gscene, name=name, fixed=False, **kwargs)
        verts, radii = self.geometry.get_vertice_radius()
        verts_rot = np.matmul(self.geometry.orientation_mat, verts.transpose())  ## verices with global orientaion
        verts_rot_loc = np.matmul(workplane.geometry.Toff[:3, :3].transpose(),
                                  verts_rot)  ## verices with local orientaion
        max_verts = np.max(verts_rot_loc, axis=-1)
        min_verts = np.min(verts_rot_loc, axis=-1)
        if XYZ_LOC is None:
            self.XYZ_LOC = np.random.uniform(np.negative(workplane.DIM) / 2 - min_verts + radii + WALL_MARGIN,
                                             np.array(workplane.DIM) / 2 - max_verts - radii - WALL_MARGIN)
            self.XYZ_LOC[2] = workplane.DIM[2] / 2 + self.DIM[2] / 2 + CLEARANCE
        else:
            self.XYZ_LOC = self.XYZ_LOC
        self.XYZ = np.matmul(workplane.geometry.Toff[:3, :3], self.XYZ_LOC) + workplane.geometry.Toff[:3, 3]
        self.geometry.set_offset_tf(center=self.XYZ)
        self.RTH = cart2cyl(*self.XYZ)
        gscene.update_marker(self.geometry)


def disperse_objects(gscene, object_class, key, Nmax, workplane_on, GRIP_DEPTH, CLEARANCE=1e-3):
    clear_class(gscene, key, Nmax)

    obs_list = []
    while len(obs_list) < Nmax:
        iw = len(obs_list)
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


##
# @class PlaneObstacle
# @brief Obstacles on the workplane
class PlaneObjectBig(ObstacleBase):
    RTH_MIN = (0.3, -np.pi/2, -0.2)
    RTH_MAX = (0.8, +np.pi/2, +0.5)
    RPY_MIN = (0, 0, -np.pi)
    RPY_MAX = (0, 0, +np.pi)
    DIM_MIN = (0.2, 0.2, 0.4)
    DIM_MAX = (0.2, 0.2, 0.4)
    GTYPE = GEOTYPE.CYLINDER
    COLOR =  (0.8,0.8,0.2,1)
    def __init__(self, gscene, name, workplane, GRIP_DEPTH, CLEARANCE, XYZ_LOC=None, WALL_MARGIN=0.04, **kwargs):
        self.GRIP_DEPTH = GRIP_DEPTH
        ObstacleBase.__init__(self, gscene=gscene, name=name, fixed=False, **kwargs)
        verts, radii = self.geometry.get_vertice_radius()
        verts_rot = np.matmul(self.geometry.orientation_mat, verts.transpose()) ## verices with global orientaion
        verts_rot_loc = np.matmul(workplane.geometry.Toff[:3,:3].transpose(), verts_rot) ## verices with local orientaion
        max_verts = np.max(verts_rot_loc, axis=-1)
        min_verts = np.min(verts_rot_loc, axis=-1)
        if XYZ_LOC is None:
            self.XYZ_LOC = np.random.uniform(np.negative(workplane.DIM)/2-min_verts+radii+WALL_MARGIN,
                                             np.array(workplane.DIM)/2-max_verts-radii-WALL_MARGIN)
            self.XYZ_LOC[2] = workplane.DIM[2]/2 + self.DIM[2]/2 + CLEARANCE
        else:
            self.XYZ_LOC = self.XYZ_LOC
        self.XYZ = np.matmul(workplane.geometry.Toff[:3,:3], self.XYZ_LOC) + workplane.geometry.Toff[:3,3]
        self.geometry.set_offset_tf(center = self.XYZ)
        self.RTH = cart2cyl(*self.XYZ)
        gscene.update_marker(self.geometry)