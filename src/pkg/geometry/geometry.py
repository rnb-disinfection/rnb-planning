from __future__ import print_function

from .ros_rviz import show_motion, get_markers, get_publisher
from .geotype import GEOTYPE
from ..utils.rotation_utils import *
from ..utils.joint_utils import get_tf, get_link_adjacency_map, get_min_distance_map, get_link_control_dict
from ..utils.utils import list2dict, dict2list, TextColors
from collections import defaultdict
from copy import deepcopy

POINT_DEFAULT = np.array([[0,0,0]])
SEG_DEFAULT = np.array([[0,0,1.0],[0,0,-1.0]])/2
N_CYL = 10
CYL_DEFAULT = np.reshape(np.array([[np.cos(float(i_theta)/N_CYL*np.pi*2)/2, np.sin(float(i_theta)/N_CYL*np.pi*2)/2, 0] for i_theta in range(N_CYL)])[:,np.newaxis,:]
                         + (np.array([[0,0,1.0],[0,0,-1.0]])/2)[np.newaxis,:,:], newshape=(-1,3))
BOX_DEFAULT = np.array([[[(i,j,k) for k in range(2)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5
PLANE_DEFAULT = np.array([[[(i,j,0)] for j in range(2)] for i in range(2)], dtype=np.float).reshape((-1,3))-0.5

DEFAULT_VERT_DICT = {
    GEOTYPE.SPHERE: POINT_DEFAULT,
    GEOTYPE.CAPSULE: SEG_DEFAULT,
    GEOTYPE.CYLINDER: CYL_DEFAULT,
    GEOTYPE.BOX: BOX_DEFAULT,
    GEOTYPE.MESH: BOX_DEFAULT,
    GEOTYPE.PLANE: PLANE_DEFAULT
}

##
# @class GeometryScene
# @brief Geometric scene & visualization handle. Also a list of GeometryItem.
class GeometryScene(list):
    COLOR_LIST = [(1,0,0,0.5), (1,0,1,0.5), (0.7,0.7,0,0.5), (0,1,0,0.5), (0,1,1,0.5), (0,0,1,0.5), (0,0,0,0.5)]

    def __init__(self, urdf_content, urdf_path, joint_names, link_names, rviz=True):
        self.NAME_DICT = {}
        self.joint_names, self.link_names = joint_names, link_names
        self.joint_num = len(self.joint_names)
        self.__set_urdf(urdf_content, urdf_path)
        self.rviz = rviz
        self.highlight_dict = defaultdict(dict)
        self.marker_dict = defaultdict(list)
        self.marker_dict_fixed = defaultdict(list)
        self.virtuals = []
        if self.rviz:
            self.set_rviz()

    def __set_urdf(self, urdf_content, urdf_path):
        self.urdf_content = urdf_content
        self.urdf_path = urdf_path
        self.link_adjacency_map, self.link_adjacency_map_ext = get_link_adjacency_map(urdf_content)
        self.fixed_link_adjacency_map, _ = get_link_adjacency_map(urdf_content, fixed_only=True)
        self.link_control_map = get_link_control_dict(urdf_content)
        self.min_distance_map = get_min_distance_map(urdf_content)

    ##
    # @brief append new geometry, NEVER CALL EXPLICITLY: automatically called by constructor of geometry.
    def append(self, geo):
        if geo.name in self.NAME_DICT:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!"+" geometry name already registered - {} ".format(geo.name)+"!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        assert geo.name not in self.NAME_DICT, "geometry name already registered - {}".format(geo.name)
        list.append(self, geo)
        self.NAME_DICT[geo.name] = geo
        if self.rviz:
            self.__add_marker(geo)

    ##
    # @brief clear GeometryScene
    def clear(self):
        for x in self:
            self.remove(x)

    ##
    # @brief clear GeometryItems on specific link
    def clear_link(self, link_name):
        names_to_remove = [gtem.name for gtem in self if gtem.link_name == link_name and gtem.parent is None]
        for gname in names_to_remove:
            try:
                self.remove(self.NAME_DICT[gname])
            except Exception as e:
                print(e)

    ##
    # @brief remove one item from handle
    def remove(self, geo, call_from_parent=False):
        for child in geo.children:
            try: self.remove(self.NAME_DICT[child], call_from_parent=True)
            except Exception as e: print("ERROR: remove gtem - {}".format(e))
        if not call_from_parent:
            if geo.parent is not None:
                self.NAME_DICT[geo.parent].children.remove(geo.name)
        if self.rviz:
            self.__remove_marker(geo)
        list.remove(self, geo)
        del self.NAME_DICT[geo.name]

    ##
    # @brief update fixed/movable flags
    def update(self):
        self.fixed_gtems = [ctem for ctem in self if ctem.fixed]
        self.movable_gtems = [ctem for ctem in self if not ctem.fixed]
        self.fixed_ctems = [ctem for ctem in self.fixed_gtems if ctem.collision]
        self.movable_ctems = [ctem for ctem in self.movable_gtems if ctem.collision]

    ##
    # @brief safely create or get geometry item. If same name exist, re-initialize the existing item.
    # @param gtype GEOTYPE
    # @param name name of geometry
    # @param link_name attached link name
    # @param dims dimension of geometry in m scale
    # @param center center offset from attached link coordinate
    # @param rpy orientation respect to attached link coordinate, in radian rpy
    # @param color color of object, RGBA, 0~1
    # @param display flag for showing the object
    # @param collision flag for collision checking
    # @param fixed flag that the object is fixed to the attached link (transferring is not considered)
    def create_safe(self, gtype, name, link_name, dims, center, rpy=(0,0,0), color=(0,1,0,1), display=True,
                    collision=True, fixed=False, **kwargs):
        if isinstance(gtype, str):
            gtype = getattr(GEOTYPE, gtype)
        if name in self.NAME_DICT:
            gtem = self.NAME_DICT[name]
            self.remove(gtem)
        #     gtem.__init__(self, gtype=gtype, name=name, link_name=link_name,
        #                   center=center, rpy=rpy, dims=dims,
        #                   color=color, display=display, collision=collision, fixed=fixed,
        #                   create=False, **kwargs)
        # else:
        gtem = GeometryItem(self,  gtype=gtype, name=name, link_name=link_name,
                      center=center, rpy=rpy, dims=dims,
                      color=color, display=display, collision=collision, fixed=fixed,
                      create=True, **kwargs)
        return gtem

    def display_text(self, name, msg, loc, link_name="base_link", scale=0.2, color=(0,0,0,1)):
        scale = scale if isinstance(scale, tuple) else (scale, )*3
        return self.create_safe(GEOTYPE.TEXT, name, link_name=link_name, dims=(0.1, 0.1, 0.1),
                                  scale=scale, center=loc, rpy=(0, 0, 0),
                                  text=msg, color=color, collision=False, fixed=True)

    ##
    # @brief copy from existing geometry item
    # @param new_name new name
    # @param color specify color to change color
    # @param display specify display flag to change
    # @param collision specify collision flag to change
    def copy_from(self, gtem, new_name=None, color=None, display=None, collision=None):
        color = gtem.color if color is None else color
        display = gtem.display if display is None else display
        collision = gtem.collision if collision is None else collision
        new_name = gtem.name if new_name is None else new_name
        return self.create_safe(name=new_name, link_name=gtem.link_name, gtype=gtem.gtype,
                                center=gtem.center, rpy=gtem.rpy, dims=gtem.dims,
                                color=color, display=display, collision=collision, fixed=gtem.fixed)

    ##
    # @brief clear all non-fixed items
    def clear_non_fixed(self):
        gnames = sorted(self.NAME_DICT.keys())
        for gname in gnames :
            if gname in self.NAME_DICT:
                gtem = self.NAME_DICT[gname]
                if gtem.parent is None and gtem.fixed == False:
                    self.remove(gtem)

    ##
    # @brief initialize rviz
    def set_rviz(self, joint_pose=None):
        # prepare ros
        if not (hasattr(self, 'pub') and hasattr(self, 'joints') and hasattr(self, 'rate')):
            self.pub, self.joints, self.rate = get_publisher(self.joint_names)
        if joint_pose is None:
            joint_pose = self.joints.position
        # prepare visualization markers
        self.__clear_markers()
        self.marker_dict_fixed = get_markers([gtem for gtem in self if gtem.fixed_on_world], self.joints, self.joint_names)
        self.marker_dict = get_markers([gtem for gtem in self if not gtem.fixed_on_world], self.joints, self.joint_names)
        self.show_pose(joint_pose)

    ##
    # @brief add new marker (for internal use)
    def __add_marker(self, gtem):
        if self.rviz:
            markers = get_markers([gtem], self.joints, self.joint_names)
            if gtem.fixed_on_world:
                self.marker_dict_fixed.update(markers)
            else:
                self.marker_dict.update(markers)

    ##
    # @brief republish markers with last published position
    def update_marker(self, gtem):
        if self.rviz:
            marker_dict = self.marker_dict_fixed if gtem.fixed_on_world else self.marker_dict
            joint_dict = {self.joints.name[i]: self.joints.position[i] for i in range(len(self.joint_names))}
            marks = marker_dict[gtem.name]
            if gtem.display and len(marks)==0:
                self.__add_marker(gtem)
            else:
                for mk in marks:
                    if gtem.display:
                        mk.set_marker(joint_dict, create=False)
                    else:
                        self.__remove_marker(gtem)
            return marks

    ##
    # @brief republish all markers with last published position
    def update_markers_all(self, include_fixed=True):
        if self.rviz:
            joint_dict = {self.joints.name[i]: self.joints.position[i] for i in range(len(self.joint_names))}
            if include_fixed:
                for mks in self.marker_dict_fixed.values():
                    for mk in mks:
                        mk.set_marker(joint_dict, create=False)
            for mks in self.marker_dict.values():
                for mk in mks:
                    mk.set_marker(joint_dict, create=False)

    ##
    # @brief remove marker for specific geometry item
    def __remove_marker(self, gtem, sleep=True):
        marker_dict = self.marker_dict_fixed if gtem.fixed_on_world else self.marker_dict
        if gtem.name in marker_dict:
            for marker in marker_dict[gtem.name]:
                marker.delete(sleep=sleep)
            if gtem.name in self.marker_dict_fixed:
                del self.marker_dict_fixed[gtem.name]
            if gtem.name in self.marker_dict:
                del self.marker_dict[gtem.name]

    ##
    # @brief clear all markers
    def __clear_markers(self, include_fixed=True):
        for hl_key in self.highlight_dict.keys():
            self.clear_highlight(hl_key)
        if include_fixed:
            for mks in self.marker_dict_fixed.values():
                for mk in mks:
                    mk.delete()
            self.marker_dict_fixed = defaultdict(list)
        for mks in self.marker_dict.values():
            for mk in mks:
                mk.delete()
        self.marker_dict = defaultdict(list)

    ##
    # @brief show pose
    # @param pose Q in radian numpy array
    def show_pose(self, pose, **kwargs):
        self.show_motion([pose], **kwargs)

    ##
    # @brief show motion list
    # @param pose_list list of Q in radian numpy array
    def show_motion(self, pose_list, period=0.01, **kwargs):
        if self.rviz:
            marker_list = []
            for mks in self.marker_dict.values():
                marker_list += mks
            if period<0.01:
                pose_list = pose_list[::int(0.01/period)]
                period = 0.01
            show_motion(pose_list, marker_list, self.pub, self.joints, self.joint_names, period=period, **kwargs)

    ##
    # @brief clear all highlights
    def clear_highlight(self, hl_keys=[], sleep=True):
        for hl_key, hl_set in self.highlight_dict.items():
            if hl_key in hl_keys or not hl_keys:
                for k in deepcopy(sorted(hl_set.keys())):
                    v = self.highlight_dict[hl_key][k]
                    del self.highlight_dict[hl_key][k]
                    if v in self:
                        self.remove(v)
                del self.highlight_dict[hl_key]

    ##
    # @brief highlight a geometry
    def highlight_geometry(self, hl_key, gname, color=(1, 0.3, 0.3, 0.5)):
        if gname not in self.NAME_DICT:
            return
        gtem = self.NAME_DICT[gname]
        dims = gtem.dims if np.sum(gtem.dims) > 0.001 else (0.03, 0.03, 0.03)
        hname = "hl_" + gtem.name
        if hname in self.NAME_DICT:
            return
        htem = self.create_safe(gtype=gtem.gtype, name=hname, link_name=gtem.link_name,
                            center=gtem.center, dims=dims, rpy=Rot2rpy(gtem.orientation_mat), color=color,
                            collision=False)

        self.highlight_dict[hl_key][htem.name] = htem
        if self.rviz:
            self.__add_marker(htem)
        self.add_highlight_axis(hname, "axis", gtem.link_name, center=gtem.center, orientation_mat=gtem.orientation_mat)

    ##
    # @param points (Nx3)
    def show_point_cloud(self, points, name="cloud", link_name="base_link", sample_to=10000, point_size=0.01,
                         center=(0,0,0), rpy=(0,0,0), color=(0, 1, 0, 0.5), collision=False, **kwargs):
        sample_step = int(len(points) / sample_to)
        if sample_step == 0:
            sample_step = 1
        points = points[::sample_step]
        gtem = self.create_safe(gtype=GEOTYPE.MESH, name=name, link_name=link_name,
                                center=center, rpy=rpy, dims=(0.1,)*3, color=color, collision=collision,
                                vertices=points, scale=(point_size,point_size,1), **kwargs)
        return gtem

    ##
    # @brief add highlight axis
    def add_highlight_axis(self, hl_key, name, link_name="base_link",
                           center=None, orientation_mat=None, T=None,
                           color=None, axis="xyz", dims=(0.10, 0.01, 0.01)):
        assert center is not None or T is not None, "Either center, orientation or T should be given"
        if center is None:
            center, orientation_mat = T[:3,3], T[:3,:3]
        elif orientation_mat is None:
            orientation_mat = np.identity(3)
            
        if axis == '' or axis == None:
            ctem = self.create_safe(gtype=GEOTYPE.SPHERE, name="cp_" + name, link_name=link_name,
                                  center=center, dims=(min(dims),)*3, rpy=(0,0,0),
                                  color=(0, 0, 0, 0.5) if color is None else color,
                                  collision=False)
            if self.rviz:
                self.__add_marker(ctem)
            self.highlight_dict[hl_key][ctem.name] = ctem
            return

        if 'x' in axis:
            axtemx = self.create_safe(gtype=GEOTYPE.ARROW, name="axx_" + name, link_name=link_name,
                                  center=center, dims=dims, rpy=Rot2rpy(orientation_mat), color=(1, 0, 0, 0.5) if color is None else color,
                                  collision=False)
            if self.rviz:
                self.__add_marker(axtemx)
            self.highlight_dict[hl_key][axtemx.name] = axtemx

        if 'y' in axis:
            axtemy = self.create_safe(gtype=GEOTYPE.ARROW, name="axy_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(3, np.pi / 2))), color=(0, 1, 0, 0.5) if color is None else color,
                                  collision=False)
            if self.rviz:
                self.__add_marker(axtemy)
            self.highlight_dict[hl_key][axtemy.name] = axtemy

        if 'z' in axis:
            axtemz = self.create_safe(gtype=GEOTYPE.ARROW, name="axz_" + name, link_name=link_name,
                                  center=center, dims=dims,
                                  rpy=Rot2rpy(np.matmul(orientation_mat, Rot_axis(2, -np.pi / 2))),
                                  color=(0, 0, 1, 0.5) if color is None else color,
                                  collision=False)
            if self.rviz:
                self.__add_marker(axtemz)
            self.highlight_dict[hl_key][axtemz.name] = axtemz

    ##
    # @brief get geometries attched to specific links
    # @param links list of link names
    def get_items_on_links(self, links, colliding_only=True):
        return [gtem for gtem in self if gtem.link_name in links if gtem.collision or not colliding_only]

    ##
    # @brief set workspace boundary
    def set_workspace_boundary(self, XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX, thickness=0.01):
        self.create_safe(GEOTYPE.BOX, "ceiling_ws", "base_link", (XMAX - XMIN, YMAX - YMIN, thickness),
                         ((XMAX + XMIN) / 2, (YMAX + YMIN) / 2, ZMAX+thickness/2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "floor_ws", "base_link", (XMAX - XMIN, YMAX - YMIN, thickness),
                         ((XMAX + XMIN) / 2, (YMAX + YMIN) / 2, ZMIN-thickness/2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "frontwall_ws", "base_link", (thickness, YMAX - YMIN, ZMAX - ZMIN),
                         (XMAX+thickness/2, (YMAX + YMIN) / 2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "backwall_ws", "base_link", (thickness, YMAX - YMIN, ZMAX - ZMIN),
                         (XMIN-thickness/2, (YMAX + YMIN) / 2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "leftwall_ws", "base_link", (XMAX - XMIN, thickness, ZMAX - ZMIN),
                         ((XMAX + XMIN) / 2, YMIN-thickness/2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "rightwall_ws", "base_link", (XMAX - XMIN, thickness, ZMAX - ZMIN),
                         ((XMAX + XMIN) / 2, YMAX+thickness/2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=True)
        self.create_safe(GEOTYPE.BOX, "room_ws", "base_link", (XMAX - XMIN, YMAX - YMIN, ZMAX - ZMIN),
                         ((XMAX + XMIN) / 2, (YMAX + YMIN) / 2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                         color=(0.8, 0.8, 0.8, 0.1), display=True, fixed=True, collision=False)

    def clear_virtuals(self):
        for virtual in self.virtuals:
            try:
                self.remove(virtual)
            except:
                pass
        self.virtuals = []

    ##
    # @brief set workspace boundary
    def add_virtual_guardrail(self, plane_gtem, THICKNESS=0.01, HEIGHT=0.05, margin=0.01,
                              axis="y", color=(0.8, 0.8, 0.8, 0.1)):
        gname = plane_gtem.name
        dims = plane_gtem.dims
        XMIN, XMAX, YMIN, YMAX, ZMIN, ZMAX = -dims[0]/2, dims[0]/2, -dims[1]/2, dims[1]/2, -HEIGHT, HEIGHT
        if "x" in axis.lower() and "y" in axis.lower():
            add_edge = THICKNESS + margin * 2
        else:
            add_edge = 0
        if "x" in axis.lower():
            gtem0 = self.create_safe(GEOTYPE.BOX, "{}_front_gr".format(gname), "base_link",
                                     (THICKNESS, YMAX - YMIN + add_edge, ZMAX - ZMIN + add_edge),
                             (XMAX + margin, (YMAX + YMIN) / 2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                             color=color, display=True, fixed=True, collision=True, parent=gname)
            gtem1 = self.create_safe(GEOTYPE.BOX, "{}_back_gr".format(gname), "base_link",
                                     (THICKNESS, YMAX - YMIN + add_edge, ZMAX - ZMIN + add_edge),
                             (XMIN - margin, (YMAX + YMIN) / 2, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                             color=color, display=True, fixed=True, collision=True, parent=gname)
            virtuals = []
            for virtual in self.virtuals:
                if virtual.name not in [gtem0.name, gtem1.name]:
                    virtuals.append(virtual)
            self.virtuals = virtuals + [gtem0, gtem1]
        if "y" in axis.lower():
            gtem0 = self.create_safe(GEOTYPE.BOX, "{}_left_gr".format(gname), "base_link",
                                     (XMAX - XMIN + add_edge, THICKNESS, ZMAX - ZMIN + add_edge),
                             ((XMAX + XMIN) / 2, YMIN - margin, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                             color=color, display=True, fixed=True, collision=True, parent=gname)
            gtem1 = self.create_safe(GEOTYPE.BOX, "{}_right_gr".format(gname), "base_link",
                                     (XMAX - XMIN + add_edge, THICKNESS, ZMAX - ZMIN + add_edge),
                             ((XMAX + XMIN) / 2, YMAX + margin, (ZMAX + ZMIN) / 2), rpy=(0, 0, 0),
                             color=color, display=True, fixed=True, collision=True, parent=gname)
            virtuals = []
            for virtual in self.virtuals:
                if virtual.name not in [gtem0.name, gtem1.name]:
                    virtuals.append(virtual)
            self.virtuals = virtuals + [gtem0, gtem1]

    def highlight_robot(self, color=(1,0,0,0.5)):
        vis_bak = {}
        for gtem in self:
            if gtem.link_name != "base_link":
                vis_bak[gtem.name] = (gtem.display, gtem.color)
                gtem.display = True
                gtem.color = color
                self.update_marker(gtem)
        return vis_bak

    def recover_robot(self, vis_bak):
        for gname, (display, color) in vis_bak.items():
            gtem = self.NAME_DICT[gname]
            gtem.display = display
            gtem.color = color
            self.update_marker(gtem)

    def get_gtem_args(self, condition=lambda gtem:True):
        gtem_args = []
        for gtem in self:
            if condition(gtem):
                gtem_args.append(deepcopy(gtem.get_args()))
        return gtem_args

    def get_tf(self, to_link, Q, from_link="base_link"):
        Q_dict = Q if isinstance(Q, dict) else list2dict(Q, self.joint_names)
        return get_tf(to_link=to_link, joint_dict=Q_dict,
                      urdf_content=self.urdf_content, from_link=from_link)

    ##
    # @brief calculate jacobian for a geometry movement
    # @param gtem   GeometryItem
    # @param Q      current joint pose as array
    # @param ref_link reference link to calculate pose
    def get_jacobian(self, link_name, Q, Toff=np.identity(4), ref_link="base_link"):
        Q_dict = list2dict(Q, self.joint_names)
        Jac = []
        chain = self.urdf_content.get_chain(root=ref_link, tip=link_name)
        T_p = np.matmul(self.get_tf(link_name, Q_dict, ref_link), Toff)
        for ij, jname in enumerate(self.joint_names):
            if jname not in chain:
                Jac.append(np.zeros(6))
                continue
            T_bj, zi = self.get_joint_tf(jname, Q_dict, ref_link)
            dpi = T_p[:3, 3] - T_bj[:3, 3]
            zp = np.cross(zi, dpi)
            Ji = np.concatenate([zp, zi])
            Jac.append(Ji)
        Jac = np.array(Jac).transpose()
        return Jac

    ##
    # @param dVW twist on base coord = concat(w, v)
    def get_joint_increment(self, link_name, Q0, twist, Toff=np.identity(4), ref_link="base_link", Jac=None, Tcur=None):
        if Jac is None:
            Jac = self.get_jacobian(link_name, Q0, Toff=Toff, ref_link=ref_link)
        if Tcur is None:
            Tcur = np.matmul(self.get_tf(link_name, Q0, ref_link), Toff)
        Jinv = np.linalg.pinv(Jac)
        dQ = np.matmul(Jinv, np.asarray(twist)[[3, 4, 5, 0, 1, 2]])
        return np.add(Q0, dQ)

    def get_children_links(self, link_name):
        if link_name in self.urdf_content.child_map:
            children = map(lambda x: x[1], self.urdf_content.child_map[link_name])
            for child in children:
                children += self.get_children_links(child)
            return sorted(set(children))
        else:
            return []

    def get_joint_tf(self, jname, Q_dict, ref_link="base_link"):
        joint = self.urdf_content.joint_map[jname]
        Tj = T_xyzrpy((joint.origin.xyz, joint.origin.rpy))
        T_link = self.get_tf(joint.parent, Q_dict, from_link=ref_link)
        T_bj = np.matmul(T_link, Tj)
        zi = np.matmul(T_bj[:3, :3], joint.axis)
        return T_bj, zi

    def show_vector(self, hl_key, name, origin, vector, link_name="base_link", color=(1,0,0,1)):
        vector_len = np.linalg.norm(vector)
        vector_nm = vector / (vector_len + 1e-6)
        rotvec = calc_rotvec_vecs([0, 0, 1], vector_nm)
        R = Rotation.from_rotvec(rotvec).as_dcm()
        return self.add_highlight_axis(hl_key, name, link_name=link_name,
                                       center=origin, orientation_mat=R,
                                       axis="z", color=color, dims=(vector_len, vector_len*0.1, vector_len*0.1))

    def show_wrench(self, name, wrench, Tbp, fscale=0.01, tscale=0.5, hl_key="wc", link_name="base_link"):
        v, w = wrench[:3]*fscale, wrench[3:]*tscale
        Rbp, Pbp = Tbp[:3,:3], Tbp[:3,3]
        self.show_vector(hl_key, name+"_v", origin=Pbp,
                         vector=np.matmul(Rbp, v), color=(1,0,0,1), link_name=link_name)
        self.show_vector(hl_key, name+"_w", origin=Pbp,
                         vector=np.matmul(Rbp, w), color=(0,0,1,1), link_name=link_name)

##
# @class GeometryItem
# @brief Instance of geometry item
class GeometryItem(object):
    ##
    # @brief create geometry item. use GeometryScene.create_safe instead.
    # @param gtype rnb-planning.src.pkg.geometry.geotype.GEOTYPE
    # @param name name of geometry
    # @param link_name attached link name
    # @param dims dimension of geometry in m scale
    # @param center center offset from attached link coordinate, in the case of child geometry, offset from the parent
    # @param rpy roll-pitch-yaw orientation respect to attached link coordinate in radian, in the case of child geometry, offset from the parent
    # @param color color of object, RGBA, 0~1
    # @param display flag for showing the object
    # @param collision flag for collision checking
    # @param fixed flag that the object is fixed to the attached link (transferring is not considered)
    # @param online flag that the object should be detected online
    # @param K_col collision weighting for soft collision boundary of eTaSL
    # @param uri mesh uri in case of mesh geometry using file resource
    # @param scale scale of the obejct in case of mesh geometry. In case of points, scale[0:2] are points' widths and heights
    # @param parent parent geometry, in case it is a child geometry
    # @param vertices (Nx3) points of mesh if you put vertices directly
    # @param triangles (Nx3) indeces of points for triangles
    # @param colors colors of points or triangles, in case of mesh with them
    # @param in_urdf for geometries that are already in urdf file - e.g. no need to add to move it scene
    # @param text text for GEOTYPE.TEXT. only scale[0] is the scale of text
    def __init__(self, gscene, gtype, name, link_name, dims, center, rpy=(0,0,0), color=(0,1,0,1), display=True,
                 collision=True, fixed=False, soft=False, online=False, K_col=None, uri="", scale=(1,1,1), create=True,
                 parent=None, vertices=None, triangles=None, colors=None, in_urdf=False, text=None):
        self.uri, self.scale = uri, scale
        self.vertices=vertices
        self.triangles=triangles
        self.colors = colors
        self.in_urdf = in_urdf
        self.text = text
        if vertices is not None:
            if not all(np.abs(np.mean([np.max(vertices, axis=0), np.min(vertices, axis=0)], axis=0)) <=1e-6):
                TextColors.YELLOW.println(
                    "[WARN] Vertices for mesh should be have center point (0,0,0). Auto adjusting.")
                vertices = np.copy(vertices)
                point_ct = np.mean([np.max(vertices, axis=0), np.min(vertices, axis=0)], axis=0)
                self.vertices = vertices - point_ct
                center = center+np.matmul(Rot_rpy(rpy), point_ct)
        self.children = []
        self.parent = parent
        self.name = name
        if gtype in GEOTYPE:
            self.gtype = gtype
        else:
            self.gtype = GEOTYPE[gtype]
        if create:
            self.gscene = gscene
        if self.parent is not None:
            gscene.NAME_DICT[self.parent].children.append(self.name)
        self.set_offset_tf(center=center, orientation_mat=Rot_rpy(rpy), call_in_parent_coord=self.parent is not None)
        self.set_dims(dims)
        self.color = color
        self.display = display
        self.collision = collision
        self.fixed = fixed
        self.fixed_on_world = fixed and link_name=="base_link"
        self.soft = soft
        self.online = online
        self.K_col = K_col
        self.set_link(link_name, call_from_parent=True)
        if create:
            self.gscene.append(self)

    ##
    # @brief set dimension, update raidus and length for cylinder, capsule and sphere
    # @param dims tuple of 3 doubles in m scale.
    def set_dims(self, dims):
        if dims is None:
            return
        self.dims = dims
        self.radius = np.mean(dims[:2])/2 if self.gtype in [GEOTYPE.SPHERE, GEOTYPE.CAPSULE, GEOTYPE.CYLINDER] else 0
        self.length = dims[2]

    ##
    # @brief set attached link and update adjacent links
    # @param link_name name of link to be attached
    def set_link(self, link_name, call_from_parent=False):
        if self.parent is not None and not call_from_parent:
            self.gscene.NAME_DICT[self.parent].set_link(link_name)
        self.link_name = link_name
        self.adjacent_links = self.gscene.link_adjacency_map[self.link_name]
        for child in self.children:
            self.gscene.NAME_DICT[child].set_link(link_name, call_from_parent=True)

    ##
    # @brief get x direction vector
    def get_xvec(self):
        return self.orientation_mat[:,0]

    ##
    # @brief get y direction vector
    def get_yvec(self):
        return self.orientation_mat[:,1]

    ##
    # @brief get z direction vector
    def get_zvec(self):
        return self.orientation_mat[:,2]

    # @brief get transformation matrix from reference link
    # @param joint_dict joint values in dictionary format {name: radian value}
    # @param from_link name of reference link
    def get_tf(self, joint_dict, from_link='base_link'):
        joint_dict = joint_dict if isinstance(joint_dict, dict) else list2dict(joint_dict, self.gscene.joint_names)
        T = get_tf(to_link=self.link_name, joint_dict=joint_dict, urdf_content=self.gscene.urdf_content, from_link=from_link)
        T = np.matmul(T, self.Toff)
        return T

    # @brief set transformation matrix to attached link coordinate, update rpy and Toff
    # @param center xyz position in attached link coordinate (tuple, m scale). (if child, in parent coordinate)
    # @param orientation_mat 3x3 orientation matrix relative to attached link coordinate. (if child, in parent coordinate)
    # @param call_in_parent_coord flag for child geometry, make sure center and orientation are in parent's coordinate
    def set_offset_tf(self, center=None, orientation_mat=None, call_in_parent_coord=False):
        if call_in_parent_coord:
            if self.parent is None:
                raise(RuntimeError("Parent is not set for a child geometry"))
            ptem = self.gscene.NAME_DICT[self.parent]
            ## @brief (if child) xyz position in parent coordinate (tuple, m scale)
            self.center_child = deepcopy(center) if center is not None else self.center_child
            ## @brief orientation matrix relative to attached link coordinate
            self.orientation_mat_child = orientation_mat if orientation_mat is not None else self.orientation_mat_child
            ## @brief transformation matrix from parent geometry to attached link coordinate
            self.Toff_child = SE3(self.orientation_mat_child, self.center_child)

            ## @brief transformation matrix from geometry coordinate to attached link coordinate
            self.Toff = np.matmul(ptem.Toff, self.Toff_child)
            ## @brief xyz position in attached link coordinate (tuple, m scale)
            self.center = tuple(self.Toff[:3,3])
            ## @brief orientation matrix relative to attached link coordinate
            self.orientation_mat = self.Toff[:3,:3].copy()
            ## @brief roll-pitch-yaw orientation relative to attached link coordinate
            self.rpy = Rot2rpy(self.orientation_mat)
        else:
            self.center = deepcopy(center) if center is not None else self.center
            self.rpy = Rot2rpy(orientation_mat) if orientation_mat is not None else self.rpy
            self.orientation_mat = orientation_mat if orientation_mat is not None else self.orientation_mat
            self.Toff = SE3(self.orientation_mat, self.center)
            if self.parent is not None:
                Toff_parent = np.matmul(self.Toff, SE3_inv(self.Toff_child))
                self.gscene.NAME_DICT[self.parent].set_offset_tf(center=tuple(Toff_parent[:3,3]), orientation_mat=Toff_parent[:3,:3])

        for child in self.children:
            self.gscene.NAME_DICT[child].set_offset_tf(call_in_parent_coord=True)

    ##
    # @brief check if the geometry can be controlled with joints
    def is_controlled(self):
        return self.gscene.link_control_map[self.link_name]

    ##
    # @brief get maximum distance from the link origin
    def get_off_max(self):
        Toff = self.Toff
        Roff, Poff = Toff[:3, :3], Toff[:3, 3]
        return np.abs(Poff) + np.abs(np.matmul(Roff, self.dims))/2
    ##
    # @brief get local vertice and maximum radius of the geometry
    def get_vertice_radius(self):
        if self.gtype == GEOTYPE.MESH:
            return self.vertices, 0
        else:
            return np.multiply(DEFAULT_VERT_DICT[self.gtype], self.dims), \
                   (0 if self.gtype == GEOTYPE.CYLINDER else self.radius)
    ##
    # @brief get vertice from specific link
    def get_vertice_radius_from(self, joint_dict, from_link='base_link'):
        T = self.get_tf(joint_dict, from_link=from_link)
        verts, radius = self.get_vertice_radius()
        return np.transpose(np.matmul(T[:3,:3], verts.transpose()) + T[:3,3:]), radius

    ##
    # @brief draw trajectory coordinates
    # @param Q_list list of joint configurations
    # @param traj_name name id for the trajectory
    def draw_traj_coords(self, Q_list, traj_name=None):
        if traj_name is None:
            traj_name = self.name
        T_q_list = []
        for i_q, q in enumerate(Q_list):
            T_q = self.get_tf(list2dict(q, self.gscene.joint_names))
            self.gscene.add_highlight_axis(traj_name, "{}_{}".format(traj_name, i_q),
                                      "base_link",  center=T_q[:3,3], orientation_mat=T_q[:3,:3])
            T_q_list.append(T_q)
        return T_q_list

    ##
    # @brief get all geometry names that share same parent-children tree
    def get_family(self):
        gtem = self
        while gtem.parent is not None:
            gtem = self.gscene.NAME_DICT[gtem.parent]
        to_add = list(gtem.children)
        family = [gtem.name]
        while len(to_add)>0:
            gtem = self.gscene.NAME_DICT[to_add.pop(0)]
            family.append(gtem.name)
            to_add += gtem.children
        return family

    ##
    # @brief get root geometry item
    def get_root(self):
        gtem = self
        while gtem.parent is not None:
            gtem = self.gscene.NAME_DICT[gtem.parent]
        return gtem.name

    ##
    # @brief permute geometry axis to match z direction with link
    def permute_axis_match_z_link(self):
        gtem = self
        if gtem.gtype in [GEOTYPE.ARROW, GEOTYPE.MESH]:  # none-permutable types
            return
        Rbo = gtem.orientation_mat
        xbo, ybo, zbo = Rbo
        zmax = np.argmax(np.abs(zbo))
        Roff = np.identity(3)
        if gtem.gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER, GEOTYPE.PLANE]: # only 180-degree flippable types
            if zbo[2]<0:
                Roff = Rot_axis(1, np.pi)
        elif gtem.gtype in [GEOTYPE.BOX, GEOTYPE.SPHERE]: # 90-degree permutable types
            if zmax == 2:  # if already aligned with z axis
                if zbo[2]<0:  # if flipped
                    Roff = Rot_axis(1, np.pi)
            else:  # if aligned with x or y
                rot_dir = (zmax*2-1) * (int(zbo[zmax]<0)*2-1)
                Roff = Rot_axis((zmax + 1) % 2 + 1, rot_dir*np.pi/2)
        else:
            raise(NotImplementedError("Permutation for {} is not implemented.".format(gtem.gtype)))

        gtem.set_offset_tf(orientation_mat=np.matmul(gtem.orientation_mat, Roff))
        gtem.set_dims(tuple(np.abs(np.matmul(Roff.transpose(), gtem.dims))))
        
    def get_args(self):
        center = self.center if self.parent is None else self.center_child
        rpy = Rot2rpy(self.orientation_mat) if self.parent is None else Rot2rpy(self.orientation_mat_child)
        return {'gtype': self.gtype, 'name': self.name, 'link_name': self.link_name, 
                'dims': self.dims, 'center': center, 'rpy': rpy, 'color': self.color, 
                'display': self.display, 'collision': self.collision, 'fixed': self.fixed, 
                'parent': self.parent}
    ##
    # @brief calculate jacobian for a geometry movement
    # @param gtem   GeometryItem
    # @param Q      current joint pose as array
    # @param ref_link reference link to calculate pose
    def get_jacobian(self, Q, ref_link="base_link"):
        return self.gscene.get_jacobian(self.link_name, Q, Toff=self.Toff, ref_link=ref_link)

    ##
    # @param dVW twist on base coord = concat(w, v)
    def get_joint_increment(self, Q0, twist, ref_link="base_link", Jac=None, Tcur=None):
        return get_joint_increment(self.link_name, Q0, twist, Toff=self.Toff, ref_link=ref_link, Jac=Jac, Tcur=Tcur)

def load_gtem_args(gscene, gtem_args):
    gtem_remove = []
    for gtem in gscene:
        if ((gtem.link_name == "base_link" or not gtem.fixed)
                and gtem.parent is None):
            gtem_remove.append(gtem)
    for gtem in gtem_remove:
        gscene.remove(gtem)

    gid_list = np.arange(len(gtem_args)).tolist()
    for gidx in gid_list:
        args = gtem_args[gidx]
        if args['parent'] is not None:
            if args['parent'] not in gscene.NAME_DICT:
                gid_list.append(gidx)
                continue
        gscene.create_safe(**args)