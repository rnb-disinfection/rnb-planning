import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, get_gjk_distance


##
# @class    GraspChecker
# @brief    grasp checker
class GraspChecker(MotionFilterInterface):
    ##
    # @param gscene rnb-planning.src.pkg.geometry.GeometryScene
    # @param end_link_couple_dict links to douple  in reserse order, {end_link: [end_link, parent1, parnt2, ...]}
    def __init__(self, gscene, end_link_couple_dict):
        for k,v in end_link_couple_dict.items():
            assert v[0] == k, "actor_link_names should be in reverse order including actor's link as the first item"
        self.gscene = gscene
        self.end_link_couple_dict = end_link_couple_dict

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy redundancy in dictionary format {axis: value}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    def check(self, actor, obj, handle, redundancy, Q_dict):

        actor_vertinfo_list, object_vertinfo_list, _, _, _ = self.get_grasping_vert_infos(actor, obj, handle, redundancy, Q_dict)
        actor_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in actor_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            actor_vertice_list.append((get_point_list(verts), radius))
        object_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in object_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            object_vertice_list.append((get_point_list(verts), radius))

        #     with gtimer.block("calc_distance"):
        dist_list = []
        for actor_vertice, actor_radius in actor_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(actor_vertice, object_vertice) - actor_radius - object_radius)

        res = np.min(dist_list) > 0
        return res

    ##
    # @brief transfer actor to binding position and get vertices' information
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy redundancy in dictionary format {axis: value}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @return   information for objects attached to the actor in actor_vertinfo_list and
    #           information for objects attached to the object in object_vertinfo_list.
    #           each list item consist of (geometry name, T(4x4), vertices, radius, geometry dimensions)
    def get_grasping_vert_infos(self, actor, obj, handle, redundancy, Q_dict):
        point_add_handle, rpy_add_handle = calc_redundancy(redundancy[handle.name], handle)
        point_add_actor, rpy_add_actor = calc_redundancy(redundancy[actor.name], actor)
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        actor_link_names = self.end_link_couple_dict[actor_link]
        object_link_names = self.end_link_couple_dict[object_link]
        actor_geo_list = self.gscene.get_items_on_links(actor_link_names)
        object_geo_list = self.gscene.get_items_on_links(object_link_names)

        #     with gtimer.block("link_offset"):
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_link_handle_actor_link = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        #     with gtimer.block('extract_vertices'):
        actor_Tinv_dict = get_reverse_T_dict(actor_link_names, Q_dict, self.gscene.urdf_content)
        object_Tinv_dict = get_reverse_T_dict(object_link_names, Q_dict, self.gscene.urdf_content)
        actor_vertinfo_list = []
        object_vertinfo_list = []
        for ac_geo in actor_geo_list:
            if ac_geo.collision:
                T = actor_Tinv_dict[ac_geo.link_name]
                verts, radius = ac_geo.get_vertice_radius()
                Tac = np.matmul(T_link_handle_actor_link, np.matmul(T, ac_geo.Toff))
                actor_vertinfo_list.append((ac_geo.name, Tac, verts, radius, ac_geo.dims))
        for obj_geo in object_geo_list:
            if obj_geo.collision:
                T = object_Tinv_dict[obj_geo.link_name]
                verts, radius = obj_geo.get_vertice_radius()
                Tobj = np.matmul(T, obj_geo.Toff)
                object_vertinfo_list.append((obj_geo.name, Tobj, verts, radius, obj_geo.dims))
        return actor_vertinfo_list, object_vertinfo_list, T_link_handle_actor_link, actor_Tinv_dict, object_Tinv_dict