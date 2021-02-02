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
        point_add, rpy_add = calc_redundancy(redundancy, actor)
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        actor_link_names = self.end_link_couple_dict[actor_link]
        object_link_names = self.end_link_couple_dict[object_link]
        actor_geo_list = self.gscene.get_items_on_links(actor_link_names)
        object_geo_list = self.gscene.get_items_on_links(object_link_names)

        #     with gtimer.block("link_offset"):
        T_handle_lh = handle.Toff_lh
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add), point_add))
        T_link_handle_actor_link = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        #     with gtimer.block('extract_vertices'):
        actor_Tinv_dict = get_reverse_T_dict(actor_link_names, Q_dict, self.gscene.urdf_content)
        object_Tinv_dict = get_reverse_T_dict(object_link_names, Q_dict, self.gscene.urdf_content)
        actor_vertice_list = []
        object_vertice_list = []
        for ac_geo in actor_geo_list:
            if ac_geo.collision:
                T = actor_Tinv_dict[ac_geo.link_name]
                verts, radius = ac_geo.get_vertice_radius()
                verts = np.matmul(verts, ac_geo.Toff[:3, :3].transpose()) + ac_geo.Toff[:3, 3]
                verts = np.matmul(verts, T[:3, :3].transpose()) + T[:3, 3]
                verts = np.matmul(verts, T_link_handle_actor_link[:3, :3].transpose()) + T_link_handle_actor_link[:3, 3]
                actor_vertice_list.append((get_point_list(verts), radius))
        for obj_geo in object_geo_list:
            if obj_geo.collision:
                T = object_Tinv_dict[obj_geo.link_name]
                verts, radius = obj_geo.get_vertice_radius()
                verts = np.matmul(verts, obj_geo.Toff[:3, :3].transpose()) + obj_geo.Toff[:3, 3]
                verts = np.matmul(verts, T[:3, :3].transpose()) + T[:3, 3]
                object_vertice_list.append((get_point_list(verts), radius))

        #     with gtimer.block("calc_distance"):
        dist_list = []
        for actor_vertice, actor_radius in actor_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(actor_vertice, object_vertice) - actor_radius - object_radius)
        return np.min(dist_list) > 0