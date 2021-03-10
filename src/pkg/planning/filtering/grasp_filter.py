import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, set_point_list, get_gjk_distance
from ...utils.utils import GlobalTimer


##
# @class    GraspChecker
# @brief    grasp checker
class GraspChecker(MotionFilterInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param end_link_couple_dict links to douple  in reserse order, {end_link: [end_link, parent1, parnt2, ...]}
    def __init__(self, pscene, end_link_couple_dict):
        for k,v in end_link_couple_dict.items():
            assert v[0] == k, "actor_link_names should be in reverse order including actor's link as the first item"
        self.gscene = pscene.gscene
        self.end_link_couple_dict = end_link_couple_dict
        # self.verts_holder_dict = {}

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    def check(self, actor, obj, handle, redundancy_values, Q_dict, obj_only=False):
        # gtimer = GlobalTimer.instance()
        # gtimer.tic("get_grasping_vert_infos")
        actor_vertinfo_list, object_vertinfo_list, _, _, _ = self.get_grasping_vert_infos(
            actor, obj, handle, redundancy_values, Q_dict, obj_only=obj_only)
        # gtimer.toc("get_grasping_vert_infos")
        actor_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in actor_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            # if geo_name not in self.verts_holder_dict:
            #     # gtimer.tic("ac_get_point_list")
            #     vert_point_list = get_point_list(verts)
            #     self.verts_holder_dict[geo_name] = vert_point_list
            #     # gtimer.toc("ac_get_point_list")
            # else:
            #     # gtimer.tic("ac_set_point_list")
            #     vert_point_list = self.verts_holder_dict[geo_name]
            #     set_point_list(vert_point_list, verts)
            #     # gtimer.toc("ac_set_point_list")
            actor_vertice_list.append((vert_point_list, radius))
        object_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in object_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            # if geo_name not in self.verts_holder_dict:
            #     # gtimer.tic("obj_get_point_list")
            #     vert_point_list = get_point_list(verts)
            #     self.verts_holder_dict[geo_name] = vert_point_list
            #     # gtimer.toc("obj_get_point_list")
            # else:
            #     # gtimer.tic("obj_set_point_list")
            #     vert_point_list = self.verts_holder_dict[geo_name]
            #     set_point_list(vert_point_list, verts)
            #     # gtimer.toc("obj_set_point_list")
            object_vertice_list.append((vert_point_list, radius))

        #     with gtimer.block("calc_distance"):
        # gtimer.tic("get_gjk_distance")
        dist_list = []
        for actor_vertice, actor_radius in actor_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(actor_vertice, object_vertice) - actor_radius - object_radius)
        # gtimer.toc("get_gjk_distance")

        res = np.min(dist_list) > + 1e-4
        return res

    ##
    # @brief transfer actor to binding position and get vertices' information
    # @param actor      rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj        rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle     rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict     joint configuration in dictionary format {joint name: radian value}
    # @param obj_only   only use object and its family's geometry from the object side
    # @return   information for objects attached to the actor in actor_vertinfo_list and
    #           information for objects attached to the object in object_vertinfo_list.
    #           each list item consist of (geometry name, T(4x4), vertices, radius, geometry dimensions)
    def get_grasping_vert_infos(self, actor, obj, handle, redundancy_values, Q_dict, obj_only=False):
        # gtimer = GlobalTimer.instance()
        # gtimer.tic("preprocess")
        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        actor_link_names = self.end_link_couple_dict[actor_link]
        object_link_names = self.end_link_couple_dict[object_link]
        actor_geo_list = self.gscene.get_items_on_links(actor_link_names)
        object_geo_list = self.gscene.get_items_on_links(object_link_names)
        if obj_only:
            object_geo_list = [gtem for gtem in object_geo_list if (gtem.name == obj.geometry.name
                                                                    or gtem.parent == obj.geometry.name
                                                                    or obj.geometry.name in gtem.children)]

        #     with gtimer.block("link_offset"):
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_link_handle_actor_link = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        # gtimer.toc("preprocess")

        #     with gtimer.block('extract_vertices'):
        # gtimer.tic("invT")
        actor_Tinv_dict = get_reverse_T_dict(actor_link_names, Q_dict, self.gscene.urdf_content)
        object_Tinv_dict = get_reverse_T_dict(object_link_names, Q_dict, self.gscene.urdf_content)
        # gtimer.toc("invT")
        actor_vertinfo_list = []
        object_vertinfo_list = []
        for ac_geo in actor_geo_list:
            if ac_geo.collision:
                # gtimer.tic("ac_geo_calc_verts")
                T = actor_Tinv_dict[ac_geo.link_name]
                verts, radius = ac_geo.get_vertice_radius()
                Tac = np.matmul(T_link_handle_actor_link, np.matmul(T, ac_geo.Toff))
                actor_vertinfo_list.append((ac_geo.name, Tac, verts, radius, ac_geo.dims))
                # gtimer.toc("ac_geo_calc_verts")
        for obj_geo in object_geo_list:
            if obj_geo.collision:
                # gtimer.tic("obj_geo_calc_verts")
                T = object_Tinv_dict[obj_geo.link_name]
                verts, radius = obj_geo.get_vertice_radius()
                Tobj = np.matmul(T, obj_geo.Toff)
                object_vertinfo_list.append((obj_geo.name, Tobj, verts, radius, obj_geo.dims))
                # gtimer.toc("obj_geo_calc_verts")
        return actor_vertinfo_list, object_vertinfo_list, T_link_handle_actor_link, actor_Tinv_dict, object_Tinv_dict