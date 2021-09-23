import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, set_point_list, get_gjk_distance
from ...utils.utils import GlobalTimer,TextColors
import random

DEBUG_MODE_GRAB_FILT = False

if DEBUG_MODE_GRAB_FILT:
    TextColors.RED.println("===== WARNING: grasp_filter in DEBUG MODE====")


##
# @class    GraspChecker
# @brief    grasp checker
class GraspChecker(MotionFilterInterface):

    BEFORE_IK = True

    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param put_banned GeometryItem list to indicate area where not to put object
    def __init__(self, pscene, put_banned=[], POS_STEP=0.05, ROT_STEP=np.pi/8):
        self.pscene = pscene
        self.gscene = pscene.gscene
        self.put_banned = put_banned
        self.chain_dict = pscene.robot_chain_dict
        self.POS_STEP = POS_STEP
        self.ROT_STEP = ROT_STEP
        ##
        # @brief link-to-robot dictionary {link name: robot name}
        self.link_robot_dict = {}
        ##
        # @brief links external to the robot {robot name: [link1, link2, ...]}
        self.robot_ex_link_dict = {}
        ##
        # @brief coupled links in reverse order, {end_link: [end_link, parent1, parnt2, ...]}
        self.end_link_couple_dict = {"base_link": ["base_link"]}
        for rname, chain_vals in self.chain_dict.items():
            tip_link = chain_vals['tip_link']
            robot_link_names = chain_vals['link_names']
            self.end_link_couple_dict[tip_link] = [lname for lname in reversed(robot_link_names)
                                                   if lname in self.gscene.fixed_link_adjacency_map[tip_link]]
            index_arr = np.array([self.gscene.link_names.index(lname) for lname in robot_link_names])
            assert np.all((index_arr[1:]-index_arr[:-1])>0), "link_name should be ordered as same as chain order"
            self.robot_ex_link_dict[rname] = [lname for lname in self.gscene.link_names if lname not in robot_link_names]
            for lname in robot_link_names:
                self.link_robot_dict[lname] = rname
        # self.verts_holder_dict = {}


    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param T_loal     transformation matrix from object-side link to actor-side link
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    def check_T_loal(self, actor, obj, T_loal, Q_dict, interpolate=False, obj_only=False, ignore=[],
              **kwargs):
        actor_vertinfo_list, object_vertinfo_list, _, _ = self.get_grasping_vert_infos(
            actor, obj, T_loal, Q_dict, obj_only=obj_only,
            interpolate=interpolate, ignore=ignore)
        # gtimer.toc("get_grasping_vert_infos")
        actor_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in actor_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            actor_vertice_list.append((vert_point_list, radius))
            if DEBUG_MODE_GRAB_FILT:
                for actor_vertice, actor_radius in actor_vertice_list:
                    for i_v, vert in enumerate(verts):
                        self.gscene.add_highlight_axis("gc_actor", geo_name+"_{:03}".format(i_v), "base_link",
                                                       center=vert, orientation_mat=None,
                                                       color=(0,0,1,0.3), axis=None)

        object_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in object_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            object_vertice_list.append((vert_point_list, radius))
            if DEBUG_MODE_GRAB_FILT:
                for actor_vertice, actor_radius in actor_vertice_list:
                    for i_v, vert in enumerate(verts):
                        self.gscene.add_highlight_axis("gc_object", geo_name+"_{:03}".format(i_v), "base_link",
                                                       center=vert, orientation_mat=None,
                                                       color=(1,1,0,0.3), axis=None)

        dist_list = []
        for actor_vertice, actor_radius in actor_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(actor_vertice, object_vertice) - actor_radius - object_radius)
        res = np.min(dist_list) > + 1e-4
        if DEBUG_MODE_GRAB_FILT:
            print("res: {} ({})".format(res, round(np.min(dist_list), 4)))
            if not res:
                i_ac, i_ob = np.unravel_index(np.argmin(dist_list), (len(actor_vertinfo_list),len(object_vertinfo_list)))
                print("{} - {}".format(actor_vertinfo_list[i_ac][0], object_vertinfo_list[i_ob][0]))
            self.gscene.add_highlight_axis("gc_center", "Tloal", obj.geometry.link_name,
                                           center=T_loal[:3,3], orientation_mat=T_loal[:3,:3])
            self.gscene.clear_highlight(hl_keys=["gc_actor", "gc_object", "gc_center"])

        return res

    ##
    # @brief transfer actor to binding position and get vertices' information
    # @param actor      rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj        rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param T_loal     transformation matrix from object-side link to actor-side link
    # @param Q_dict     joint configuration in dictionary format {joint name: radian value}
    # @param obj_only   only use object and its family's geometry from the object side
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    # @return   information for objects attached to the actor in actor_vertinfo_list and
    #           information for objects attached to the object in object_vertinfo_list.
    #           each list item consist of (geometry name, T(4x4), vertices, radius, geometry dimensions)
    def get_grasping_vert_infos(self, actor, obj, T_loal, Q_dict, obj_only=False,
                                interpolate=False, ignore=[]):
        # gtimer = GlobalTimer.instance()
        # gtimer.tic("preprocess")

        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        if actor_link in self.link_robot_dict:      # actor is on the robot (pick)
            actor_link_names = self.end_link_couple_dict[actor_link]
            object_link_names = self.robot_ex_link_dict[self.link_robot_dict[actor_link]]
        elif object_link in self.link_robot_dict:   # object is held by a robot (put)
            actor_link_names = self.robot_ex_link_dict[self.link_robot_dict[object_link]]
            object_link_names = self.end_link_couple_dict[object_link]
        else:
            actor_link_names = self.end_link_couple_dict[actor_link]
            object_link_names = self.end_link_couple_dict[object_link]
        actor_geo_list = self.gscene.get_items_on_links(actor_link_names)
        object_geo_list = self.gscene.get_items_on_links(object_link_names)

        if object_link in self.link_robot_dict:   # object is held by a robot (put, actor is kind of PlacePlane)
            for pb in self.put_banned:
                if pb.link_name in actor_link_names:
                    actor_geo_list.append(pb)
                elif pb.link_name in object_link_names:
                    object_geo_list.append(pb)
                else:
                    raise(NotImplementedError("Put-ban geometric not in famility of object nor actor"))

        if obj_only:
            obj_family = obj.geometry.get_family()
            object_geo_list = [gtem for gtem in object_geo_list if gtem.name in obj_family]

        for gtem_ig in ignore:
            if gtem_ig in actor_geo_list:
                actor_geo_list.remove(gtem_ig)
            elif gtem_ig in object_geo_list:
                object_geo_list.remove(gtem_ig)

        #     with gtimer.block("link_offset"):

        if interpolate:
            T_loal_cur = get_tf(actor_link, Q_dict, self.gscene.urdf_content, from_link=object_link)
            T_loal_list = interpolate_T(T_loal_cur, T_loal,
                                        POS_STEP=self.POS_STEP, ROT_STEP=self.ROT_STEP)
        else:
            T_loal_list = [T_loal]
        # gtimer.toc("preprocess")

        #     with gtimer.block('extract_vertices'):
        # gtimer.tic("invT")
        if actor_link == "base_link":
            actor_Tinv_dict = get_T_dict_foward(actor_link, actor_link_names, Q_dict, self.gscene.urdf_content)
        else:
            actor_Tinv_dict = get_T_dict_reverse(actor_link, actor_link_names, Q_dict, self.gscene.urdf_content)
        if object_link == "base_link":
            object_Tinv_dict = get_T_dict_foward(object_link, object_link_names, Q_dict, self.gscene.urdf_content)
        else:
            object_Tinv_dict = get_T_dict_reverse(object_link, object_link_names, Q_dict, self.gscene.urdf_content)

        # gtimer.toc("invT")
        actor_vertinfo_list = []
        object_vertinfo_list = []
        for ac_geo in actor_geo_list:
            for T_loal_ in T_loal_list:
                # gtimer.tic("ac_geo_calc_verts")
                T = actor_Tinv_dict[ac_geo.link_name]
                verts, radius = ac_geo.get_vertice_radius()
                Tac = np.matmul(T_loal_, np.matmul(T, ac_geo.Toff))
                actor_vertinfo_list.append((ac_geo.name, Tac, verts, radius, ac_geo.dims))
                # gtimer.toc("ac_geo_calc_verts")

        for obj_geo in object_geo_list:
            # gtimer.tic("obj_geo_calc_verts")
            T = object_Tinv_dict[obj_geo.link_name]
            verts, radius = obj_geo.get_vertice_radius()
            Tobj = np.matmul(T, obj_geo.Toff)
            object_vertinfo_list.append((obj_geo.name, Tobj, verts, radius, obj_geo.dims))
            # gtimer.toc("obj_geo_calc_verts")
        return actor_vertinfo_list, object_vertinfo_list, actor_Tinv_dict, object_Tinv_dict
