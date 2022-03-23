import numpy as np
from .filter_interface import MotionFilterInterface, save_scene
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, set_point_list, get_gjk_distance
from ...utils.utils import GlobalTimer,TextColors
from ...geometry.geotype import *
import random

DEBUG_MODE_GRAB_FILT_LOG = False

if DEBUG_MODE_GRAB_FILT_LOG:
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
            children = self.gscene.get_children_links(self.pscene.combined_robot.get_robot_base_dict()[rname])
            self.end_link_couple_dict[tip_link] = [lname for lname in reversed(robot_link_names)
                                                   if lname in self.gscene.fixed_link_adjacency_map[tip_link]]
            index_arr = np.array([self.gscene.link_names.index(lname) for lname in robot_link_names])
            assert np.all((index_arr[1:]-index_arr[:-1])>0), "link_name should be ordered as same as chain order"
            self.robot_ex_link_dict[rname] = [lname for lname in self.gscene.link_names
                                              if lname not in children]
            for lname in robot_link_names:
                self.link_robot_dict[lname] = rname
        # @brief set show_vertices=True to show collision vertices
        self.show_vertices = False
        self.mesh_first_encounter = True
        self.ignore_always = []
        # self.verts_holder_dict = {}


    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    # @param ignore         GeometryItems to ignore
    def check(self, btf, Q_dict, interpolate=False, obj_only=False, ignore=[],
              **kwargs):
        obj, handle, actor = btf.get_instance_chain(self.pscene)
        T_loal = btf.T_loal
        actor_vertinfo_list, object_vertinfo_list, _, _ = self.get_grasping_vert_infos(
            actor, obj, T_loal, Q_dict, obj_only=obj_only,
            interpolate=interpolate, ignore=ignore)
        # gtimer.toc("get_grasping_vert_infos")
        actor_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in actor_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            actor_vertice_list.append((vert_point_list, radius))
            if self.show_vertices:
                Tbol = np.identity(4) \
                    if obj.geometry.link_name=="base_link" \
                    else SE3_inv(T_loal)
                for i_v, vert in enumerate(verts):
                    self.gscene.add_highlight_axis("gc_actor", geo_name+"_{:03}".format(i_v), "base_link",
                                                   center=np.matmul(Tbol[:3,:3], vert)+ Tbol[:3,3], orientation_mat=None,
                                                   color=(0,0,1,0.3), axis=None)

        object_vertice_list = []
        for geo_name, T, verts, radius, geo_dims in object_vertinfo_list:
            verts = np.matmul(verts, T[:3,:3].transpose())+T[:3,3]
            vert_point_list = get_point_list(verts)
            object_vertice_list.append((vert_point_list, radius))
            if self.show_vertices:
                for i_v, vert in enumerate(verts):
                    self.gscene.add_highlight_axis("gc_object", geo_name+"_{:03}".format(i_v), "base_link",
                                                   center=np.matmul(Tbol[:3,:3], vert)+ Tbol[:3,3], orientation_mat=None,
                                                   color=(1,1,0,0.3), axis=None)

        dist_list = []
        for actor_vertice, actor_radius in actor_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(actor_vertice, object_vertice) - actor_radius - object_radius)
        res = np.min(dist_list) > + 1e-6
        if self.show_vertices:
            print("res: {} ({})".format(res, np.min(dist_list)))
            if not res:
                i_ac, i_ob = np.unravel_index(np.argmin(dist_list), (len(actor_vertinfo_list),len(object_vertinfo_list)))
                print("{} - {}".format(actor_vertinfo_list[i_ac][0], object_vertinfo_list[i_ob][0]))
            T_bl= np.matmul(Tbol, T_loal)
            self.gscene.add_highlight_axis("gc_center", "Tloal", "base_link",
                                           center=T_bl[:3,3], orientation_mat=T_bl[:3,:3])
            # raw_input("wait for key")
            self.gscene.clear_highlight(hl_keys=["gc_actor", "gc_object", "gc_center"])


        if DEBUG_MODE_GRAB_FILT_LOG:
            save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                       error_state=False, result=res,
                       interpolate=interpolate, obj_only=obj_only, ignore=[igtem.name for igtem in ignore], **kwargs)
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

        (object_geo_list, object_T2end_dict), (actor_geo_list, actor_T2end_dict) = \
            self.get_geolist_tflist_pairs(actor, obj, Q_dict, obj_only=obj_only, ignore=ignore)

        if interpolate:
            T_loal_cur = get_tf(actor_link, Q_dict, self.gscene.urdf_content, from_link=object_link)
            T_loal_list = interpolate_T(T_loal_cur, T_loal,
                                        POS_STEP=self.POS_STEP, ROT_STEP=self.ROT_STEP)
        else:
            T_loal_list = [T_loal]

        actor_vertinfo_list = []
        object_vertinfo_list = []
        for ac_geo in actor_geo_list:
            for T_loal_ in T_loal_list:
                if ac_geo.gtype==GEOTYPE.MESH:
                    if self.mesh_first_encounter:
                        self.mesh_first_encounter = False
                        TextColors.RED.println("[WARN] GJK algorithm in GraspChecker cannot handle non-convex mesh. Ignoring all mesh items")
                    continue
                T = actor_T2end_dict[ac_geo.link_name]
                verts, radius = ac_geo.get_vertice_radius()
                Tac = np.matmul(T_loal_, np.matmul(T, ac_geo.Toff))
                actor_vertinfo_list.append((ac_geo.name, Tac, verts, radius, ac_geo.dims))

        for obj_geo in object_geo_list:
            if ac_geo.gtype==GEOTYPE.MESH:
                if self.mesh_first_encounter:
                    self.mesh_first_encounter = False
                    TextColors.RED.println("[WARN] GJK algorithm in GraspChecker cannot handle non-convex mesh. Ignoring all mesh items")
                continue
            T = object_T2end_dict[obj_geo.link_name]
            verts, radius = obj_geo.get_vertice_radius()
            Tobj = np.matmul(T, obj_geo.Toff)
            object_vertinfo_list.append((obj_geo.name, Tobj, verts, radius, obj_geo.dims))
        return actor_vertinfo_list, object_vertinfo_list, actor_T2end_dict, object_T2end_dict

    ##
    # @brief get geometries and corresponding link transform matrices related to grasping motion
    # @param actor      rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj        rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param Q_dict     joint configuration in dictionary format {joint name: radian value}
    # @param obj_only   only use object and its family's geometry from the object side
    # @param ignore         GeometryItems to ignore
    # @return   (object_geo_list,object_T2end_dict), (actor_geo_list, actor_T2end_dict), where T2end_dict is
    #           {link: link transform matrix to actor's or object's link}
    def get_geolist_tflist_pairs(self, actor, obj, Q_dict, obj_only=False, ignore=[]):
        # gtimer = GlobalTimer.instance()
        # gtimer.tic("preprocess")
        _ignore = ignore + self.ignore_always

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

        for gtem_ig in _ignore:
            if gtem_ig in actor_geo_list:
                actor_geo_list.remove(gtem_ig)
            elif gtem_ig in object_geo_list:
                object_geo_list.remove(gtem_ig)

        if actor_link == "base_link":
            actor_T2end_dict = get_T_dict_foward(actor_link, actor_link_names, Q_dict, self.gscene.urdf_content)
        else:
            actor_T2end_dict = get_T_dict_reverse(actor_link, actor_link_names, Q_dict, self.gscene.urdf_content)
        if object_link == "base_link":
            object_T2end_dict = get_T_dict_foward(object_link, object_link_names, Q_dict, self.gscene.urdf_content)
        else:
            object_T2end_dict = get_T_dict_reverse(object_link, object_link_names, Q_dict, self.gscene.urdf_content)

        return (object_geo_list,object_T2end_dict), (actor_geo_list, actor_T2end_dict)
