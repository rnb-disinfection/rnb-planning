import numpy as np
from .filter_interface import MotionFilterInterface, save_scene
from ..constraint.constraint_subject import AbstractTask
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, set_point_list, get_gjk_distance
from ...utils.utils import GlobalTimer

##
# @class    TaskClearanceChecker
# @brief    Check pre-defined clearance for task
class TaskClearanceChecker(MotionFilterInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param put_banned GeometryItem list to indicate area where not to put object
    def __init__(self, pscene, gcheck):
        self.pscene = pscene
        self.gcheck = gcheck
        self.gscene = pscene.gscene
        self.dist = -1

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check(self, actor, obj, handle, btf, Q_dict, interpolate=False):
        if (not isinstance(obj, AbstractTask)) \
                or (not hasattr(obj, "clearance")) \
                or obj.clearance is None:
            return True

        gcheck = self.gcheck
        object_geo_list = [gtem for gtem in self.gscene if gtem.collision and not gtem.fixed]
        ex_link_list = gcheck.robot_ex_link_dict[gcheck.link_robot_dict[actor.geometry.link_name]]
        T_dict = get_T_dict_foward("base_link", ex_link_list, Q_dict, self.gscene.urdf_content)

        object_vertice_list = []
        for obj_geo in object_geo_list:
            T = np.matmul(T_dict[obj_geo.link_name], obj_geo.Toff)
            verts, radius = obj_geo.get_vertice_radius()
            verts = np.matmul(verts, T[:3 ,:3].transpose() ) +T[:3 ,3]
            vert_point_list = get_point_list(verts)
            object_vertice_list.append((vert_point_list, radius))

        clearance_vertice_list = []
        for gtem_clr in obj.clearance:
            T = np.matmul(T_dict[gtem_clr.link_name], gtem_clr.Toff)
            verts, radius = gtem_clr.get_vertice_radius()
            verts = np.matmul(verts, T[:3 ,:3].transpose() ) +T[:3 ,3]
            vert_point_list = get_point_list(verts)
            clearance_vertice_list.append((vert_point_list, radius))

        dist_list = []
        for clear_vertice, clear_radius in clearance_vertice_list:
            for object_vertice, object_radius in object_vertice_list:
                dist_list.append(get_gjk_distance(clear_vertice, object_vertice) - clear_radius - object_radius)
        if len(dist_list)>0:
            self.dist = np.min(dist_list)
            res = np.min(dist_list) > + 1e-4
        else:
            self.dist = 0
            res = True
        return res