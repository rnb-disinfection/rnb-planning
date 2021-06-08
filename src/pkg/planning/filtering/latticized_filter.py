import os
RNB_PLANNING_DIR = os.environ["RNB_PLANNING_DIR"]

import numpy as np
from .filter_interface import MotionFilterInterface
from ..constraint.constraint_common import calc_redundancy
from ...utils.joint_utils import *
from ...utils.gjk import get_point_list, get_gjk_distance
from ...utils.rotation_utils import *
from ...utils.utils import *
import itertools
import subprocess

from .lattice_model.latticizer_py import *
from .grasp_filter import *
import SharedArray as sa

BATCH_SIZE = 1
SERVER_PERIOD = 1e-3
GRASP_SHAPE = (20,20,20)
ARM_SHAPE = (20,20,20)
RH_MASK_SIZE = 512
RH_MASK_STEP = 64

# def div_r(r):
#     return floor(sigmoid((r)/0.1-8)*8)

def div_r(r):
    return floor(sigmoid((r)/0.1-7)*8)


def div_h(h):
    return int(floor(sigmoid((h + 0.6) / 0.2 - 4.5) * 8))

## CHANGED 2021.06.06
# T_ej not exact, get r, h from T_ee

##
# @class    LatticedChecker
# @brief    lattice the scene and predict feasibility by a learnt model
# @remark   launch SharedArray predictor server rnb-planning.src.pkg.planning.filtering.lattice_model.shared_lattice_predictor.SharedLatticePredictor,
#           which is separated because tensorflow needs to be run in python 3
class LatticedChecker(MotionFilterInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param gcheck GraspChecker
    def __init__(self, pscene, gcheck):
        self.pscene = pscene
        self.combined_robot = pscene.combined_robot
        self.model_dict = {}
        self.robot_names = pscene.combined_robot.robot_names
        self.robot_chain_dict = pscene.robot_chain_dict
        binder_links = [self.robot_chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}
        self.rconfig_dict = self.combined_robot.get_robot_config_dict()

        self.gscene = pscene.gscene
        self.gcheck = gcheck
        self.end_link_couple_dict = gcheck.end_link_couple_dict
        self.ltc_effector = Latticizer_py(WDH=(1, 1, 1), L_CELL=0.05, OFFSET_ZERO=(0.5, 0.5, 0.5))
        self.ltc_arm_10 = Latticizer_py(WDH=(2, 2, 2), L_CELL=0.10, OFFSET_ZERO=(0.5, 1.0, 1.0))
        # Create an array in shared memory.
        self.prepared_p_dict = {}
        self.grasp_img_p_dict, self.arm_img_p_dict, self.rh_vals_p_dict, self.result_p_dict,\
            self.query_in_dict, self.response_out_dict, self.query_quit_dict, = {}, {}, {}, {}, {}, {}, {}

        self.subp_list=[]
        for rconfig in self.combined_robot.robots_on_scene:
            robot_type_name = rconfig.type.name
            try:
                self.prepared_p_dict[robot_type_name] = sa.attach("shm://{}.prepared".format(robot_type_name))
            except:
                self.subp_list.append(subprocess.Popen(['python3',
                                                        '{}src/pkg/planning/filtering/lattice_model/shared_lattice_predictor.py'.format(RNB_PLANNING_DIR),
                                                        '--rtype', robot_type_name]))
                time.sleep(0.5)
                self.prepared_p_dict[robot_type_name] = sa.attach("shm://{}.prepared".format(robot_type_name))
                while not self.prepared_p_dict[robot_type_name][0]:
                    time.sleep(0.1)

            self.grasp_img_p_dict[robot_type_name] = sa.attach("shm://{}.grasp_img".format(robot_type_name))
            self.arm_img_p_dict[robot_type_name] = sa.attach("shm://{}.arm_img".format(robot_type_name))
            self.rh_vals_p_dict[robot_type_name] = sa.attach("shm://{}.rh_vals".format(robot_type_name))
            self.result_p_dict[robot_type_name] = sa.attach("shm://{}.result".format(robot_type_name))
            self.query_in_dict[robot_type_name] = sa.attach("shm://{}.query_in".format(robot_type_name))
            self.response_out_dict[robot_type_name] = sa.attach("shm://{}.response_out".format(robot_type_name))
            self.query_quit_dict[robot_type_name] = sa.attach("shm://{}.query_quit".format(robot_type_name))

        self.base_dict = self.combined_robot.get_robot_base_dict()

        self.shoulder_link_dict = {rname: pscene.gscene.urdf_content.joint_map[rchain['joint_names'][1]].child
                                   for rname, rchain in self.robot_chain_dict.items()}
        self.shoulder_height_dict = {
            rname: get_tf(shoulder_link, self.combined_robot.home_dict, pscene.gscene.urdf_content)[2, 3]
            for rname, shoulder_link in self.shoulder_link_dict.items()}

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check(self, actor, obj, handle, redundancy_values, Q_dict, interpolate=False):
        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_loal = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        actor_vertinfo_list, object_vertinfo_list, actor_Tinv_dict, object_Tinv_dict = \
            self.gcheck.get_grasping_vert_infos(actor, obj, T_loal, Q_dict)

        obj_names = obj.geometry.get_family()
        group_name_handle = self.binder_link_robot_dict[
            handle.geometry.link_name] if handle.geometry.link_name in self.binder_link_robot_dict else None
        group_name_actor = self.binder_link_robot_dict[
            actor.geometry.link_name] if actor.geometry.link_name in self.binder_link_robot_dict else None

        if group_name_actor and not group_name_handle:
            group_name = group_name_actor
            T_robot_base = get_tf(self.base_dict[group_name], joint_dict=Q_dict,
                                  urdf_content=self.gscene.urdf_content, from_link=obj.geometry.link_name)
            T_end_effector = np.matmul(SE3_inv(T_robot_base), T_loal)
            tool_vertinfo_list = [(name, np.matmul(SE3_inv(T_robot_base), T), verts, radius, dims) for name, T, verts, radius, dims in actor_vertinfo_list]
            target_vertinfo_list = [(name, np.matmul(SE3_inv(T_robot_base), T), verts, radius, dims) for name, T, verts, radius, dims in object_vertinfo_list]
        elif group_name_handle and not group_name_actor:
            group_name = group_name_handle
            T_robot_base = get_tf(self.base_dict[group_name], joint_dict=Q_dict,
                                  urdf_content=self.gscene.urdf_content, from_link=actor.geometry.link_name)
            T_end_effector = np.matmul(SE3_inv(T_robot_base), SE3_inv(T_loal))
            tool_vertinfo_list = [(name, np.matmul(T_end_effector, T), verts, radius, dims) for name, T, verts, radius, dims in object_vertinfo_list]
            target_vertinfo_list = [(name, np.matmul(T_end_effector, T), verts, radius, dims) for name, T, verts, radius, dims in actor_vertinfo_list]
        else:
            raise ("Invaild robot")

        verts_to_move = []
        for vertinfo in tool_vertinfo_list:
            gname = vertinfo[0]
            if gname in obj_names:
                verts_to_move.append(vertinfo)
        target_vertinfo_list = target_vertinfo_list + verts_to_move
        for vertinfo in verts_to_move:
            tool_vertinfo_list.remove(vertinfo)

        self.ltc_effector.clear()
        self.ltc_arm_10.clear()

        T_end_joint = T_end_effector

        r, th, h = cart2cyl(*T_end_effector[:3, 3])
        Tref = SE3(Rot_axis(3, th), T_end_effector[:3, 3]) # in robot base link coordinate
        target_names = [item[0] for item in target_vertinfo_list if item[0] not in obj_names]
        tool_names = [item[0] for item in tool_vertinfo_list]

        self.ltc_effector.convert_vertices(tool_vertinfo_list, Tref=Tref)
        self.ltc_effector.convert_vertices(target_vertinfo_list, Tref=Tref)

        Tref_base = SE3(Tref[:3, :3], (0, 0, self.shoulder_height_dict[group_name])) # in robot base link coordinate

        self.ltc_arm_10.convert([gtem for gtem in self.gscene
                                 if gtem.collision
                                 and gtem.link_name not in self.robot_chain_dict[group_name]["link_names"]],
                                self.combined_robot.home_dict,
                                Tref=np.matmul(T_robot_base, Tref_base))

        grasp_tar_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname] for tname in target_names if
                                                     tname in self.ltc_effector.coll_idx_dict])))
        grasp_tool_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname] for tname in tool_names if
                                                      tname in self.ltc_effector.coll_idx_dict])))
        grasp_obj_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname] for tname in obj_names if
                                                      tname in self.ltc_effector.coll_idx_dict])))
        arm_tar_idx = sorted(set(itertools.chain(*[self.ltc_arm_10.coll_idx_dict[tname] for tname in target_names if
                                                   tname in self.ltc_arm_10.coll_idx_dict])))
        T_ee, T_ej = T_end_effector, T_end_joint

        r, th, h = cart2cyl(*T_ee[:3, 3])
#         r_ej, th, h_ej = cart2cyl(*T_ej[:3, 3])
        rh_vals = np.array(r, h)
        grasp_tool_img = np.zeros(GRASP_SHAPE)
        grasp_tar_img = np.zeros(GRASP_SHAPE)
        grasp_obj_img = np.zeros(GRASP_SHAPE)
        grasp_tool_img[np.unravel_index(grasp_tool_idx, shape=GRASP_SHAPE)] = 1
        grasp_tar_img[np.unravel_index(grasp_tar_idx, shape=GRASP_SHAPE)] = 1
        grasp_obj_img[np.unravel_index(grasp_obj_idx, shape=GRASP_SHAPE)] = 1
        arm_img = np.zeros(ARM_SHAPE + (1,))
        arm_img[np.unravel_index(arm_tar_idx, shape=ARM_SHAPE)] = 1
        grasp_img = np.stack([grasp_tool_img, grasp_obj_img, grasp_tar_img], axis=-1)
        res = self.query_wait_response(self.rconfig_dict[group_name].type.name,
                                       np.array([grasp_img]), np.array([arm_img]), np.array([rh_vals]),
                                       )[0]
        return res[-1]>0.5

    def query_wait_response(self, robot_type_name, grasp_img_batch, arm_img_batch, rh_vals_batch):
        self.grasp_img_p_dict[robot_type_name][:] = grasp_img_batch[:]
        self.arm_img_p_dict[robot_type_name][:] = arm_img_batch[:]
        self.rh_vals_p_dict[robot_type_name][:] = rh_vals_batch[:]
        self.query_in_dict[robot_type_name][0] = True
        while not self.response_out_dict[robot_type_name][0]:
            time.sleep(SERVER_PERIOD)
        self.response_out_dict[robot_type_name][0] = False
        return np.copy(self.result_p_dict[robot_type_name])

    def quit_shared_server(self, robot_type_name):
        self.query_quit_dict[robot_type_name][0] = True

    def __del__(self):
        for rconfig in self.combined_robot.robots_on_scene:
            self.quit_shared_server(rconfig.type.name)