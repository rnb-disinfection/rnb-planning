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

from .lattice_model.latticizer import *
from .grasp_filter import *
import SharedArray as sa

BATCH_SIZE = 1
SERVER_PERIOD = 1e-3
GRASP_SHAPE = (20,20,20)
ARM_SHAPE = (20,20,20)
RH_MASK_SIZE = 512
RH_MASK_STEP = 64

def div_r(r):
    return int(floor(sigmoid((r) / 0.1 - 8) * 8))


def div_h(h):
    return int(floor(sigmoid((h + 0.6) / 0.2 - 4.5) * 8))


##
# @class    LatticedChecker
# @brief    lattice the scene and predict feasibility by a learnt model
# @remark   launch SharedArray predictor server rnb-planning.src.pkg.planning.filtering.lattice_model.shared_lattice_predictor.SharedLatticePredictor,
#           which is separated because tensorflow needs to be run in python 3
class LatticedChecker(MotionFilterInterface):
    ##
    # @param pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param end_link_couple_dict links to douple  in reserse order, {end_link: [end_link, parent1, parnt2, ...]}
    def __init__(self, pscene, end_link_couple_dict):
        self.pscene = pscene
        self.combined_robot = pscene.combined_robot
        self.model_dict = {}
        self.robot_names = pscene.combined_robot.robot_names
        chain_dict = pscene.get_robot_chain_dict()
        binder_links = [chain_dict[rname]['tip_link'] for rname in self.robot_names]
        self.binder_link_robot_dict = {blink: rname for blink, rname in zip(binder_links, self.robot_names)}
        self.rconfig_dict = self.combined_robot.get_robot_config_dict()

        self.gscene = pscene.gscene
        self.end_link_couple_dict = end_link_couple_dict
        self.gcheck = GraspChecker(pscene=pscene, end_link_couple_dict=end_link_couple_dict)
        self.ltc_effector = Latticizer(WDH=(1, 1, 1), L_CELL=0.05, OFFSET_ZERO=(0.5, 0.5, 0.5))
        self.ltc_arm_10 = Latticizer(WDH=(2, 2, 2), L_CELL=0.10, OFFSET_ZERO=(0.5, 1.0, 1.0))
        # Create an array in shared memory.
        self.prepared_p_dict = {}
        self.grasp_img_p_dict, self.arm_img_p_dict, self.rh_mask_p_dict, self.result_p_dict,\
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
            self.rh_mask_p_dict[robot_type_name] = sa.attach("shm://{}.rh_mask".format(robot_type_name))
            self.result_p_dict[robot_type_name] = sa.attach("shm://{}.result".format(robot_type_name))
            self.query_in_dict[robot_type_name] = sa.attach("shm://{}.query_in".format(robot_type_name))
            self.response_out_dict[robot_type_name] = sa.attach("shm://{}.response_out".format(robot_type_name))
            self.query_quit_dict[robot_type_name] = sa.attach("shm://{}.query_quit".format(robot_type_name))
        assert len(self.robot_names) == 1 and self.robot_names[0] == "indy0", \
            "only indy0 supported. to use other, make shoulder_height as dictionary and modify predictor server to inference depending on robot type"
        shoulder_link = pscene.gscene.urdf_content.joint_map[pscene.gscene.joint_names[1]].child
        self.shoulder_height = get_tf(shoulder_link, self.combined_robot.home_dict, pscene.gscene.urdf_content)[2,3]

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy redundancy in dictionary format {axis: value}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    def check(self, actor, obj, handle, redundancy, Q_dict):
        actor_vertinfo_list, object_vertinfo_list, \
        T_link_handle_actor_link, actor_Tinv_dict, object_Tinv_dict = \
            self.gcheck.get_grasping_vert_infos(actor, obj, handle, redundancy, Q_dict)

        group_name_handle = self.binder_link_robot_dict[
            handle.geometry.link_name] if handle.geometry.link_name in self.binder_link_robot_dict else None
        group_name_actor = self.binder_link_robot_dict[
            actor.geometry.link_name] if actor.geometry.link_name in self.binder_link_robot_dict else None

        if group_name_actor and not group_name_handle:
            group_name = group_name_actor
            tool_Tinv_dict = actor_Tinv_dict
            tool_vertinfo_list = actor_vertinfo_list
            target_vertinfo_list = object_vertinfo_list
            T_end_effector = T_link_handle_actor_link
            TOOL_LINK_BUNDLE = self.end_link_couple_dict[actor.geometry.link_name]
        elif group_name_handle and not group_name_actor:
            group_name = group_name_handle
            tool_Tinv_dict = object_Tinv_dict
            tool_vertinfo_list = object_vertinfo_list
            target_vertinfo_list = actor_vertinfo_list
            T_end_effector = SE3_inv(T_link_handle_actor_link)
            TOOL_LINK_BUNDLE = self.end_link_couple_dict[handle.geometry.link_name]
        else:
            raise ("Invaild robot")

        verts_to_move = []
        for vertinfo in tool_vertinfo_list:
            gname = vertinfo[0]
            if gname == obj.geometry.name or gname in obj.geometry.children:
                verts_to_move.append(vertinfo)
        target_vertinfo_list = target_vertinfo_list + verts_to_move
        for vertinfo in verts_to_move:
            tool_vertinfo_list.remove(vertinfo)

        self.ltc_effector.clear()
        self.ltc_arm_10.clear()

        T_end_joint = T_end_effector
        for lname in TOOL_LINK_BUNDLE:
            T_end_joint = np.matmul(T_end_joint, tool_Tinv_dict[lname])

        r, th, h = cart2cyl(*T_end_effector[:3, 3])
        Tref = SE3(Rot_axis(3, th), T_end_effector[:3, 3])
        obj_names = [obj.geometry.name] + obj.geometry.children
        target_names = [item[0] for item in target_vertinfo_list if item[0] not in obj_names]
        tool_names = [item[0] for item in tool_vertinfo_list]

        self.ltc_effector.convert_vertices(tool_vertinfo_list, self.combined_robot.home_dict, Tref=Tref)
        self.ltc_effector.convert_vertices(target_vertinfo_list, self.combined_robot.home_dict, Tref=Tref)

        Tref_base = SE3(Tref[:3, :3], (0, 0, self.shoulder_height))
        self.ltc_arm_10.convert([gtem for gtem in self.gscene if gtem.collision and gtem.link_name=="base_link"], self.combined_robot.home_dict,
                           Tref=Tref_base)

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
        r_ej, th, h_ej = cart2cyl(*T_ej[:3, 3])
        r_class = div_r(r_ej)
        h_class = div_h(h_ej)
        r_mask = np.zeros(RH_MASK_SIZE)
        r_mask[r_class * RH_MASK_STEP:r_class * RH_MASK_STEP + RH_MASK_STEP] = 1
        h_mask = np.zeros(RH_MASK_SIZE)
        h_mask[h_class * RH_MASK_STEP:h_class * RH_MASK_STEP + RH_MASK_STEP] = 1
        rh_mask = np.concatenate([r_mask, h_mask])
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
                                       np.array([grasp_img]), np.array([arm_img]), np.array([rh_mask]),
                                       )[0]
        return res[-1]>0.5

    def query_wait_response(self, robot_type_name, grasp_img_batch, arm_img_batch, rh_mask_batch):
        self.grasp_img_p_dict[robot_type_name][:] = grasp_img_batch[:]
        self.arm_img_p_dict[robot_type_name][:] = arm_img_batch[:]
        self.rh_mask_p_dict[robot_type_name][:] = rh_mask_batch[:]
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