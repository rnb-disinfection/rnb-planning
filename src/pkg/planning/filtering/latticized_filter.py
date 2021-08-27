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
WDH_GRASP = (1, 1, 1)
L_CELL_GRASP = 0.05
OFFSET_ZERO_GRASP = (0.5, 0.5, 0.5)
WDH_ARM = (2, 2, 2)
L_CELL_ARM = 0.10
OFFSET_ZERO_ARM = (0.5, 1.0, 1.0)
RH_MASK_SIZE = 512
RH_MASK_STEP = 64

LOG_SCENES = True


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
        self.ltc_effector = Latticizer_py(WDH=WDH_GRASP, L_CELL=L_CELL_GRASP, OFFSET_ZERO=OFFSET_ZERO_GRASP)
        self.ltc_arm_10 = Latticizer_py(WDH=WDH_ARM, L_CELL=L_CELL_ARM, OFFSET_ZERO=OFFSET_ZERO_ARM)
        # Create an array in shared memory.
        self.prepared_p_dict = {}
        self.grasp_img_p_dict, self.arm_img_p_dict, self.rh_vals_p_dict, self.result_p_dict,\
            self.query_in_dict, self.response_out_dict, self.query_quit_dict, = {}, {}, {}, {}, {}, {}, {}

        self.subp_list=[]
        self.server_on_me = []
        for rconfig in self.combined_robot.robots_on_scene:
            robot_type_name = rconfig.type.name
            try:
                self.prepared_p_dict[robot_type_name] = sa.attach("shm://{}.prepared".format(robot_type_name))
            except:
                self.subp_list.append(subprocess.Popen(['python3',
                                                        '{}src/pkg/planning/filtering/lattice_model/shared_lattice_predictor.py'.format(RNB_PLANNING_DIR),
                                                        '--rtype', robot_type_name]))
                self.server_on_me.append(robot_type_name)
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
        self.lock = DummyBlock()

        if LOG_SCENES:
            for _ in range(10):
                TextColors.RED.println("==============================================================")
                TextColors.RED.println("========= WARNING: LatticedChecker SCENE LOGGING ON ==========")
                TextColors.RED.println("==============================================================")

    ##
    # @brief define lock if it needs lock in multiprocess calls
    def prepare_multiprocess_lock(self, manager):
        if manager is not None:
            self.lock = manager.Lock()
        else:
            self.lock = DummyBlock()

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check(self, actor, obj, handle, redundancy_values, Q_dict, interpolate=False, **kwargs):
        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_loal = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        return self.check_T_loal(actor, obj, T_loal, Q_dict, interpolate=interpolate,
                                 **kwargs)

    ##
    # @brief check end-effector collision in grasping
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param T_loal     transformation matrix from object-side link to actor-side link
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check_T_loal(self, actor, obj, T_loal, Q_dict, interpolate=False,**kwargs):
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name

        actor_vertinfo_list, object_vertinfo_list, actor_Tinv_dict, object_Tinv_dict = \
            self.gcheck.get_grasping_vert_infos(actor, obj, T_loal, Q_dict)

        obj_names = obj.geometry.get_family()
        group_name_handle = self.binder_link_robot_dict[object_link] if object_link in self.binder_link_robot_dict else None
        group_name_actor = self.binder_link_robot_dict[actor_link] if actor_link in self.binder_link_robot_dict else None

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
        try:
            grasp_obj_img[np.unravel_index(grasp_obj_idx, shape=GRASP_SHAPE)] = 1
            if LOG_SCENES:
                save_scene(self.pscene.gscene, arm_tar_idx, grasp_tool_idx, grasp_tar_idx, grasp_obj_idx, [r, th, h],
                           error_state=False)
        except Exception as e:
            if LOG_SCENES:
                save_scene(self.pscene.gscene, arm_tar_idx, grasp_tool_idx, grasp_tar_idx, grasp_obj_idx, [r, th, h],
                           error_state=True)
            print("===== THE ERROR OCCURED!!! =====")
            print("===== THE ERROR OCCURED!!! =====")
            print("===== THE ERROR OCCURED!!! =====")
            print(e)
            print("===== grasp_obj_idx =====")
            print(grasp_obj_idx)
            print("===== coll_idx_dict.keys() =====")
            print(self.ltc_effector.coll_idx_dict.keys())
            print("===== obj_names =====")
            print(obj_names)
            value = raw_input("Wait key input : ")
        # if not hasattr(LatticedChecker, "test_count"):
        #     LatticedChecker.test_count = 0
        # LatticedChecker.test_count += 1
        # try_mkdir("data")
        # np.save("data/grasp_tool_img_%04d.npy"%(LatticedChecker.test_count),grasp_tool_img)
        # np.save("data/grasp_tar_img%04d.npy"%(LatticedChecker.test_count),grasp_tar_img)
        # np.save("data/grasp_obj_img%04d.npy"%(LatticedChecker.test_count),grasp_obj_img)

        arm_img = np.zeros(ARM_SHAPE + (1,))
        arm_img[np.unravel_index(arm_tar_idx, shape=ARM_SHAPE)] = 1
        grasp_img = np.stack([grasp_tool_img, grasp_obj_img, grasp_tar_img], axis=-1)
        res = self.query_wait_response(self.rconfig_dict[group_name].type.name,
                                       np.array([grasp_img]), np.array([arm_img]), np.array([rh_vals]),
                                       )[0]
        return res[-1]>0.5

    def query_wait_response(self, robot_type_name, grasp_img_batch, arm_img_batch, rh_vals_batch):
        with self.lock:
            self.grasp_img_p_dict[robot_type_name][:] = grasp_img_batch[:]
            self.arm_img_p_dict[robot_type_name][:] = arm_img_batch[:]
            self.rh_vals_p_dict[robot_type_name][:] = rh_vals_batch[:]
            self.query_in_dict[robot_type_name][0] = True
            while not self.response_out_dict[robot_type_name][0]:
                time.sleep(SERVER_PERIOD)
            self.response_out_dict[robot_type_name][0] = False
            result = np.copy(self.result_p_dict[robot_type_name])
        return result

    def quit_shared_server(self, robot_type_name):
        self.query_quit_dict[robot_type_name][0] = True

    def __del__(self):
        for rname in self.server_on_me:
            self.quit_shared_server(rname)


from copy import deepcopy

SCENE_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data/lcheck_scenes")
try_mkdir(SCENE_PATH)

def save_scene(gscene, arm_tar_idx, grasp_tool_idx, grasp_tar_idx, grasp_obj_idx, rth, error_state):
    gtem_args = []
    for gtem in gscene:
        if gtem.link_name == "base_link" or not gtem.fixed:
            gtem_args.append(deepcopy(gtem.get_args()))

    scene_data = {}
    scene_data["arm_tar_idx"] = arm_tar_idx
    scene_data["grasp_tool_idx"] = grasp_tool_idx
    scene_data["grasp_tar_idx"] = grasp_tar_idx
    scene_data["grasp_obj_idx"] = grasp_obj_idx
    scene_data["rth"] = rth
    scene_data["gtem_args"] = gtem_args
    scene_data["error_state"] = error_state
    save_pickle(
        os.path.join(SCENE_PATH,
                     "{0:08d}-{1}.pkl".format(
                         len(os.listdir(SCENE_PATH)),
                         "ERROR" if error_state else "OK"
                     )), scene_data)