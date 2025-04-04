import os
RNB_PLANNING_DIR = os.environ["RNB_PLANNING_DIR"]

import numpy as np
from .filter_interface import MotionFilterInterface, save_scene
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

DEBUG_LAT_FILT_LOG = False

if DEBUG_LAT_FILT_LOG:
    TextColors.RED.println("===== WARNING: latticized_filter in DEBUG MODE====")


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
    BEFORE_IK = False

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

    ##
    # @brief define lock if it needs lock in multiprocess calls
    def prepare_multiprocess_lock(self, manager):
        if manager is not None:
            self.lock = manager.Lock()
        else:
            self.lock = DummyBlock()


    ##
    # @brief check end-effector collision in grasping - approximate cell-overlaps with inscribed sphere cells
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check(self, btf, Q_dict, interpolate=False, ignore=[],**kwargs):
        obj, handle, actor = btf.get_instance_chain(self.pscene)
        T_loal = btf.T_loal
        object_link = obj.geometry.link_name
        actor_link = actor.geometry.link_name

        gtimer = GlobalTimer.instance()
        (object_geo_list, object_T2end_dict), (actor_geo_list, actor_T2end_dict) = \
            self.gcheck.get_geolist_tflist_pairs(actor, obj, Q_dict, ignore=ignore, **kwargs)
        with gtimer.block("set_relation"):
            obj_names = obj.geometry.get_family()
            group_name_handle = self.binder_link_robot_dict[
                object_link] if object_link in self.binder_link_robot_dict else None
            group_name_actor = self.binder_link_robot_dict[
                actor_link] if actor_link in self.binder_link_robot_dict else None

            if group_name_actor and not group_name_handle:
                group_name = group_name_actor
                T_br = self.gscene.get_tf(self.base_dict[group_name], Q=Q_dict,
                                          from_link=obj.geometry.link_name)
                T_re = np.matmul(SE3_inv(T_br), T_loal)
                T_tool_to_rob = T_re
                tool_geo_list, tool_T2end_dict = actor_geo_list, actor_T2end_dict
                T_tar_to_rob = SE3_inv(T_br)
                target_geo_list, target_T2end_dict = object_geo_list, object_T2end_dict

            elif group_name_handle and not group_name_actor:
                group_name = group_name_handle
                T_br = self.gscene.get_tf(self.base_dict[group_name], Q=Q_dict,
                                          from_link=actor.geometry.link_name)
                T_re = np.matmul(SE3_inv(T_br), SE3_inv(T_loal))
                T_tool_to_rob = T_re
                tool_geo_list, tool_T2end_dict = object_geo_list, object_T2end_dict
                T_tar_to_rob = np.matmul(T_re, T_loal)
                target_geo_list, target_T2end_dict = actor_geo_list, actor_T2end_dict

            else:
                raise ("Invaild robot")

        self.ltc_effector.clear()
        self.ltc_arm_10.clear()

        r, th, h = cart2cyl(*T_tool_to_rob[:3, 3])
        self.rth_last = r, th, h
        T_rl = SE3(Rot_axis(3, th), T_re[:3, 3])  # in robot base link coordinate
        target_names = [gtem.name for gtem in target_geo_list if gtem.name not in obj_names]
        tool_names = [gtem.name for gtem in tool_geo_list]

        with gtimer.block("effector"):
            T_gl_list = []
            gtem_list = target_geo_list + tool_geo_list
            for gtem in gtem_list:
                if gtem.link_name in tool_T2end_dict:
                    T_rg = matmul_series(T_tool_to_rob, tool_T2end_dict[gtem.link_name], gtem.Toff)
                else:
                    T_rg = matmul_series(T_tar_to_rob, target_T2end_dict[gtem.link_name], gtem.Toff)
                T_lg = np.matmul(SE3_inv(T_rl), T_rg)
                T_gl = SE3_inv(T_lg)
                T_gl_list.append(T_gl)

            self.ltc_effector.convert_vertices_approx(gtem_list, T_gl_list)

        with gtimer.block("arm"):
            link_env = [lname for lname in self.gscene.link_names
                        if lname not in self.robot_chain_dict[group_name]["link_names"]]
            gtems_env = [gtem for gtem in self.gscene
                         if gtem.collision
                         and gtem.link_name in link_env]
            Trl_base = SE3(T_rl[:3, :3], (0, 0, self.shoulder_height_dict[group_name]))  # in robot base link coordinate
            T_bl = np.matmul(T_br, Trl_base)
            T_lb = SE3_inv(T_bl)
            Tlist_env = {lname: self.gscene.get_tf(lname, Q_dict) for lname in link_env}
            T_gl_list_env = []
            for gtem in gtems_env:
                T_lg = matmul_series(T_lb, Tlist_env[gtem.link_name], gtem.Toff)
                T_gl = SE3_inv(T_lg)
                T_gl_list_env.append(T_gl)
            self.ltc_arm_10.convert_vertices_approx(gtems_env, T_gl_list_env)

        with gtimer.block("indexing_vertices"):
            grasp_tar_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname]
                                                         for tname in target_names
                                                         if tname in self.ltc_effector.coll_idx_dict])))
            grasp_tool_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname]
                                                          for tname in tool_names
                                                          if tname in self.ltc_effector.coll_idx_dict])))
            grasp_obj_idx = sorted(set(itertools.chain(*[self.ltc_effector.coll_idx_dict[tname]
                                                         for tname in obj_names
                                                         if tname in self.ltc_effector.coll_idx_dict])))
            arm_tar_idx = sorted(set(itertools.chain(*[self.ltc_arm_10.coll_idx_dict[tname]
                                                       for tname in target_names
                                                       if tname in self.ltc_arm_10.coll_idx_dict])))

            r, th, h = cart2cyl(*T_re[:3, 3])
            rh_vals = np.array([r, h])
            grasp_tool_img = np.zeros(GRASP_SHAPE)
            grasp_tar_img = np.zeros(GRASP_SHAPE)
            grasp_obj_img = np.zeros(GRASP_SHAPE)
            try:
                grasp_tool_img[np.unravel_index(grasp_tool_idx, shape=GRASP_SHAPE)] = 1
                grasp_tar_img[np.unravel_index(grasp_tar_idx, shape=GRASP_SHAPE)] = 1
                grasp_obj_img[np.unravel_index(grasp_obj_idx, shape=GRASP_SHAPE)] = 1
                arm_img = np.zeros(ARM_SHAPE + (1,))
                arm_img[np.unravel_index(arm_tar_idx, shape=ARM_SHAPE)] = 1
                grasp_img = np.stack([grasp_tool_img, grasp_obj_img, grasp_tar_img], axis=-1)
            except Exception as e:
                save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                           error_state=True, result=None, ignore=[igtem.name for igtem in ignore], **kwargs)
                print("===== THE ERROR OCCURED!!! =====")
                print("===== THE ERROR OCCURED!!! =====")
                print("===== THE ERROR OCCURED!!! =====")
                print(e)
                print("===== obj_names =====")
                print(obj_names)

        with gtimer.block("query_wait_response"):
            res = self.query_wait_response(self.rconfig_dict[group_name].type.name,
                                           np.array([grasp_img]), np.array([arm_img]), np.array([rh_vals]),
                                           )[0]
        if DEBUG_LAT_FILT_LOG:
            save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                       error_state=False, result=res, ignore=[igtem.name for igtem in ignore], **kwargs)
        return res[-1]>0.5


    ##
    # @brief check end-effector collision in grasping, calculate voxels by strict cell-overlap checking
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @param interpolate    interpolate path and check intermediate poses
    def check_strict(self, btf, Q_dict, interpolate=False, ignore=[],**kwargs):
        obj, handle, actor = btf.get_instance_chain(self.pscene)
        actor_link = actor.geometry.link_name
        object_link = obj.geometry.link_name
        T_loal = btf.T_loal

        gtimer = GlobalTimer.instance()
        with gtimer.block("get_grasping_vert_infos"):
            actor_vertinfo_list, object_vertinfo_list, actor_T2end_dict, object_T2end_dict = \
                self.gcheck.get_grasping_vert_infos(actor, obj, T_loal, Q_dict, ignore=ignore,
                                                    **kwargs)

        with gtimer.block("transform_verts"):
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

        with gtimer.block("convert_vertices_end"):
            self.ltc_effector.clear()
            self.ltc_arm_10.clear()

            T_end_joint = T_end_effector

            r, th, h = cart2cyl(*T_end_effector[:3, 3])
            self.rth_last = r, th, h
            Tref = SE3(Rot_axis(3, th), T_end_effector[:3, 3]) # in robot base link coordinate
            target_names = [item[0] for item in target_vertinfo_list if item[0] not in obj_names]
            tool_names = [item[0] for item in tool_vertinfo_list]

            self.ltc_effector.convert_vertices(tool_vertinfo_list, Tref=Tref)
            self.ltc_effector.convert_vertices(target_vertinfo_list, Tref=Tref)

        with gtimer.block("convert_vertices_arm"):
            Tref_base = SE3(Tref[:3, :3], (0, 0, self.shoulder_height_dict[group_name])) # in robot base link coordinate

            self.ltc_arm_10.convert([gtem for gtem in self.gscene
                                     if gtem.collision
                                     and gtem.link_name not in self.robot_chain_dict[group_name]["link_names"]],
                                    self.combined_robot.home_dict,
                                    Tref=np.matmul(T_robot_base, Tref_base))

        with gtimer.block("indexing_vertices"):
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
            rh_vals = np.array([r, h])
            grasp_tool_img = np.zeros(GRASP_SHAPE)
            grasp_tar_img = np.zeros(GRASP_SHAPE)
            grasp_obj_img = np.zeros(GRASP_SHAPE)
            try:
                grasp_tool_img[np.unravel_index(grasp_tool_idx, shape=GRASP_SHAPE)] = 1
                grasp_tar_img[np.unravel_index(grasp_tar_idx, shape=GRASP_SHAPE)] = 1
                grasp_obj_img[np.unravel_index(grasp_obj_idx, shape=GRASP_SHAPE)] = 1
            except Exception as e:
                save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                           error_state=True, result=None, ignore=[igtem.name for igtem in ignore], **kwargs)
                print("===== THE ERROR OCCURED!!! =====")
                print("===== THE ERROR OCCURED!!! =====")
                print("===== THE ERROR OCCURED!!! =====")
                print(e)
                print("===== obj_names =====")
                print(obj_names)
    #             value = raw_input("Wait key input : ")
            # if not hasattr(LatticedChecker, "test_count"):
            #     LatticedChecker.test_count = 0
            # LatticedChecker.test_count += 1
            # try_mkdir("data")
            # np.save("data/grasp_tool_img_%04d.npy"%(LatticedChecker.test_count),grasp_tool_img)
            # np.save("data/grasp_tar_img%04d.npy"%(LatticedChecker.test_count),grasp_tar_img)
            # np.save("data/grasp_obj_img%04d.npy"%(LatticedChecker.test_count),grasp_obj_img)

        with gtimer.block("query_wait_response"):
            arm_img = np.zeros(ARM_SHAPE + (1,))
            arm_img[np.unravel_index(arm_tar_idx, shape=ARM_SHAPE)] = 1
            grasp_img = np.stack([grasp_tool_img, grasp_obj_img, grasp_tar_img], axis=-1)
            res = self.query_wait_response(self.rconfig_dict[group_name].type.name,
                                           np.array([grasp_img]), np.array([arm_img]), np.array([rh_vals]),
                                           )[0]
        if DEBUG_LAT_FILT_LOG:
            save_scene(self.__class__.__name__, self.pscene, btf, Q_dict,
                       error_state=False, result=res, ignore=[igtem.name for igtem in ignore], **kwargs)
        return res[-1]>0.5

    def query_wait_response(self, robot_type_name, grasp_img_batch, arm_img_batch, rh_vals_batch):
        with self.lock:
            self.response_out_dict[robot_type_name][0] = False
            self.grasp_img_p_dict[robot_type_name][:] = grasp_img_batch[:]
            self.arm_img_p_dict[robot_type_name][:] = arm_img_batch[:]
            self.rh_vals_p_dict[robot_type_name][:] = rh_vals_batch[:]
            self.query_in_dict[robot_type_name][0] = True
            while not self.response_out_dict[robot_type_name][0]:
                time.sleep(SERVER_PERIOD)
            result = np.copy(self.result_p_dict[robot_type_name])
        return result

    def quit_shared_server(self, robot_type_name):
        self.query_quit_dict[robot_type_name][0] = True

    def __del__(self):
        for rname in self.server_on_me:
            self.quit_shared_server(rname)

