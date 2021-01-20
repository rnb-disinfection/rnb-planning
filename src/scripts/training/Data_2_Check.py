#!/usr/bin/env python
# coding: utf-8

# In[1]:


from __future__ import print_function
from IPython.core.display import display, HTML

display(HTML("<style>.container { width:90% !important; } </style>"))
import matplotlib.pyplot as plt
from pkg.marker_config import *
from pkg.tmp_framework import *
from pkg.constraint.constraint_action import *
from pkg.constraint.constraint_object import *
from pkg.constants import *
from pkg.utils.plot_utils import *
from pkg.utils.utils import *
from pkg.scene_builder import *
from pkg.ui.ui_broker import *
from pkg.controller.combined_robot import *
from pkg.controller.combined_robot import CombinedRobot, XYZ_RPY_ROBOTS_DEFAULT
from pkg.data_collecting.sampling import *

VISUALIZE = False
graph = None
SAMPLED_DATA = defaultdict(dict)
UPDATE_DAT = True

GLOBAL_FILENAME = "global.json"
WORLD_FILENAME = "world.json"
SCENE_FILENAME = "scene.json"
DATA_PATH = "./data"
gtimer = GlobalTimer.instance()

def main(dataset_list=None, N_retry_test = None):
    elog = Logger()
    crob = CombinedRobot(connection_list=(False, False))
    false_fail_accum = 0
    false_succ_accum = 0
    succ_accum = 0
    fail_accum = 0

    CHECK_DICT = {}

    # ## Load Global params
    if dataset_list is None:
        DATASET_LIST = sorted(filter(lambda x: x not in ["backup", "converted"], os.listdir(DATA_PATH)))
    else:
        DATASET_LIST = dataset_list
    for DATASET in DATASET_LIST:
        CHECK_DICT[DATASET] = {}
        CURRENT_PATH = os.path.join(DATA_PATH, DATASET)

        ## Load global params
        GLOBAL_PARAMS = load_json(os.path.join(CURRENT_PATH, GLOBAL_FILENAME))
        WDH = GLOBAL_PARAMS["WDH"]
        L_CELL = GLOBAL_PARAMS["L_CELL"]
        RATIO_DIMS = GLOBAL_PARAMS["RATIO_DIMS"]
        REACH_OFFSET_DICT = GLOBAL_PARAMS["REACH_OFFSET_DICT"]
        GRIPPER_REFS = GLOBAL_PARAMS["GRIPPER_REFS"]
        BASE_LINK = GLOBAL_PARAMS["BASE_LINK"]
        S_F_RATIO = GLOBAL_PARAMS["S_F_RATIO"]
        TIMEOUT = GLOBAL_PARAMS["TIMEOUT"]

        CENTER = tuple(np.divide(WDH, 2, dtype=np.float))
        Ws, Ds, Hs = WDH
        Nw, Nd, Nh = Nwdh = int(Ws / L_CELL), int(Ds / L_CELL), int(Hs / L_CELL)
        L_MAX = L_CELL * RATIO_DIMS
        print("scene size: {} ({},{},{})".format(Nw * Nd * Nh, Nw, Nd, Nh))

        # ## Load world
        WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))
        for WORLD in WORLD_LIST:
            CHECK_DICT[DATASET][WORLD] = {}
            WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
            SAMPLED_DATA["WORLD"] = load_json(os.path.join(WORLD_PATH, WORLD_FILENAME))
            Trbt_dict = SAMPLED_DATA["WORLD"]["Trbt_dict"]
            reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}

            cam = None
            # set urdf
            xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = reset_ghnd(crob.robots_on_scene, Trbt_dict,
                                                                               crob.custom_limits, start_rviz=VISUALIZE)
            ghnd = GeometryHandle(urdf_content)
            time.sleep(2)

            # set graph
            graph = TMPFramework(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                                    urdf_content=urdf_content, combined_robot=crob)
            graph.set_camera(cam)
            graph.set_cam_robot_collision(_add_cam_poles=False, color=(1, 1, 0, 0.3))
            if VISUALIZE: graphghnd.set_rviz()

            # start UI
            ui_broker = UIBroker.instance()
            ui_broker.initialize(graph)
            ui_broker.start_server()

            # set rviz
            if VISUALIZE: graphghnd.set_rviz(crob.home_pose)
            ui_broker.set_tables()

            for gripper in GRIPPER_REFS.values():
                graph.register_binder(name=gripper['bname'], _type=FramedTool, point=gripper['tcp_ref'], rpy=[0, 0, 0],
                                      link_name=gripper['link_name'])
            graph.register_binder(name='base', _type=PlaceFrame, point=[0, 0, 0], rpy=[0, 0, 0], link_name=BASE_LINK)
            vtem = graph.ghnd.create_safe(name="virtual", gtype=GEOTYPE.SPHERE, link_name=BASE_LINK,
                                          dims=(0, 0, 0), center=(0, 0, 0), rpy=(0, 0, 0), collision=False, display=False
                                          )
            graph.add_object("virtual",
                             SingleHandleObject(_object=vtem,
                                                action_point=FramePoint(name="point", _object=vtem, point=(0, 0, 0),
                                                                        rpy=(0, 0, 0), name_full=None)),
                             binding=("point", "base"))

            obj_list = []
            col_obj_list = []

            # ## Load scene
            SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
            for SCENE in SCENE_LIST:
                CHECK_DICT[DATASET][WORLD][SCENE] = {}
                SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
                SAMPLED_DATA["SCENE"] = load_json(os.path.join(SCENE_PATH, SCENE_FILENAME))
                Q_s = np.array(SAMPLED_DATA["SCENE"]["Q_s"])
                Q_s, links, link_verts, link_ctems, link_rads = sample_joint(graph, Q_s_loaded=Q_s)
                Q_s_dict = SAMPLED_DATA["SCENE"]["Q_s_dict"]
                obj_dat = SAMPLED_DATA["SCENE"]["obj_dat"]

                if VISUALIZE:
                    graph.show_pose(Q_s)
                    time.sleep(1)
                    graph.show_pose(Q_s)
                    time.sleep(1)
                    graph.show_pose(Q_s)
                for obj in obj_list: graph.remove_geometry(obj)
                for odat in obj_dat:
                    nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"], \
                                                            odat["color"], odat["center"], odat["rpy"]
                    obj = graph.ghnd.create_safe(
                        name="{}_{}_{}_{}".format(gtype.name, *nbox), link_name=BASE_LINK, gtype=gtype,
                        center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
                    obj_list.append(obj)
                    graph.add_marker(obj, vis=VISUALIZE)

                for obj in col_obj_list: graph.remove_geometry(obj)

                if VISUALIZE: graphghnd.set_rviz()
                dcol = DataCollector(graph, GRIPPER_REFS, S_F_RATIO=S_F_RATIO)
                if VISUALIZE: graphghnd.set_rviz()

                # planners
                mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names, urdf_path=graph.urdf_path,
                                      urdf_content=graph.urdf_content,
                                      robot_names=graph.combined_robot.robot_names,
                                      binder_links=[v.geometry.link_name for v in graph.binder_dict.values()],
                                      ghnd=graph.ghnd)
                dual_mplan_dict = get_dual_planner_dict(GRIPPER_REFS, graph.ghnd, graph.urdf_content, graph.urdf_path,
                                                        graph.link_names, graph.combined_robot.robot_names)

                # ## Load action
                ACTION_LIST = sorted(filter(lambda x: x != SCENE_FILENAME, os.listdir(SCENE_PATH)))
                for ACTION in ACTION_LIST:
                    print("[BEGIN] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION] = {}
                    snode_dict_bak = {int(k): v for k, v in load_json(os.path.join(SCENE_PATH, ACTION)).items()}
                    dcol.snode_dict = dcol.manager.dict()
                    for k, v in snode_dict_bak.items():
                        dcol.snode_dict[k] = deepcopy(v)
                    snode_keys = sorted(snode_dict_bak.keys())

                    if snode_keys:
                        sk0 = snode_keys[0]
                        rname, tar, inhand_coll, obj_coll = \
                            snode_dict_bak[sk0]["rname1"], snode_dict_bak[sk0]["rname2"], \
                            snode_dict_bak[sk0]["obj1"]["collision"], snode_dict_bak[sk0]["obj2"]["collision"]
                    else:
                        print("[END] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
                        print("No Item")
                        continue

                    dcol.check_loop_mp(test_full_mp, GRIPPER_REFS=GRIPPER_REFS,  Q_s=Q_s, dual_mplan_dict=dual_mplan_dict, mplan=mplan, N_retry=N_retry_test, timeout=TIMEOUT)
                    succ_old_list = np.array([snode_dict_bak[skey]["success"] for skey in snode_keys])
                    succ_now_list = np.array([dcol.snode_dict[skey]["succ_count"]>0 for skey in snode_keys])

                    if rname and tar:  # handover case
                        action_type = "HANDOVER"
                    elif inhand_coll:  # place case
                        action_type = "PLACE"
                    elif obj_coll:  # pick case
                        action_type = "PICK"
                    else:
                        raise (RuntimeError("non-implemented case"))

                    check_results = succ_old_list==succ_now_list
                    print(succ_old_list)
                    print(succ_now_list)
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["action_type"] = action_type
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["check_ratio"] = np.mean(check_results)
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["succ_corrects"] = np.mean(
                        check_results[np.where(succ_old_list)])
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["fail_corrects"] = np.mean(
                        check_results[np.where(np.logical_not(succ_old_list))])
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["num"] = len(check_results)
                    
                    false_fail_accum += np.sum(np.logical_not(check_results[np.where(np.logical_not(succ_old_list))]))
                    false_succ_accum += np.sum(np.logical_not(check_results[np.where(succ_old_list)]))
                    succ_accum += np.sum(succ_old_list)
                    fail_accum += np.sum(np.logical_not(succ_old_list))
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["false_fail_accum"] = false_fail_accum
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["false_succ_accum"] = false_succ_accum
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["succ_accum"] = succ_accum
                    CHECK_DICT[DATASET][WORLD][SCENE][ACTION]["fail_accum"] = fail_accum

                    if UPDATE_DAT:
                        for isk, skey in zip(range(len(snode_keys)), snode_keys):
                            if not succ_old_list[isk] and succ_now_list[isk]:
                                snode_dict_bak[skey]["success"] = bool(succ_now_list[isk])
                        save_json(os.path.join(SCENE_PATH, ACTION), snode_dict_bak)

                    if VISUALIZE: graphghnd.set_rviz()

                    print("[END] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
                    elog.log(CHECK_DICT[DATASET][WORLD][SCENE][ACTION], "{}-{}".format(SCENE, ACTION), print_now=False)
                    for k, v in CHECK_DICT[DATASET][WORLD][SCENE][ACTION].items():
                        print("{}: {}".format(k, str(v)))
    xcustom.clear()
    rospy.signal_shutdown("ALL FINISHED")

if __name__ == "__main__":
    import Data_3_Convert
    main(['20201218-024611'])
    Data_3_Convert.main(['20201218-024611'])