#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#!/usr/bin/env python


# In[1]:


from __future__ import print_function
from IPython.core.display import display, HTML

display(HTML("<style>.container { width:90% !important; } </style>"))
import matplotlib.pyplot as plt
from pkg.marker_config import *
from pkg.constraint_graph import *
from pkg.constraint.constraint_action import *
from pkg.constraint.constraint_object import *
from pkg.constants import *
from pkg.utils.plot_utils import *
from pkg.utils.utils import *
from pkg.environment_builder import *
from pkg.ui.ui_broker import *
from pkg.controller.combined_robot import *
from pkg.controller.combined_robot import CombinedRobot, XYZ_RPY_ROBOTS_DEFAULT
from pkg.data_collecting.sampling import *

gtimer = GlobalTimer.instance()

VISUALIZE = False
graph = None
SAMPLED_DATA = defaultdict(dict)
UPDATE_DAT = True

GLOBAL_FILENAME = "global.json"
WORLD_FILENAME = "world.json"
SCENE_FILENAME = "scene.json"
DATA_PATH = "./data"


# DATASET_LIST = ['20201214-165211', '20201216-021416', '20201218-024611',
#                 '20201208-121454', '20201212-232318', '20201213-061207']
DATASET_LIST = ['20201218-024611']
N_retry_succ = 20
N_retry_fail = 3
MaxCountTest = 100


# In[ ]:


elog = Logger()
crob = CombinedRobot(connection_list=(False, False))


# In[ ]:


dual_mplan_dict = None
mplan = None


# ## Full test

# In[ ]:


def FULL_TEST_SCENE(DATASET, WORLD, SCENE, ACTION):
    dual_mplan_dict = None
    mplan = None
    data_list = []
    N_data = 0
    # ## Load Global params
    # for DATASET in DATASET_LIST:
    CURRENT_PATH = os.path.join(DATA_PATH, DATASET)
    # ## Load world
    WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))
    #     for WORLD in WORLD_LIST:
    WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
    # ## Load scene
    SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
    #             for SCENE in SCENE_LIST:
    SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
    # ## Load action
    ACTION_LIST = sorted(filter(lambda x: x != SCENE_FILENAME, os.listdir(SCENE_PATH)))
    #                 for ACTION in ACTION_LIST:
    N_action = len(load_json(os.path.join(SCENE_PATH, ACTION)))
    N_data += N_action
    for i_act in range(N_action):
        data_list.append((DATASET, WORLD, SCENE, ACTION, i_act))

    MaxCountTest = len(data_list)

    CHECK_LIST = []
    FAILURE_LIST = []
    for data_tuple in data_list:
        DATASET, WORLD, SCENE, ACTION, i_act = data_tuple
        test_results = []
        N_retry = N_retry_fail
        i_rep = 0
        while (i_rep < N_retry):
            i_rep += 1
            #  GLOBAL
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

            # WORLD
            WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
            SAMPLED_DATA["WORLD"] = load_json(os.path.join(WORLD_PATH, WORLD_FILENAME))
            Trbt_dict = SAMPLED_DATA["WORLD"]["Trbt_dict"]
            reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}

            cam = None
            # set urdf
            xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, Trbt_dict,
                                                                               crob.custom_limits, start_rviz=VISUALIZE)
            ghnd = GeometryHandle(urdf_content)
            if VISUALIZE: time.sleep(2)

            # set graph
            graph = ConstraintGraph(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                                    urdf_content=urdf_content, combined_robot=crob)
            graph.set_camera(cam)
            graph.set_cam_robot_collision(_add_cam_poles=False, color=(1, 1, 0, 0.3))
            if VISUALIZE: graph.set_rviz()

            # start UI
            ui_broker = UIBroker.instance()
            ui_broker.initialize(graph)
            ui_broker.start_server()

            # set rviz
            if VISUALIZE: graph.set_rviz(crob.home_pose)
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

            # SCENE
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
                nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"],                                                         odat["color"], odat["center"], odat["rpy"]
                obj = graph.ghnd.create_safe(
                    name="{}_{}_{}_{}".format(gtype.name, *nbox), link_name=BASE_LINK, gtype=gtype,
                    center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
                obj_list.append(obj)
                graph.add_marker(obj, vis=VISUALIZE)

            for obj in col_obj_list: graph.remove_geometry(obj)

            if VISUALIZE: graph.set_rviz()
            dcol = DataCollector(graph, GRIPPER_REFS, S_F_RATIO=S_F_RATIO)
            if VISUALIZE: graph.set_rviz()

            # ACTION
            print("[BEGIN] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
            snode_dict_bak = {int(k): v for k, v in load_json(os.path.join(SCENE_PATH, ACTION)).items()}
            dcol.snode_dict = dcol.manager.dict()
            for k, v in snode_dict_bak.items():
                dcol.snode_dict[k] = deepcopy(v)
            snode_keys = sorted(snode_dict_bak.keys())

            if i_act not in snode_keys:
                raise(RuntimeError("i_act not in snode_keys"))

            # snode
            snode = dcol.snode_dict[i_act]
            rname, inhand, obj, tar, dims_bak, color_bak, succ, _ = load_manipulation_from_dict(snode,
                                                                                                graph.ghnd)

            # action type
            if rname and tar:  # handover case
                action_type = "HANDOVER"
            elif inhand.collision:  # place case
                action_type = "PLACE"
            elif obj.collision:  # pick case
                action_type = "PICK"
            else:
                raise (RuntimeError("non-implemented case"))

            # planners
            if action_type == "HANDOVER":
                if dual_mplan_dict is not None:
                    for dplan in dual_mplan_dict.values():
                        dplan.planner.clear_scene()
                        dplan.planner.terminate()
                dual_mplan_dict = get_dual_planner_dict(GRIPPER_REFS, graph.ghnd, graph.urdf_content, graph.urdf_path,
                                                        graph.link_names, graph.combined_robot.robot_names)
            else:
                if mplan is not None:
                    mplan.planner.clear_scene()
                    mplan.planner.terminate()
                mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names, urdf_path=graph.urdf_path,
                                      urdf_content=graph.urdf_content,
                                      robot_names=graph.combined_robot.robot_names,
                                      binder_links=[v.object.link_name for v in graph.binder_dict.values()],
                                      ghnd=graph.ghnd)

            # plan
            if rname and tar:  # handover case
                remove_map = [[], [0, 1]]
                action_type = "HANDOVER"
                trajectory, Q_last, error, success_now = test_handover(graph, GRIPPER_REFS, rname, inhand,
                                                                       obj, tar, Q_s,
                                                                       dual_mplan_dict[(rname, tar)], timeout=TIMEOUT)
            elif inhand.collision:  # place case
                remove_map = [[], [0, 1]]
                action_type = "PLACE"
                trajectory, Q_last, error, success_now = test_place(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                    tar, Q_s, mplan, timeout=TIMEOUT)
            elif obj.collision:  # pick case
                remove_map = [[1], [0]]
                action_type = "PICK"
                trajectory, Q_last, error, success_now = test_pick(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                   tar, Q_s, mplan, timeout=TIMEOUT)
            else:
                remove_map = [[], [0,1]]
                action_type = "None"
                raise (RuntimeError("non-implemented case"))

            test_results.append(success_now)

            print("[END] {} - {} - {} - {} - {} ({}/{}) = {} / {}".format(DATASET, WORLD, SCENE, ACTION, i_act, i_rep, N_retry, success_now, succ), end='\r')
            xcustom.clear()
            check_ratio = int((np.mean(np.array(np.array(test_results) == succ, dtype=np.float)))*100)
            if i_rep > N_retry_fail and success_now: ## sucess detected in over-count verfication process. break now
                print("========================================================")
                print("Success in over-loop: break")
                break
            if i_rep == N_retry:
                if succ:
                    if check_ratio==0 and N_retry==N_retry_fail:
                        print("")
                        print("========================================================")
                        print("Success failure: expand N_retry from {} to {}".format(N_retry, N_retry_succ))
                        N_retry = N_retry_succ


        CHECK_RES = {}
        CHECK_RES["action_type"] = action_type
        CHECK_RES["GT"] = succ
        check_ratio = int((np.mean(np.array(np.array(test_results) == succ, dtype=np.float)))*100)
        CHECK_RES["check_ratio"] = check_ratio
        print("")
        print("========================================================")
        print("[END] {} - {} - {} - {} - {}  = {}/{} ({}/{})".format(DATASET, WORLD, SCENE, ACTION, i_act, check_ratio, succ, len(CHECK_LIST), MaxCountTest), end='\r')
        CHECK_LIST. append((DATASET,WORLD, SCENE, ACTION, i_act, CHECK_RES))
        if succ:
            if check_ratio==0:
                FAILURE_LIST.append((DATASET,WORLD, SCENE, ACTION, i_act, "SUCCESS_FAIL"))
                print("========================================================")
                print("========================================================")
                print("=====================[SUCESS FAIL]========================")
                print("========================================================")
                print("========================================================")
        else:
            if check_ratio<0.5:
                FAILURE_LIST.append((DATASET,WORLD, SCENE, ACTION, i_act, "FAILURE_FAIL"))
                print("========================================================")
                print("========================================================")
                print("=====================[FAILURE FAIL]========================")
                print("========================================================")
                print("========================================================")
    if len(FAILURE_LIST)>len(CHECK_LIST)*0.2:
        raise(RuntimeError("TOO MANY ERROE IN ONE SCENE!"))
        
    succ_old_list = np.array([snode_dict_bak[skey]["success"] for skey in snode_keys])
    for context_fail in FAILURE_LIST:
        i_fail = context_fail[-2]
        print("FAIL: {} ({})".format(context_fail, snode_dict_bak[i_fail]["success"]))
        snode_dict_bak[i_fail]["success"] = not snode_dict_bak[i_fail]["success"]
    save_json(os.path.join(SCENE_PATH, ACTION), snode_dict_bak)


# In[ ]:





# ## SAMPLE TEST

# In[ ]:


MaxCountReached = 0

RESTART = True

while RESTART:
    RESTART = False

    data_list = []
    N_data = 0
    # ## Load Global params
    for DATASET in DATASET_LIST:
        CURRENT_PATH = os.path.join(DATA_PATH, DATASET)
        # ## Load world
        WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))
        for WORLD in WORLD_LIST:
                WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
                # ## Load scene
                SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
                for SCENE in SCENE_LIST:
                    SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
                    # ## Load action
                    ACTION_LIST = sorted(filter(lambda x: x != SCENE_FILENAME, os.listdir(SCENE_PATH)))
                    for ACTION in ACTION_LIST:
                        N_action = len(load_json(os.path.join(SCENE_PATH, ACTION)))
                        N_data += N_action
                        for i_act in range(N_action):
                            data_list.append((DATASET, WORLD, SCENE, ACTION, i_act))

    CHECK_LIST = []
    random.shuffle(data_list)
    for i_dat, data_tuple in enumerate(data_list[:MaxCountTest]):
        MaxCountReached = max(MaxCountReached, i_dat)
        DATASET, WORLD, SCENE, ACTION, i_act = data_tuple
        test_results = []
        N_retry = N_retry_fail
        i_rep = 0
        while (i_rep < N_retry):
            i_rep += 1
            #  GLOBAL
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

            # WORLD
            WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
            SAMPLED_DATA["WORLD"] = load_json(os.path.join(WORLD_PATH, WORLD_FILENAME))
            Trbt_dict = SAMPLED_DATA["WORLD"]["Trbt_dict"]
            reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}

            cam = None
            # set urdf
            xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, Trbt_dict,
                                                                               crob.custom_limits, start_rviz=VISUALIZE)
            ghnd = GeometryHandle(urdf_content)
            if VISUALIZE: time.sleep(2)

            # set graph
            graph = ConstraintGraph(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                                    urdf_content=urdf_content, combined_robot=crob)
            graph.set_camera(cam)
            graph.set_cam_robot_collision(_add_cam_poles=False, color=(1, 1, 0, 0.3))
            if VISUALIZE: graph.set_rviz()

            # start UI
            ui_broker = UIBroker.instance()
            ui_broker.initialize(graph)
            ui_broker.start_server()

            # set rviz
            if VISUALIZE: graph.set_rviz(crob.home_pose)
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

            # SCENE
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
                nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"],                                                         odat["color"], odat["center"], odat["rpy"]
                obj = graph.ghnd.create_safe(
                    name="{}_{}_{}_{}".format(gtype.name, *nbox), link_name=BASE_LINK, gtype=gtype,
                    center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
                obj_list.append(obj)
                graph.add_marker(obj, vis=VISUALIZE)

            for obj in col_obj_list: graph.remove_geometry(obj)

            if VISUALIZE: graph.set_rviz()
            dcol = DataCollector(graph, GRIPPER_REFS, S_F_RATIO=S_F_RATIO)
            if VISUALIZE: graph.set_rviz()

            # ACTION
            print("[BEGIN] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
            snode_dict_bak = {int(k): v for k, v in load_json(os.path.join(SCENE_PATH, ACTION)).items()}
            dcol.snode_dict = dcol.manager.dict()
            for k, v in snode_dict_bak.items():
                dcol.snode_dict[k] = deepcopy(v)
            snode_keys = sorted(snode_dict_bak.keys())

            if i_act not in snode_keys:
                raise(RuntimeError("i_act not in snode_keys"))

            # snode
            snode = dcol.snode_dict[i_act]
            rname, inhand, obj, tar, dims_bak, color_bak, succ, _ = load_manipulation_from_dict(snode,
                                                                                                graph.ghnd)

            # action type
            if rname and tar:  # handover case
                action_type = "HANDOVER"
            elif inhand.collision:  # place case
                action_type = "PLACE"
            elif obj.collision:  # pick case
                action_type = "PICK"
            else:
                raise (RuntimeError("non-implemented case"))

            # planners
            if action_type == "HANDOVER":
                if dual_mplan_dict is not None:
                    for dplan in dual_mplan_dict.values():
                        dplan.planner.clear_scene()
                        dplan.planner.terminate()
                dual_mplan_dict = get_dual_planner_dict(GRIPPER_REFS, graph.ghnd, graph.urdf_content, graph.urdf_path,
                                                        graph.link_names, graph.combined_robot.robot_names)
            else:
                if mplan is not None:
                    mplan.planner.clear_scene()
                    mplan.planner.terminate()
                mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names, urdf_path=graph.urdf_path,
                                      urdf_content=graph.urdf_content,
                                      robot_names=graph.combined_robot.robot_names,
                                      binder_links=[v.object.link_name for v in graph.binder_dict.values()],
                                      ghnd=graph.ghnd)

            # plan
            if rname and tar:  # handover case
                remove_map = [[], [0, 1]]
                action_type = "HANDOVER"
                trajectory, Q_last, error, success_now = test_handover(graph, GRIPPER_REFS, rname, inhand,
                                                                       obj, tar, Q_s,
                                                                       dual_mplan_dict[(rname, tar)], timeout=TIMEOUT)
            elif inhand.collision:  # place case
                remove_map = [[], [0, 1]]
                action_type = "PLACE"
                trajectory, Q_last, error, success_now = test_place(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                    tar, Q_s, mplan, timeout=TIMEOUT)
            elif obj.collision:  # pick case
                remove_map = [[1], [0]]
                action_type = "PICK"
                trajectory, Q_last, error, success_now = test_pick(graph, GRIPPER_REFS, rname, inhand, obj,
                                                                   tar, Q_s, mplan, timeout=TIMEOUT)
            else:
                remove_map = [[], [0,1]]
                action_type = "None"
                raise (RuntimeError("non-implemented case"))

            test_results.append(success_now)

            print("[END] {} - {} - {} - {} - {} ({}/{}) = {} / {}".format(DATASET, WORLD, SCENE, ACTION, i_act, i_rep, N_retry, success_now, succ), end='\r')
            xcustom.clear()
            check_ratio = int((np.mean(np.array(np.array(test_results) == succ, dtype=np.float)))*100)
            if i_rep > N_retry_fail and success_now: ## sucess detected in over-count verfication process. break now
                print("========================================================")
                print("Success in over-loop: break")
                break
            if i_rep == N_retry:
                if succ:
                    if check_ratio==0 and N_retry==N_retry_fail:
                        print("")
                        print("========================================================")
                        print("Success failure: expand N_retry from {} to {}".format(N_retry, N_retry_succ))
                        N_retry = N_retry_succ

        CHECK_RES = {}
        CHECK_RES["action_type"] = action_type
        CHECK_RES["GT"] = succ
        check_ratio = int((np.mean(np.array(np.array(test_results) == succ, dtype=np.float)))*100)
        CHECK_RES["check_ratio"] = check_ratio
        print("")
        print("========================================================")
        print("[END] {} - {} - {} - {} - {}  = {}/{} ({}/{}/{})".format(DATASET, WORLD, SCENE, ACTION, i_act, check_ratio, succ, i_dat, MaxCountReached, MaxCountTest), end='\r')
#         CHECK_LIST. append((DATASET,WORLD, SCENE, ACTION, i_act, CHECK_RES))
        if succ:
            if check_ratio==0:
#                 raise(RuntimeError("Success failure"))
                RESTART = True
        else:
            if check_ratio<0.5:
#                 raise(RuntimeError("Fail failure"))
                RESTART = True
        if RESTART:
            FULL_TEST_SCENE(DATASET, WORLD, SCENE, ACTION)
            break


# In[ ]:





# In[ ]:





# In[ ]:





# # Convert

# In[ ]:


#!/usr/bin/env python


# In[1]:


import matplotlib.pyplot as plt
from pkg.marker_config import *
from pkg.constraint_graph import *
from pkg.constraint.constraint_action import *
from pkg.constraint.constraint_object import *
from pkg.constants import *
from pkg.utils.plot_utils import *
from pkg.utils.utils import *
from pkg.environment_builder import *
from pkg.ui.ui_broker import *
from pkg.controller.combined_robot import *
from pkg.controller.combined_robot import CombinedRobot, XYZ_RPY_ROBOTS_DEFAULT
from pkg.data_collecting.sampling import *

VISUALIZE = False
graph = None
SAMPLED_DATA = defaultdict(dict)
UPDATE_DAT = True

# custom_xacro = '{}robots/dataset_2012/custom_robots.urdf.xacro'.format(TAMP_ETASL_DIR)
custom_xacro = None

GLOBAL_FILENAME = "global.json"
WORLD_FILENAME = "world.json"
SCENE_FILENAME = "scene.json"
DATA_PATH = "./data"
CONVERTED_PATH = "./data/converted"
try_mkdir(CONVERTED_PATH)

gtimer = GlobalTimer.instance()
elog = Logger()

crob = CombinedRobot(connection_list=(False, False))
CHECK_DICT = {}
# ## Load Global params
# DATASET_LIST = sorted(filter(lambda x: x not in ["backup", "converted", "converted_bak"], os.listdir(DATA_PATH)))
# DATASET_LIST = ['20201214-165211', '20201216-021416', '20201218-024611',
#                 '20201208-121454', '20201212-232318', '20201213-061207']
# DATASET_LIST = [ '20201212-232318']


# In[ ]:


data_list = []
N_data = 0
# ## Load Global params
for DATASET in DATASET_LIST:
    CURRENT_PATH = os.path.join(DATA_PATH, DATASET)
    # ## Load world
    WORLD_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(CURRENT_PATH)))
    for WORLD in WORLD_LIST:
            WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
            # ## Load scene
            SCENE_LIST = sorted(filter(lambda x: not x.endswith(".json"), os.listdir(WORLD_PATH)))
            for SCENE in SCENE_LIST:
                SCENE_PATH = os.path.join(WORLD_PATH, SCENE)
                # ## Load action
                ACTION_LIST = sorted(filter(lambda x: x != SCENE_FILENAME, os.listdir(SCENE_PATH)))
                for ACTION in ACTION_LIST:
                    N_action = len(load_json(os.path.join(SCENE_PATH, ACTION)))
                    N_data += N_action
                    for i_act in range(N_action):
                        data_list.append((DATASET, WORLD, SCENE, ACTION, i_act))


# In[ ]:


for i_dat, data_tuple in enumerate(data_list):
    DATASET, WORLD, SCENE, ACTION, i_act = data_tuple
    try_mkdir(os.path.join(CONVERTED_PATH, DATASET))
    try_mkdir(os.path.join(CONVERTED_PATH, DATASET, WORLD))
    try_mkdir(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE))

    #  GLOBAL
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

    # WORLD
    WORLD_PATH = os.path.join(CURRENT_PATH, WORLD)
    SAMPLED_DATA["WORLD"] = load_json(os.path.join(WORLD_PATH, WORLD_FILENAME))
    Trbt_dict = SAMPLED_DATA["WORLD"]["Trbt_dict"]
    print("[WORLD] -")
    print(Trbt_dict)
    reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}

    cam = None
    # set urdf
    xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, Trbt_dict,
                                                                       crob.custom_limits, start_rviz=VISUALIZE)
    ghnd = GeometryHandle(urdf_content)
    if VISUALIZE: time.sleep(2)

    # set graph
    graph = ConstraintGraph(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                            urdf_content=urdf_content, combined_robot=crob)
    graph.set_camera(cam)
    graph.set_cam_robot_collision(_add_cam_poles=False, color=(1, 1, 0, 0.3))
    if VISUALIZE: graph.set_rviz()

    # start UI
    ui_broker = UIBroker.instance()
    ui_broker.initialize(graph)
    ui_broker.start_server()

    # set rviz
    if VISUALIZE: graph.set_rviz(crob.home_pose)
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

    # SCENE
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
        nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"],                                                 odat["color"], odat["center"], odat["rpy"]
        obj = graph.ghnd.create_safe(
            name="{}_{}_{}_{}".format(gtype.name, *nbox), link_name=BASE_LINK, gtype=gtype,
            center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
        obj_list.append(obj)
        graph.add_marker(obj, vis=VISUALIZE)

    for obj in col_obj_list: graph.remove_geometry(obj)

    if VISUALIZE: graph.set_rviz()
    dcol = DataCollector(graph, GRIPPER_REFS, S_F_RATIO=S_F_RATIO)
    if VISUALIZE: graph.set_rviz()

#########################################################################
    # ## initialize scene params
    link_names = graph.link_names
    joint_names = crob.joint_names
    joint_num = crob.joint_num
    IGNORE_CTEMS = ["panda1_hand_Cylinder_1", 'panda1_hand_Cylinder_2']
    joint_index_dict = {joint.name:None for joint in urdf_content.joints}
    joint_index_dict.update({jname:idx for idx, jname in zip(range(joint_num), joint_names)})
    centers = get_centers(Nwdh, L_CELL)
    merge_pairs = get_merge_pairs(ghnd, BASE_LINK)
    merge_paired_ctems(graph=graph, merge_pairs=merge_pairs, VISUALIZE=VISUALIZE)
    for cname in IGNORE_CTEMS:
        graph.remove_geometry(graph.ghnd.NAME_DICT[cname])

    N_vtx_box = 3*8
    N_mask_box = 1
    N_joint_box = joint_num
    N_label_box = N_vtx_box+N_mask_box+N_joint_box

    N_vtx_cyl = 3*2+1
    N_mask_cyl = 1
    N_joint_cyl = joint_num
    N_label_cyl = N_vtx_cyl+N_mask_cyl+N_joint_cyl

    N_vtx_init = 3*8
    N_mask_init = 1
    N_joint_init = joint_num
    N_label_init = N_vtx_init+N_mask_init+N_joint_init

    N_vtx_goal = 3*8
    N_mask_goal = 1
    N_joint_goal = joint_num
    N_label_goal = N_vtx_goal+N_mask_goal+N_joint_goal

    N_joint_label_begin = N_label_box+N_label_cyl+N_label_init+N_label_goal

    N_joint_label = 6*joint_num

    N_joint_limits = 3*joint_num

    N_cell_label = N_joint_label_begin + N_joint_label + N_joint_limits

    gtimer.tic("test_links")
    Tlink_dict = {}
    chain_dict = {}
    Tj_arr = np.zeros((joint_num, 4, 4))
    Tj_inv_arr = np.zeros((joint_num, 4, 4))
    joint_axis_arr = np.zeros((joint_num,3))

    gtimer.tic("T_chain")
    Tlink_dict = build_T_chain(link_names, Q_s_dict, urdf_content)
    gtimer.toc("T_chain")

    for lname in link_names:
        gtimer.tic("get_chain")
        jnames = filter(lambda x: x in joint_names,
                        urdf_content.get_chain(root=BASE_LINK, tip=lname, joints=True, links=False))
        gtimer.toc("get_chain")
        chain_dict[lname] = [1 if pj in jnames else 0 for pj in joint_names]
    gtimer.toc("test_links")

    gtimer.tic("test_joints")
    for i_j, jname in zip(range(joint_num), joint_names):
        joint = urdf_content.joint_map[jname]
        lname = joint.child
        Tj_arr[i_j,:,:] = Tlink_dict[lname]
        Tj_inv_arr[i_j,:,:] = SE3_inv(Tlink_dict[lname])
        joint_axis_arr[i_j] = joint.axis
    gtimer.toc("test_joints")
    gtimer.tic("calc_cell")
    __p = centers.reshape(Nwdh+(1,3)) - Tj_arr[:,:3,3].reshape((1,1,1,joint_num,3))
    __w = np.sum(Tj_arr[:,:3,:3] * joint_axis_arr.reshape(joint_num, 1, 3), axis=-1).reshape((1,1,1,joint_num,3))
    __w = np.tile(__w, Nwdh+(1,1))
    __v = np.cross(__w, __p)
    xi = np.concatenate([__w, __v],axis=-1)
    gtimer.toc("calc_cell")
    gtimer.tic("calc_ctems")
    ctem_names, ctem_TFs, ctem_dims, ctem_cells, ctem_links, ctem_joints, ctem_types =             defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), defaultdict(list), dict()
    for ctem in ghnd:
        if not ctem.collision:
            continue
        key = gtype_to_otype(ctem.gtype).name
        ctem_names[key].append(ctem.name)
        Ttem = np.matmul(Tlink_dict[ctem.link_name], ctem.Toff)
        ctem_TFs[key].append(Ttem)
        ctem_dims[key].append(ctem.dims)
        ctem_cells[key].append(get_cell(Ttem[:3,3], L_CELL, Nwdh))
        ctem_links[key].append(ctem.link_name)
        ctem_joints[key].append(chain_dict[ctem.link_name])
        ctem_types[key] = ctem.gtype


    verts_dict = {}
    for key_cur in ctem_cells:
        ctem_TFs[key_cur] = np.array(ctem_TFs[key_cur])
        cell_array = np.array(ctem_cells[key_cur])
        all_rearranged = False
        while not all_rearranged:
            all_rearranged = True
            for cell in cell_array:
                idxset = np.where(np.all(cell_array==cell, axis=-1))[0]
                if len(idxset)>1:
                    all_rearranged = False
                    cell_array_bak = cell_array.copy()
                    cell_array = rearrange_cell_array(cell_array, idxset, L_CELL, Nwdh, ctem_TFs[key_cur], centers)
                    break
        ctem_cells[key_cur] = cell_array
        gtype = ctem_types[key_cur]
        TFs = ctem_TFs[key_cur]
        dims = np.array(ctem_dims[key_cur])
        default_vert = DEFAULT_VERT_DICT[gtype]
        verts_dim = default_vert.reshape((1,-1,3))*dims.reshape((-1,1,3))
        cell_array = ctem_cells[key_cur]
        verts = np.zeros_like(verts_dim)
        for iv in range(verts.shape[1]):
            verts[:,iv,:] = matmul_md(TFs[:,:3,:3], verts_dim[:,iv,:,np.newaxis])[:,:,0]+TFs[:,:3,3]
        verts_loc = (verts - (cell_array[:,np.newaxis,:]*L_CELL+L_CELL/2))
        verts_loc = verts_loc.reshape((verts_loc.shape[0], -1))
        if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER]:
            verts_loc = np.concatenate([verts_loc, dims[:,0:1]], axis=-1)
        verts_dict[key_cur] = verts_loc
    gtimer.toc("calc_ctems")

    ### initialize data volume
    scene_data = np.zeros(Nwdh+(N_cell_label,))
    ### put cell joint data
    scene_data[:,:,:,N_joint_label_begin:N_joint_label_begin+N_joint_label] = xi.reshape(Nwdh+(N_joint_label,))

    jtem_list = [graph.urdf_content.joint_map[jname] for jname in graph.joint_names]
    joint_limits = [jtem.limit.lower for jtem in jtem_list] + [jtem.limit.upper for jtem in jtem_list]
    scene_data[:,:,:,-N_joint_limits:] = Q_s.tolist() + joint_limits
    ### put cell item data
    for verts, cell, chain in zip(verts_dict["BOX"], ctem_cells["BOX"], ctem_joints["BOX"]):
        scene_data[cell[0],cell[1],cell[2],:N_vtx_box] = verts
        scene_data[cell[0],cell[1],cell[2],N_vtx_box:N_vtx_box+N_mask_box] = 1
        scene_data[cell[0],cell[1],cell[2],N_vtx_box+N_mask_box:N_vtx_box+N_mask_box+N_joint_box] = chain

    N_BEGIN_CYL = N_vtx_box+N_mask_box+N_joint_box
    for verts, cell, chain in zip(verts_dict["CYLINDER"], ctem_cells["CYLINDER"], ctem_joints["CYLINDER"]):
        scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL:N_BEGIN_CYL+N_vtx_cyl] = verts
        scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL+N_vtx_cyl:N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl] = 1
        scene_data[cell[0],cell[1],cell[2],N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl:N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl+N_joint_cyl] = chain
    save_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, "scene.pkl"), {"scene_data": scene_data, "ctem_names": ctem_names, "ctem_cells":ctem_cells})            

##############################################################

    # ACTION
    print("[BEGIN] {} - {} - {} - {} ===============".format(DATASET, WORLD, SCENE, ACTION))
    snode_dict_bak = {int(k): v for k, v in load_json(os.path.join(SCENE_PATH, ACTION)).items()}
    dcol.snode_dict = dcol.manager.dict()
    for k, v in snode_dict_bak.items():
        dcol.snode_dict[k] = deepcopy(v)
    snode_keys = sorted(snode_dict_bak.keys())

    if i_act not in snode_keys:
        raise(RuntimeError("i_act not in snode_keys"))


    # snode
    snode = dcol.snode_dict[i_act]
    rname, inhand, obj, tar, dims_bak, color_bak, succ, _ = load_manipulation_from_dict(snode,
                                                                                        graph.ghnd)


    # action type
    if rname and tar:  # handover case
        action_type = "HANDOVER"
    elif inhand.collision:  # place case
        action_type = "PLACE"
    elif obj.collision:  # pick case
        action_type = "PICK"
    else:
        raise (RuntimeError("non-implemented case"))


##############################################################
    init_box_dat=get_cell_data(inhand, L_CELL, Nwdh, Tlink_dict, chain_dict, gtype=GEOTYPE.BOX)
    goal_box_dat=get_cell_data(obj, L_CELL, Nwdh, Tlink_dict, chain_dict, gtype=GEOTYPE.BOX)

    if rname and tar:  # handover case
        remove_map = [[], [0, 1]]
        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict)) for ctem in [inhand, obj] if ctem.collision]
        recover_tems = []
        action_type = "HANDOVER"
    elif inhand.collision:  # place case
        remove_map = [[], [0, 1]]
        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict)) for ctem in [inhand, obj] if ctem.collision]
        recover_tems = []
        action_type = "PLACE"
    elif obj.collision:  # pick case
        remove_map = [[1], [0]]
        col_tem_dats=[(ctem.name, gtype_to_otype(ctem.gtype).name, get_cell_data(ctem, L_CELL, Nwdh, Tlink_dict, chain_dict,
                                                cell=ctem_cells[gtype_to_otype(ctem.gtype).name][ctem_names[gtype_to_otype(ctem.gtype).name].index(ctem.name)])) for ctem in [inhand, obj] if ctem.collision]
        recover_tems = [obj]
        action_type = "PICK"
    else:
        remove_map = [[], [0,1]]
        col_tem_dats = []
        recover_tems = []
        action_type = "None"
        raise (RuntimeError("non-implemented case"))
    ### put init, goal item data
    N_BEGIN_INIT = N_BEGIN_CYL+N_vtx_cyl+N_mask_cyl+N_joint_cyl
    cell, verts, chain = init_box_dat

    N_BEGIN_GOAL = N_BEGIN_INIT+N_vtx_init+N_mask_init+N_joint_init
    cell, verts, chain = goal_box_dat

    ### add/replace collilsion object
    ctem_dat_list=[]
    for cname, ctype, cdat in col_tem_dats:
        cell, verts, chain = cdat
        if ctype == "BOX":
            N_BEGIN_REP, N_vtx, N_mask, N_joint = 0, N_vtx_box, N_mask_box, N_joint_box
        elif ctype == "CYLINDER":
            N_BEGIN_REP, N_vtx, N_mask, N_joint = N_BEGIN_CYL, N_vtx_cyl, N_mask_cyl, N_joint_cyl
        else:
            raise(RuntimeError("Non considered shape key"))
        if scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask-1]:
            if cname not in ctem_names[ctype]:
                near_range = np.clip(
                    ((cell[0]-1,cell[0]+1),(cell[1]-1,cell[1]+1),(cell[2]-1,cell[2]+1)),
                    [[0,0]]*3, np.transpose([Nwdh]*2)-1)
                marks_near = scene_data[near_range[0][0]:near_range[0][1]+1,near_range[1][0]:near_range[1][1]+1,near_range[2][0]:near_range[2][1]+1,N_BEGIN_REP+N_vtx+N_mask-1]
                marks_flt = marks_near.flatten()
                idx_free = np.where(np.logical_not(marks_flt))[0]
                near_cell_min = np.subtract(near_range,np.array(cell)[:,np.newaxis])[:,0]
                cells_near = get_centers(marks_near.shape, 1)-0.5+near_cell_min
                centers_cells = cells_near*L_CELL
                centers_flt = centers_cells.reshape((-1,3))[idx_free]

                if ctype == "CYLINDER":
                    verts_res = verts[:-1].reshape((-1,3))
                else:
                    verts_res = verts.reshape((-1,3))
                center_loc = np.mean(verts_res, axis=0)
                if len(idx_free)==0:
                    raise (RuntimeError("No free cell near {}".format(cname)))
                idx_cell = idx_free[np.argmin(np.linalg.norm(centers_flt-center_loc, axis=-1))]
                cell_new = cells_near.reshape((-1,3))[idx_cell] + cell
                diff = np.subtract(cell, cell_new)*L_CELL
                if ctype == "CYLINDER":
                    verts[:-1] = (verts_res + diff).flatten()
                else:
                    verts = (verts_res + diff).flatten()
                cell_bak = deepcopy(cell)
                cell = tuple(cell_new.astype(np.int))
                if scene_data[cell[0],cell[1],cell[2],N_BEGIN_REP+N_vtx+N_mask-1]:
                    print("wrong")
                    raise(RuntimeError("collision item cell occupied"))
        ctem_dat_list.append((cname, ctype, cell, verts, chain))
    save_pickle(os.path.join(CONVERTED_PATH, DATASET, WORLD, SCENE, ACTION.replace(".json", "-%03d.pkl"%i_act)), 
                {"init_box_dat":init_box_dat, "goal_box_dat":goal_box_dat, "ctem_dat_list":ctem_dat_list, "success":succ, "skey": i_act})
    xcustom.clear()
    print("DONE: {}/{}".format(i_dat, N_data))


# In[ ]:





# In[ ]:





# In[ ]:




