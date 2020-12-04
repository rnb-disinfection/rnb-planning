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

gtimer = GlobalTimer.instance()
gtimer.reset()

from pkg.controller.combined_robot import CombinedRobot, XYZ_RPY_ROBOTS_DEFAULT
from pkg.data_collecting.sampling import *

crob = CombinedRobot(connection_list=(False, False))

VISUALIZE = False
graph = None

DATA_PATH = "./data"
try: os.mkdir(DATA_PATH)
except: pass
import datetime
def get_now():
    return str(datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
CURRENT_PATH = os.path.join(DATA_PATH,get_now())
os.mkdir(CURRENT_PATH)

SAMPLED_DATA = defaultdict(dict)
GLOBAL_PARAMS = {}
__local_params_bak = __local_params = None
__local_params_bak = list(locals().keys())
####################################################################################
######################## Global parameter section ##################################
BASE_LINK = "base_link"
MAX_REACH_DICT = {'indy0': 1.5, 'panda1': 1.5}
REACH_OFFSET_DICT = {'indy0': (0,0,0.3), 'panda1': (0,0,0.3)}
GRIPPER_REFS = {"indy0":{"link_name": "indy0_tcp", "tcp_ref": [0,0,0.14], "depth_range": (-0.07,0.00), "width_range": (0.03,0.05), "bname": "grip0"},
                "panda1":{"link_name": "panda1_hand", "tcp_ref": [0,0,0.112], "depth_range": (-0.04,0.00), "width_range": (0.03,0.06), "bname": "grip1"}
               }
Nrbt = len(crob.robot_names) # 2
Njoints = len(crob.joint_names) # 13
WDH = (3,3,3)
WDH_MIN_RBT = (0.75,0.75,0.5)
WDH_MAX_RBT = (2.25,2.25,1.0)
L_CELL = 0.2
MIN_DIST_ROBOT = 1
No = 50
RATIO_COVER = 2.0
RATIO_DIMS = 2.0
LINK_COLL_MARGIN = 0.01
N_search = 20
N_retry = 1
GLOBAL_FILENAME = "global.json"
WORLD_FILENAME = "world.json"
SCENE_FILENAME = "scene.json"
SAMPLE_NUM_WORLD = 5
SAMPLE_NUM_SCENE = 5
SAMPLE_NUM_ACTION = 5
######################## Global parameter section ##################################
####################################################################################
__local_params = locals()
for k,v in __local_params.items():
    if k not in __local_params_bak:
        GLOBAL_PARAMS[k] = v
save_json(os.path.join(CURRENT_PATH, GLOBAL_FILENAME), GLOBAL_PARAMS)


CENTER = tuple(np.divide(WDH, 2, dtype=np.float))
Ws, Ds, Hs = WDH 
Nw, Nd, Nh = Nwdh = int(Ws/L_CELL), int(Ds/L_CELL), int(Hs/L_CELL)
L_MAX = L_CELL*RATIO_DIMS
print("scene size: {} ({},{},{})".format(Nw*Nd*Nh, Nw, Nd, Nh))


####################################################################################
############################## Sample world ########################################
####################################################################################
for _ in range(SAMPLE_NUM_WORLD):
    SAMPLED_DATA["WORLD"]["Trbt_dict"] = Trbt_dict= sample_Trbt(Nrbt, crob.robot_names, WDH_MIN_RBT, WDH_MAX_RBT, MIN_DIST_ROBOT)
    reach_center_dict = {k: tuple(np.add(v[0], REACH_OFFSET_DICT[k])) for k, v in Trbt_dict.items()}
    WORLD_PATH = os.path.join(CURRENT_PATH, "WORLD-"+get_now())
    os.mkdir(WORLD_PATH)
    save_json(os.path.join(WORLD_PATH, WORLD_FILENAME), SAMPLED_DATA["WORLD"])

    cam = None
    # set urdf
    xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(crob.robots_on_scene, Trbt_dict, crob.custom_limits, start_rviz=VISUALIZE)
    ghnd = GeometryHandle(urdf_content)
    time.sleep(2)


    # set graph
    if graph is None:
        graph = ConstraintGraph(ghnd=ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                                urdf_content=urdf_content, combined_robot=crob)
        graph.set_camera(cam)
        graph.set_cam_robot_collision(_add_cam_poles=False, color=(1,1,0,0.3))
        if VISUALIZE: graph.set_rviz()

        # start UI
        ui_broker = UIBroker(graph)
        ui_broker.start_server()

        # set rviz
        if VISUALIZE: graph.set_rviz(crob.home_pose)
        ui_broker.set_tables()
    else:
        graph.clear_markers()
        graph.clear_highlight()
        graph.ghnd.clear()
        graph.__init__(ghnd=graph.ghnd, urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                       urdf_content=urdf_content, combined_robot=crob)
        graph.set_cam_robot_collision(_add_cam_poles=False, color=(1,1,0,0.3))
        if VISUALIZE: graph.set_rviz()

    for gripper in GRIPPER_REFS.values():
        graph.register_binder(name=gripper['bname'], _type=FramedTool, point=gripper['tcp_ref'], rpy=[0,0,0], link_name=gripper['link_name'])
    graph.register_binder(name='base', _type=PlaceFrame, point=[0,0,0], rpy=[0,0,0], link_name=BASE_LINK)
    vtem = graph.ghnd.create_safe(name="virtual", gtype=GEOTYPE.SPHERE, link_name=BASE_LINK,
                                                 dims=(0,0,0), center=(0,0,0), rpy=(0,0,0), collision=False, display=False
                                                )
    graph.add_object("virtual",
                     SingleHandleObject(_object=vtem,
                                       action_point=FramePoint(name="point", _object=vtem, point=(0,0,0), rpy=(0,0,0), name_full=None)),
                     binding=("point", "base"))

    obj_list=[]
    col_obj_list=[]

    ####################################################################################
    ############################## Sample scene ########################################
    ####################################################################################
    for _ in range(SAMPLE_NUM_SCENE):
        SAMPLED_DATA["SCENE"]["Q_s"], _, _, _, _ = Q_s, links, link_verts, link_ctems, link_rads = sample_joint(graph)
        SAMPLED_DATA["SCENE"]["Q_s_dict"] = Q_s_dict = list2dict(Q_s, crob.joint_names)

        if VISUALIZE:
            graph.show_pose(Q_s)
            time.sleep(1)
            graph.show_pose(Q_s)
            time.sleep(1)
            graph.show_pose(Q_s)

        gtimer.tic("extract_available")
        # coll_boxes = get_colliding_cells(Nwdh, L_CELL, link_ctems, link_rads)
        free_boxes = get_reachable_cells(Nwdh, L_CELL, reach_center_dict, MAX_REACH_DICT)
        gtimer.toc("extract_available")

        # draw_cells(graph, "col_cell", coll_boxes, L_CELL, color=(0.9, 0, 0.2, 0.1))
        # remove_geometries_by_prefix(graph, "col_cell")
        # draw_cells(graph, "reachable", free_boxes, L_CELL, color=(0.2, 0.7, 0.2, 0.1))
        # remove_geometries_by_prefix(graph, "reachable")

        obj_dat = []
        for geo_gen in OBJ_GEN_LIST:
            obj_boxes = random.sample(free_boxes,No)
        #     draw_cells(graph, "obj_cell", obj_boxes, L_CELL, color=(0.2, 0.2, 0.7, 0.1))
        #     remove_geometries_by_prefix(graph, "obj_cell")
            for nbox in obj_boxes:
                gtype, dims, color = geo_gen(L_MAX)
        #         if gtype.name == "BOX":
        #             nbox_bak = nbox
        #         else:
        #             nbox = nbox_bak
                obj_dat.append({"nbox": nbox, "gtype": gtype.name, "dims": dims, "color": color,
                                "center": tuple(np.multiply(nbox, L_CELL)+np.random.random((3,))*L_MAX+0.5*L_CELL - 0.5*L_MAX),
                                "rpy": np.random.random((3,))*np.pi*2})


        # ## exclude colliding object

        # In[22]:


        __obj_dat=[]
        coll_list = []
        for odat in obj_dat:
            gtype, center, rpy, dims = getattr(GEOTYPE,odat['gtype']), odat['center'], odat['rpy'], odat['dims']
            ctem = getPointList(get_vertex_rows(gtype, center, rpy, dims))
            crad = (dims[0]/2) if gtype in [GEOTYPE.CAPSULE, GEOTYPE.CYLINDER, GEOTYPE.SPHERE] else 0
            if any(np.array(get_distance_list(ctem, link_ctems, crad, link_rads))<LINK_COLL_MARGIN):
                odat['color'] = (0.9, 0.0, 0.0, 0.3)
                coll_list.append(odat)
            else:
                __obj_dat.append(odat)
        SAMPLED_DATA["SCENE"]["obj_dat"] = obj_dat = __obj_dat

        SCENE_PATH = os.path.join(WORLD_PATH, "SCENE-"+get_now())
        os.mkdir(SCENE_PATH)
        save_json(os.path.join(SCENE_PATH, SCENE_FILENAME), SAMPLED_DATA["SCENE"])


        for obj in obj_list: graph.remove_geometry(obj)
        for odat in obj_dat:
            nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"], odat["color"], odat["center"], odat["rpy"]
            obj = graph.ghnd.create_safe(
                name="{}_{}_{}_{}".format(gtype.name,*nbox), link_name=BASE_LINK, gtype=gtype,
                center=center, rpy=rpy, dims=dims, color=color, display=True, collision=True, fixed=True)
            obj_list.append(obj)
            graph.add_marker(obj, vis=VISUALIZE)

        for obj in col_obj_list: graph.remove_geometry(obj)
        for odat in coll_list:
            nbox, gtype, dims, color, center, rpy = odat["nbox"], getattr(GEOTYPE, odat["gtype"]), odat["dims"], odat["color"], odat["center"], odat["rpy"]
            obj = graph.ghnd.create_safe(
                name="col_obj_{}_{}_{}".format(*nbox), link_name=BASE_LINK, gtype=gtype,
                center=center, rpy=rpy, dims=dims, color=color, display=True, collision=False, fixed=True)
            col_obj_list.append(obj)
            graph.add_marker(obj, vis=VISUALIZE)

        if "col_obj_list" in locals():
            for obj in col_obj_list: graph.remove_geometry(obj)

        if VISUALIZE: graph.set_rviz()

        ####################################################################################
        ############################## Sample action #######################################
        ####################################################################################
        for _ in range(SAMPLE_NUM_ACTION):
            dcol = DataCollector(graph, GRIPPER_REFS)
            if VISUALIZE: graph.set_rviz()

            # planners
            mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names, urdf_path=graph.urdf_path, urdf_content=graph.urdf_content,
                                  robot_names=graph.combined_robot.robot_names, binder_links=[v.object.link_name for v in graph.binder_dict.values()], ghnd=graph.ghnd)
            dual_mplan_dict = get_dual_planner_dict(GRIPPER_REFS, graph.ghnd, graph.urdf_content, graph.urdf_path, graph.link_names, graph.combined_robot.robot_names)

            # handover
            dcol.search_loop_mp(Q_s, obj_list, dual_mplan_dict, search_fun=dcol.handover_search, L_CELL=L_CELL, N_agents=None, N_search=N_search, N_retry=N_retry)
            save_json(os.path.join(SCENE_PATH, get_now()+".json"),  {idx: {k:v for k,v in item.items() if k !="trajectory"} for idx, item in dcol.snode_dict.items()})
            if VISUALIZE: dcol.play_all(graph, GRIPPER_REFS, "HANDOVER", test_pick, Q_s, remove_map=[[],[0,1]])

            # pick
            graph.set_planner(mplan)
            mplan.update(graph)
            dcol.search_loop_mp(Q_s, obj_list, mplan, search_fun=dcol.pick_search, L_CELL=L_CELL, N_agents=None, timeout=1, N_search=N_search, N_retry=N_retry)
            save_json(os.path.join(SCENE_PATH, get_now()+".json"),  {idx: {k:v for k,v in item.items() if k !="trajectory"} for idx, item in dcol.snode_dict.items()})
            if VISUALIZE: dcol.play_all(graph, GRIPPER_REFS, "PICK", test_pick, Q_s, remove_map=[[1],[0]])

            #place
            graph.set_planner(mplan)
            mplan.update(graph)
            dcol.search_loop_mp(Q_s, obj_list, mplan, search_fun=dcol.place_search, L_CELL=L_CELL, N_agents=None, timeout=1, N_search=N_search, N_retry=N_retry)
            save_json(os.path.join(SCENE_PATH, get_now()+".json"),  {idx: {k:v for k,v in item.items() if k !="trajectory"} for idx, item in dcol.snode_dict.items()})
            if VISUALIZE: dcol.play_all(graph, GRIPPER_REFS, "PLACE", test_place, Q_s, remove_map=[[],[0,1]])
